"""Standalone Zenoh-to-Unitree Go2 control node."""

from __future__ import annotations

import argparse
import logging
import signal
import time
from collections.abc import Iterator, Sequence
from contextlib import ExitStack, contextmanager
from dataclasses import dataclass
from pathlib import Path
from threading import Condition, Event, Lock
from types import FrameType
from typing import Any, Protocol

import zenoh
from pydantic import ValidationError

from config import NodeConfig, load_node_config
from controller import (
    JSON_ENCODING,
    CommandBuffer,
    Controller,
    Keyspace,
    NodeState,
    PostureCommand,
    RobotObservation,
    SportClientProtocol,
    VelocityCommand,
    decode_command,
)

DEFAULT_NODE_CONFIG_PATH = Path("config/node-config.json5")
DEFAULT_ZENOH_CONFIG_PATH = Path("config/zenoh-config.json5")
STATE_POLL_SECONDS = 0.05
LOGGER = logging.getLogger(__name__)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="unitree-go2-zenoh-node",
        description="Forward State-authorized Zenoh commands to one Unitree Go2.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--node-config",
        type=Path,
        default=DEFAULT_NODE_CONFIG_PATH,
        help="path to the node JSON5 configuration",
    )
    parser.add_argument(
        "--zenoh-config",
        type=Path,
        default=DEFAULT_ZENOH_CONFIG_PATH,
        help="path to the Zenoh JSON5 configuration",
    )
    return parser


class TimeSpecLike(Protocol):
    @property
    def sec(self) -> int: ...

    @property
    def nanosec(self) -> int: ...


class SportModeStateLike(Protocol):
    @property
    def stamp(self) -> TimeSpecLike: ...

    @property
    def error_code(self) -> int: ...

    @property
    def mode(self) -> int: ...

    @property
    def velocity(self) -> Sequence[float]: ...

    @property
    def yaw_speed(self) -> float: ...


def to_observation(
    message: SportModeStateLike,
    *,
    received_at: float,
) -> RobotObservation:
    velocity = tuple(float(value) for value in message.velocity)
    if len(velocity) < 3:
        raise ValueError("SportModeState.velocity must contain three components")
    stamp = message.stamp
    return RobotObservation(
        received_at=received_at,
        stamp_sec=int(stamp.sec),
        stamp_nanosec=int(stamp.nanosec),
        error_code=int(message.error_code),
        mode=int(message.mode),
        velocity=(velocity[0], velocity[1], velocity[2]),
        yaw_speed=float(message.yaw_speed),
    )


@dataclass(frozen=True, slots=True)
class StateEvent:
    observation: RobotObservation | None = None
    error: str | None = None


class StateInbox:
    """Expose only the newest unprocessed State to the control loop."""

    def __init__(self) -> None:
        self._condition = Condition()
        self._revision = 0
        self._event: StateEvent | None = None

    def put(self, event: StateEvent) -> None:
        with self._condition:
            self._revision += 1
            self._event = event
            self._condition.notify()

    def wait_after(
        self,
        revision: int,
        *,
        timeout: float,
    ) -> tuple[int, StateEvent] | None:
        with self._condition:
            self._condition.wait_for(
                lambda: self._revision > revision,
                timeout=timeout,
            )
            if self._revision <= revision or self._event is None:
                return None
            return (self._revision, self._event)


class ZenohStateBus:
    """Own Zenoh command ingress and the single public State key."""

    def __init__(
        self,
        session: zenoh.Session,
        robot_key: str,
        commands: CommandBuffer,
    ) -> None:
        self._session = session
        self._keyspace = Keyspace(robot_key)
        self._commands = commands
        self._payload: str | None = None
        self._payload_lock = Lock()
        self._command_lock = Lock()
        self._resources = ExitStack()

    def __enter__(self) -> ZenohStateBus:
        try:
            self._resources.enter_context(
                self._session.declare_subscriber(
                    self._keyspace.command,
                    self._on_command,
                )
            )
            self._resources.enter_context(
                self._session.declare_queryable(
                    self._keyspace.state,
                    self._reply,
                    complete=True,
                )
            )
        except Exception:
            self._resources.close()
            raise
        return self

    def __exit__(self, *_exc_info: object) -> None:
        self._resources.close()

    def publish(self, state: NodeState) -> None:
        payload = state.model_dump_json()
        with self._payload_lock:
            self._payload = payload
        self._session.put(
            self._keyspace.state,
            payload,
            encoding=JSON_ENCODING,
        )

    def _reply(self, query: zenoh.Query) -> None:
        with query:
            with self._payload_lock:
                payload = self._payload
            if payload is not None:
                query.reply(
                    self._keyspace.state,
                    payload,
                    encoding=JSON_ENCODING,
                )

    def _on_command(self, sample: zenoh.Sample) -> None:
        with self._command_lock:
            received_at = time.monotonic()
            try:
                command = decode_command(sample.payload.to_bytes())
            except (UnicodeDecodeError, ValidationError, ValueError) as error:
                LOGGER.warning("Rejected invalid Zenoh command: %s", error)
                return
            if isinstance(command, VelocityCommand):
                accepted = self._commands.update_velocity(
                    command,
                    received_at=received_at,
                )
            else:
                assert isinstance(command, PostureCommand)
                accepted = self._commands.update_posture(
                    command,
                    received_at=received_at,
                )
            if not accepted:
                LOGGER.debug("Ignored command while physical State is Unknown")


def create_unitree_resources(
    config: NodeConfig,
) -> tuple[SportClientProtocol, Any]:
    from unitree_sdk2py.core.channel import (
        ChannelFactoryInitialize,
        ChannelSubscriber,
    )
    from unitree_sdk2py.go2.sport.sport_client import SportClient
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_

    ChannelFactoryInitialize(
        config.dds.domain_id,
        config.dds.network_interface,
    )
    client = SportClient()
    client.SetTimeout(config.dds.rpc_timeout_seconds)
    client.Init()
    subscriber = ChannelSubscriber(
        config.dds.sport_mode_state_topic,
        SportModeState_,
    )
    return client, subscriber


@contextmanager
def stop_on_signals(stop_event: Event) -> Iterator[None]:
    previous_handlers: dict[int, Any] = {}

    def request_stop(_signum: int, _frame: FrameType | None) -> None:
        stop_event.set()

    for signum in (signal.SIGINT, signal.SIGTERM):
        previous_handlers[signum] = signal.getsignal(signum)
        signal.signal(signum, request_stop)
    try:
        yield
    finally:
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)


def process_state_event(controller: Controller, event: StateEvent) -> None:
    if event.observation is not None:
        controller.on_state(event.observation)
    else:
        controller.reject_state(event.error or "invalid SportModeState")


def run(zenoh_config: zenoh.Config, node_config: NodeConfig) -> None:
    zenoh.init_log_from_env_or("error")
    commands = CommandBuffer()
    states = StateInbox()
    stop_event = Event()
    client, subscriber = create_unitree_resources(node_config)

    def on_state(message: SportModeStateLike) -> None:
        try:
            event = StateEvent(
                observation=to_observation(
                    message,
                    received_at=time.monotonic(),
                )
            )
        except (TypeError, ValueError) as error:
            event = StateEvent(error=f"Invalid SportModeState: {error}")
        states.put(event)

    with (
        zenoh.open(zenoh_config) as session,
        ZenohStateBus(session, node_config.robot_key, commands) as state_bus,
        stop_on_signals(stop_event),
    ):
        controller = Controller(
            client,
            commands,
            state_bus,
            maximum_state_age_seconds=node_config.state.maximum_age_seconds,
            linear_velocity_quiescent_threshold=(
                node_config.state.linear_velocity_quiescent_threshold
            ),
            yaw_speed_quiescent_threshold=(
                node_config.state.yaw_speed_quiescent_threshold
            ),
            velocity_deadman_seconds=node_config.control.velocity_deadman_seconds,
        )
        subscriber.Init(on_state, queueLen=1)
        revision = 0
        started = False
        next_heartbeat = time.monotonic()
        try:
            startup_deadline = (
                time.monotonic() + node_config.state.startup_timeout_seconds
            )
            while not stop_event.is_set() and not controller.has_state:
                remaining = startup_deadline - time.monotonic()
                if remaining <= 0:
                    raise RuntimeError("Timed out waiting for SportModeState")
                received = states.wait_after(
                    revision,
                    timeout=min(STATE_POLL_SECONDS, remaining),
                )
                if received is not None:
                    revision, event = received
                    process_state_event(controller, event)

            started = controller.has_state
            LOGGER.info("Listening for commands on %s/command", node_config.robot_key)
            while not stop_event.is_set():
                received = states.wait_after(revision, timeout=STATE_POLL_SECONDS)
                if received is not None:
                    revision, event = received
                    process_state_event(controller, event)
                controller.expire_state()
                now = time.monotonic()
                if now >= next_heartbeat:
                    controller.publish()
                    next_heartbeat = now + node_config.state_heartbeat_seconds
        finally:
            if started:
                controller.begin_shutdown()
                shutdown_deadline = (
                    time.monotonic() + node_config.control.shutdown_timeout_seconds
                )
                while (
                    not controller.shutdown_complete
                    and time.monotonic() < shutdown_deadline
                ):
                    received = states.wait_after(
                        revision,
                        timeout=STATE_POLL_SECONDS,
                    )
                    if received is not None:
                        revision, event = received
                        process_state_event(controller, event)
                shutdown_error = (
                    None
                    if controller.shutdown_complete
                    else "Shutdown timed out before State confirmed down"
                )
                controller.finish_shutdown(shutdown_error)
            subscriber.Close()


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")
    try:
        node_config = load_node_config(args.node_config)
        zenoh_config = zenoh.Config.from_file(args.zenoh_config)
        run(zenoh_config, node_config)
    except KeyboardInterrupt:
        LOGGER.info("Stopped")
    except (OSError, RuntimeError, ValueError, ValidationError, zenoh.ZError) as error:
        LOGGER.error("%s", error)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
