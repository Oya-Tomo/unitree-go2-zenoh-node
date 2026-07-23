"""Standalone Zenoh-to-Unitree Go2 control node."""

from __future__ import annotations

import argparse
import logging
import math
import signal
import time
from collections.abc import Iterator, Sequence
from contextlib import ExitStack, contextmanager
from pathlib import Path
from threading import Event, Thread
from types import FrameType
from typing import Any, Protocol

import zenoh
from pydantic import ValidationError

from config import NodeConfig, load_node_config
from controller import (
    JSON_ENCODING,
    Controller,
    Keyspace,
    PostureCommand,
    RobotObservation,
    SportClientProtocol,
    VelocityCommand,
    decode_command,
)

DEFAULT_NODE_CONFIG_PATH = Path("config/node-config.json5")
DEFAULT_ZENOH_CONFIG_PATH = Path("config/zenoh-config.json5")
CONTROL_WAIT_MAX_SECONDS = 0.05
PUBLISHER_JOIN_TIMEOUT_SECONDS = 1.0
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
        state_machine_code=int(message.error_code),
        mode=int(message.mode),
        velocity=(velocity[0], velocity[1], velocity[2]),
        yaw_speed=float(message.yaw_speed),
    )


class ZenohStateBus:
    """Own Zenoh command ingress, queries, and periodic State publication."""

    def __init__(
        self,
        session: zenoh.Session,
        zenoh_key_prefix: str,
        controller: Controller,
        *,
        state_publish_frequency_hz: float,
    ) -> None:
        self._session = session
        self._keyspace = Keyspace(zenoh_key_prefix)
        self._controller = controller
        self._state_publish_period_seconds = 1.0 / state_publish_frequency_hz
        self._resources = ExitStack()
        self._publisher: zenoh.Publisher | None = None
        self._publisher_stop = Event()
        self._publisher_thread: Thread | None = None

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
            self._publisher = self._resources.enter_context(
                self._session.declare_publisher(
                    self._keyspace.state,
                    encoding=JSON_ENCODING,
                )
            )
            self._publisher_thread = Thread(
                target=self._run_publisher,
                name="node-state-publisher",
                daemon=True,
            )
            self._publisher_thread.start()
        except Exception:
            self._resources.close()
            raise
        return self

    def __exit__(self, *_exc_info: object) -> None:
        self._publisher_stop.set()
        if self._publisher_thread is not None:
            self._publisher_thread.join(timeout=PUBLISHER_JOIN_TIMEOUT_SECONDS)
            if self._publisher_thread.is_alive():
                LOGGER.error("Timed out stopping the Node State publisher")
        self._resources.close()

    def _publish(self) -> None:
        publisher = self._publisher
        assert publisher is not None
        payload = self._controller.published_state().model_dump_json()
        publisher.put(payload)

    def _run_publisher(self) -> None:
        deadline = time.monotonic()
        while True:
            if self._publisher_stop.wait(max(0.0, deadline - time.monotonic())):
                try:
                    self._publish()
                except Exception as error:
                    LOGGER.error("Failed to publish final Node State: %s", error)
                return
            try:
                self._publish()
            except Exception as error:
                LOGGER.error("Failed to publish Node State: %s", error)
            deadline = next_publish_deadline(
                deadline,
                now=time.monotonic(),
                period_seconds=self._state_publish_period_seconds,
            )

    def _reply(self, query: zenoh.Query) -> None:
        with query:
            payload = self._controller.published_state().model_dump_json()
            query.reply(
                self._keyspace.state,
                payload,
                encoding=JSON_ENCODING,
            )

    def _on_command(self, sample: zenoh.Sample) -> None:
        received_at = time.monotonic()
        try:
            command = decode_command(sample.payload.to_bytes())
        except (UnicodeDecodeError, ValidationError, ValueError) as error:
            LOGGER.warning("Rejected invalid Zenoh command: %s", error)
            return
        assert isinstance(command, (VelocityCommand, PostureCommand))
        if not self._controller.receive_command(command, received_at=received_at):
            LOGGER.debug("Ignored command while command input is disabled")


def next_publish_deadline(
    deadline: float,
    *,
    now: float,
    period_seconds: float,
) -> float:
    """Advance an absolute periodic deadline without replaying missed periods."""

    elapsed_periods = max(1, math.floor((now - deadline) / period_seconds) + 1)
    return deadline + elapsed_periods * period_seconds


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
    client.SetTimeout(config.sport_client.rpc_timeout_seconds)
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


def run(zenoh_config: zenoh.Config, node_config: NodeConfig) -> None:
    zenoh.init_log_from_env_or("error")
    stop_event = Event()
    client, subscriber = create_unitree_resources(node_config)
    controller = Controller(
        client,
        state_freshness_seconds=node_config.dds.state_freshness_seconds,
        quiescent_linear_speed_mps=(node_config.robot_state.quiescent_linear_speed_mps),
        quiescent_yaw_rate_rad_s=(node_config.robot_state.quiescent_yaw_rate_rad_s),
        velocity_command_timeout_seconds=(
            node_config.control.velocity_command_timeout_seconds
        ),
    )

    def on_state(message: SportModeStateLike) -> None:
        received_at = time.monotonic()
        try:
            observation = to_observation(
                message,
                received_at=received_at,
            )
        except (TypeError, ValueError) as error:
            controller.reject_observation(f"Invalid SportModeState: {error}")
        else:
            controller.receive_observation(observation)

    last_processed_received_at: float | None = None

    def process_next_observation(*, timeout: float) -> bool:
        nonlocal last_processed_received_at
        if not controller.wait_for_observation(
            after=last_processed_received_at,
            timeout=timeout,
        ):
            return False
        processed_received_at = controller.process_observation(
            after=last_processed_received_at,
        )
        if processed_received_at is None:
            return False
        last_processed_received_at = processed_received_at
        return True

    with (
        zenoh.open(zenoh_config) as session,
        ZenohStateBus(
            session,
            node_config.zenoh_key_prefix,
            controller,
            state_publish_frequency_hz=(node_config.node_state_publish_frequency_hz),
        ),
        stop_on_signals(stop_event),
    ):
        started = False
        subscriber_started = False
        run_error: str | None = None
        try:
            subscriber.Init(on_state, queueLen=1)
            subscriber_started = True
            startup_deadline = (
                time.monotonic() + node_config.dds.first_state_timeout_seconds
            )
            while not stop_event.is_set() and not controller.has_observation:
                remaining = startup_deadline - time.monotonic()
                if remaining <= 0:
                    raise RuntimeError("Timed out waiting for SportModeState")
                process_next_observation(
                    timeout=min(CONTROL_WAIT_MAX_SECONDS, remaining),
                )

            if stop_event.is_set():
                return
            started = controller.has_observation
            LOGGER.info(
                "Listening for commands on %s/command",
                node_config.zenoh_key_prefix,
            )
            while not stop_event.is_set():
                process_next_observation(timeout=CONTROL_WAIT_MAX_SECONDS)
                controller.enforce_observation_freshness()
        except Exception as error:
            run_error = str(error)
            raise
        finally:
            shutdown_error: str | None = None
            if started:
                controller.begin_shutdown()
                shutdown_deadline = (
                    time.monotonic() + node_config.control.shutdown_timeout_seconds
                )
                while (
                    not controller.shutdown_complete
                    and time.monotonic() < shutdown_deadline
                ):
                    process_next_observation(timeout=CONTROL_WAIT_MAX_SECONDS)
                    controller.enforce_observation_freshness()
                shutdown_error = (
                    None
                    if controller.shutdown_complete
                    else "Shutdown timed out before State confirmed down"
                )
            controller.finish_shutdown(run_error or shutdown_error)
            if subscriber_started:
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
