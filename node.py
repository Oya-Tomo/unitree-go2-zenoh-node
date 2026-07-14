"""Bridge Zenoh high-level commands to one Unitree Go2 SportClient."""

from __future__ import annotations

import argparse
import logging
import signal
import time
from collections.abc import Callable, Iterator, Sequence
from contextlib import ExitStack, contextmanager
from pathlib import Path
from threading import Event, Lock
from types import FrameType
from typing import Any

import zenoh
from pydantic import BaseModel, ValidationError

from command_mailbox import CommandBatch, CommandMailbox
from controller import RobotController, SportClientProtocol, StateSink
from models import HealthState, PostureState, VelocityState
from settings import NodeConfig, load_node_config

DEFAULT_NODE_CONFIG_PATH = Path("config/node-config.json5")
DEFAULT_ZENOH_CONFIG_PATH = Path("config/zenoh-config.json5")
CONTROL_LOOP_PERIOD_SECONDS = 0.01
POSTURE_COMMANDS_PER_TICK = 4
JSON_ENCODING = "application/json"
LOGGER = logging.getLogger(__name__)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="unitree-go2-zenoh-node",
        description="Forward Zenoh commands to one Unitree Go2 SportClient.",
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


class ZenohStateBus(StateSink):
    def __init__(self, session: zenoh.Session, robot_key: str) -> None:
        self._session = session
        self._keys = {
            "requested": f"{robot_key}/state/command/requested",
            "applied": f"{robot_key}/state/command/applied",
            "posture": f"{robot_key}/state/posture",
            "health": f"{robot_key}/state/health",
        }
        self._payloads: dict[str, str] = {}
        self._lock = Lock()
        self._resources = ExitStack()

    def __enter__(self) -> ZenohStateBus:
        for key in self._keys.values():
            queryable = self._session.declare_queryable(
                key,
                lambda query, reply_key=key: self._reply(query, reply_key),
                complete=True,
            )
            self._resources.enter_context(queryable)
        return self

    def __exit__(self, *exc_info: object) -> None:
        self._resources.close()

    def _reply(self, query: zenoh.Query, key: str) -> None:
        with query:
            with self._lock:
                payload = self._payloads.get(key)
            if payload is not None:
                query.reply(key, payload, encoding=JSON_ENCODING)

    def _publish(self, key_name: str, state: BaseModel) -> None:
        key = self._keys[key_name]
        payload = state.model_dump_json()
        with self._lock:
            self._payloads[key] = payload
        self._session.put(key, payload, encoding=JSON_ENCODING)

    def publish_requested(self, state: VelocityState) -> None:
        self._publish("requested", state)

    def publish_applied(self, state: VelocityState) -> None:
        self._publish("applied", state)

    def publish_posture(self, state: PostureState) -> None:
        self._publish("posture", state)

    def publish_health(self, state: HealthState) -> None:
        self._publish("health", state)


def create_sport_client(config: NodeConfig) -> SportClientProtocol:
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.go2.sport.sport_client import SportClient

    ChannelFactoryInitialize(
        config.dds.domain_id,
        config.dds.network_interface,
    )
    client = SportClient()
    client.SetTimeout(config.dds.rpc_timeout_seconds)
    client.Init()
    return client


def dispatch_commands(
    controller: RobotController,
    batch: CommandBatch,
    *,
    command_timeout_seconds: float,
    clock: Callable[[], float] = time.monotonic,
) -> None:
    for received in batch.postures:
        controller.handle_command(received.command, now=received.received_at)

    velocity = batch.velocity
    if velocity is not None:
        age = clock() - velocity.received_at
        if age <= command_timeout_seconds:
            controller.handle_command(velocity.command, now=velocity.received_at)
        else:
            controller.report_command_error(
                f"stale velocity command dropped after {age:.3f} seconds"
            )

    if batch.invalid_error is not None:
        controller.report_command_error(batch.invalid_error)
    controller.tick(now=clock())


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


def run(
    zenoh_config: zenoh.Config,
    node_config: NodeConfig,
    *,
    sport_client: SportClientProtocol | None = None,
) -> None:
    zenoh.init_log_from_env_or("error")
    command_key = f"{node_config.robot_key}/command"
    command_mailbox = CommandMailbox()
    stop_event = Event()

    def on_command(sample: zenoh.Sample) -> None:
        command_mailbox.submit(
            sample.payload.to_bytes(),
            received_at=time.monotonic(),
        )

    client = (
        sport_client if sport_client is not None else create_sport_client(node_config)
    )
    with (
        zenoh.open(zenoh_config) as session,
        ZenohStateBus(session, node_config.robot_key) as state_bus,
        session.declare_subscriber(command_key, on_command),
        stop_on_signals(stop_event),
    ):
        controller = RobotController(
            client,
            state_bus,
            command_timeout_seconds=node_config.safety.command_timeout_seconds,
            posture_transition_seconds=node_config.safety.posture_transition_seconds,
            shutdown_stop_delay_seconds=node_config.safety.shutdown_stop_delay_seconds,
        )
        LOGGER.info("Listening for commands on %s", command_key)
        controller.startup()
        try:
            while not stop_event.is_set():
                dispatch_commands(
                    controller,
                    command_mailbox.take(
                        posture_limit=POSTURE_COMMANDS_PER_TICK,
                    ),
                    command_timeout_seconds=node_config.safety.command_timeout_seconds,
                )
                stop_event.wait(CONTROL_LOOP_PERIOD_SECONDS)
        finally:
            controller.graceful_shutdown()


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
