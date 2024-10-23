import logging
import threading
import time
from collections import deque

import numpy as np
from fourier_grx_client import ControlGroup, RobotClient
from scipy.interpolate import PchipInterpolator

logger = logging.getLogger(__name__)


class CommandHistory:
    def __init__(self, target_hz: int = 200, horizon: float = 0.1):
        self.horizon = horizon
        history_length = int(horizon * target_hz * 1.5)
        self.history = deque(maxlen=history_length)
        self.new_command_event = threading.Event()

    def put(self, command: np.ndarray):
        ts = time.monotonic()
        self.history.append((ts, command))
        self._discard_old(ts)
        self.new_command_event.set()

    def _discard_old(self, current_ts: float):
        while self.history and self.history[0][0] < current_ts - self.horizon:
            self.history.popleft()

    def get(self):
        if len(self.history) < 1:
            return None, None

        if not self.new_command_event.is_set():
            return None, None

        self.new_command_event.clear()

        ts = np.array([t for t, _ in self.history])
        commands = np.array([c for _, c in self.history])

        return ts, commands


def pchip_interpolate(timestamps: np.ndarray, commands: np.ndarray, target_hz: int = 200) -> np.ndarray:
    if len(timestamps) < 2:
        raise ValueError("At least two timestamps are required for interpolation")

    elapsed_time = timestamps[-1] - timestamps[-2]
    num_steps = int(elapsed_time * target_hz)

    if num_steps < 2:
        return commands

    pchip = PchipInterpolator(timestamps, commands, axis=0)

    x_interp = np.linspace(timestamps[-2], timestamps[-1], num_steps)

    output = pchip(x_interp)

    return output


class Upsampler(threading.Thread):
    def __init__(
        self, client: RobotClient, target_hz: int = 200, dimension: int = 32, initial_command: np.ndarray | None = None
    ):
        self.client = client
        self.dimension = dimension
        self.target_hz = target_hz
        self.target_dt = 1 / target_hz
        self.command_history = CommandHistory()
        if initial_command is not None and len(initial_command) == dimension:
            self.last_command = initial_command
        else:
            self.last_command = None
        self.stop_event = threading.Event()
        self._cmd_lock = threading.Lock()
        super().__init__()

    def put(self, command: np.ndarray):
        self.command_history.put(command)

    def get(self):
        timestamps, commands = self.command_history.get()
        if timestamps is None or commands is None or len(timestamps) < 2:
            with self._cmd_lock:
                if self.last_command is None:
                    return None
                return np.array([self.last_command])

        commands = pchip_interpolate(timestamps, commands, self.target_hz)
        return commands

    def stop(self):
        self.stop_event.set()

    def _send_command(self, command: np.ndarray):
        if len(command) != self.dimension:
            logger.warning(f"Sending wrong command: {command}")
        try:
            if self.client is not None:
                self.client.move_joints(ControlGroup.ALL, command, degrees=False, gravity_compensation=True)
            return True
        except Exception as ex:
            logger.warning(ex)
            logger.warning(f"Failed cmd: {command}")
            self.client.move_joints(
                ControlGroup.ALL, self.client.joint_positions, degrees=False, gravity_compensation=False
            )
            return False

    def run(self):
        logger.info("Upsampler started")
        while True:
            if self.stop_event.is_set():
                logger.info("Upsampler stopped.")
                with self._cmd_lock:
                    if self.client is not None:
                        if self.last_command is not None:
                            self.client.move_joints(
                                ControlGroup.ALL, self.last_command, degrees=False, gravity_compensation=False
                            )
                        else:
                            self.client.move_joints(
                                ControlGroup.ALL, self.client.joint_positions, degrees=False, gravity_compensation=False
                            )
                    break
            commands = self.get()
            if commands is None:
                with self._cmd_lock:
                    if self.last_command is None:
                        time.sleep(self.target_dt)
                        continue
                    self._send_command(self.last_command)
                time.sleep(self.target_dt)
                continue
            for command in commands:
                if self._send_command(command):
                    with self._cmd_lock:
                        self.last_command = command
                time.sleep(self.target_dt)
