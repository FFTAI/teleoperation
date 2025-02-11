import logging
import socket
import struct
import threading
import time

import numpy as np
from fourier_grx_client import ControlGroup, RobotClient

logger = logging.getLogger(__name__)


class GR1Robot:
    def __init__(
        self,
        namespace: str,
        controlled_joint_indices: list,
        default_qpos: list,
        named_links: dict,
    ):
        self.client = RobotClient(namespace=namespace)
        self.controlled_joint_indices = controlled_joint_indices
        self.default_qpos = default_qpos
        self.named_links = named_links

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setblocking(False)
        self._ips = []

        ips = [
            90,
            91,
            92,
            30,
            31,
            32,
            33,
            10,
            11,
            12,
            13,
        ]
        for ip in ips:
            self._ips.append(("192.168.137." + str(ip), 2335))

        self._impendance_cmds = [struct.pack(">B", cmd) for cmd in [0x07] * len(self._ips)]

        logger.info(f"Initializing {self.__class__.__name__}...")
        logger.info(f"Namespace: {namespace}")
        logger.info(f"Config: {self.default_qpos}")

        self._gravity_compensation_on = threading.Event()
        self._stop_event = threading.Event()

        self._gravity_compensation_on.clear()

        threading.Thread(target=self._set_mode_thread, name="set mode thread", daemon=True).start()

    def _set_mode_thread(self):
        while True and not self._stop_event.is_set():
            time.sleep(1 / 10)
            if self._gravity_compensation_on.is_set():
                self._set_impedance_mode()
            else:
                self._set_position_mode()

    @property
    def joint_positions(self):
        return self.client.joint_positions.copy()

    def connect(
        self,
    ):
        logger.info(f"Connecting to {self.__class__.__name__}...")
        time.sleep(1.0)
        self.client.set_enable(True)
        time.sleep(1.0)
        # move to default position
        self._move_to_default(init=True)
        logger.info(f"Connected to {self.__class__.__name__}.")

    def command_joints(self, positions, gravity_compensation=False):
        self.client.move_joints(
            ControlGroup.ALL, positions=positions, degrees=False, gravity_compensation=gravity_compensation
        )
        if gravity_compensation:
            self._gravity_compensation_on.set()
        else:
            self._gravity_compensation_on.clear()

    def init_command_joints(self, positions):
        self.client.move_joints(
            ControlGroup.ALL,
            positions=positions,
            degrees=False,
            gravity_compensation=False,
            duration=0.5,
            blocking=True,
        )

    def _set_position_mode(self):
        # logger.debug("Setting position mode...")
        payload = struct.pack(">B", 0x04)
        for ip in self._ips:
            self._sock.sendto(payload, ip)

    def _set_impedance_mode(self):
        # logger.debug("Setting impedance mode...")
        for ip, cmd in zip(self._ips, self._impendance_cmds, strict=True):
            self._sock.sendto(cmd, ip)

    def stop_joints(self):
        stopped_at = self.joint_positions
        self.command_joints(stopped_at, gravity_compensation=False)
        self._gravity_compensation_on.clear()
        logger.debug(f"Stopped at: {stopped_at}")
        logger.debug(f"Current Modes: {self.client.get_control_modes()}")
        return stopped_at

    def observe(self):
        return (self.client.joint_positions.copy(),)

    def disconnect(self):
        logger.info(f"Disconnecting from {self.__class__.__name__}...")
        self._move_to_default(init=False)

        self._stop_event.set()

    def _move_to_default(self, init=False):
        logger.info("Moving to the default position...")

        if init:
            dist = np.linalg.norm(self.client.joint_positions[12:] - self.default_qpos)
            if dist > 0.5:
                self.client.move_joints(
                    ControlGroup.UPPER,
                    positions=[0, 0.15, np.pi / 2, 0, 0, 0, 0, 0, -0.15, -np.pi / 2, 0, 0, 0, 0],
                    degrees=False,
                    gravity_compensation=False,
                    duration=1.0,
                    blocking=True,
                )

                time.sleep(0.1)

                self.client.move_joints(
                    ControlGroup.UPPER,
                    positions=[0, 0.15, np.pi / 2, -np.pi / 2, 0, 0, 0, 0, -0.15, -np.pi / 2, -np.pi / 2, 0, 0, 0],
                    degrees=False,
                    gravity_compensation=False,
                    duration=1.0,
                    blocking=True,
                )

                time.sleep(0.1)
            self.client.move_joints(
                self.controlled_joint_indices, positions=self.default_qpos, degrees=False, duration=1.0, blocking=True
            )
        else:
            self.client.move_joints(
                self.controlled_joint_indices, positions=self.default_qpos, degrees=False, duration=1.0, blocking=True
            )

            time.sleep(0.1)

            self.client.move_joints(
                ControlGroup.UPPER,
                positions=[0, 0.15, np.pi / 2, -np.pi / 2, 0, 0, 0, 0, -0.15, -np.pi / 2, -np.pi / 2, 0, 0, 0],
                degrees=False,
                gravity_compensation=False,
                duration=1.0,
                blocking=True,
            )

            time.sleep(0.1)

            self.client.move_joints(
                ControlGroup.UPPER,
                positions=[0, 0.15, np.pi / 2, 0, 0, 0, 0, 0, -0.15, -np.pi / 2, 0, 0, 0, 0],
                degrees=False,
                gravity_compensation=False,
                duration=1.0,
                blocking=True,
            )

            time.sleep(0.1)

            self.client.move_joints(
                ControlGroup.UPPER,
                positions=[0, 0.15, 0, 0, 0, 0, 0, 0, -0.15, 0, 0, 0, 0, 0],
                degrees=False,
                gravity_compensation=False,
                duration=2.0,
                blocking=True,
            )

        logger.info("Moved to the default position.")
