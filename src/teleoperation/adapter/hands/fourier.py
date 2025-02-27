import logging
import time

import dexhandpy.fdexhand as fdh
import numpy as np

logger = logging.getLogger(__name__)


class FDHSingleton:
    _instance = None

    def __new__(cls, *args, **kwargs):
        if not cls._instance:
            cls._instance = fdh.DexHand()
            ret = cls._instance.init()
            if ret == fdh.Ret.SUCCESS:
                logger.info("Successfully initialized DexHand")
            else:
                logger.error(f"Failed to initialize DexHand, error code {ret}")
                raise Exception("Failed to initialize DexHand")
        return cls._instance


class FourierDexHand:
    def __init__(self, hand_ip: str, dimension: int = 6, use_tactile=False):
        self.dh: fdh.DexHand = FDHSingleton()

        self.hand_ip = hand_ip
        self.dimension = dimension
        self.use_tactile = use_tactile

        self._hand_positions = [0.0] * dimension

        logger.info(
            f"Calibrating dex hand: {self.name}; type: {self.type}; ip: {self.hand_ip}; dimension: {dimension}, tactile: {use_tactile}"
        )

        self.calibrate()
        time.sleep(0.1)

    @property
    def name(self):
        self.dh.get_name(self.hand_ip)

    @property
    def type(self):
        self.dh.get_type(self.hand_ip)

    def calibrate(self):
        # calibrate all devices

        if self.dh.calibration(self.hand_ip) == fdh.Ret.SUCCESS:
            # time.sleep(1)
            logger.info("Calibrated successfully")
        else:
            logger.error("Failed to calibrate")
            raise Exception(f"Failed to calibrate: {self.hand_ip}")

    def get_positions(self):
        res = self.dh.get_pos(self.hand_ip)

        if isinstance(res, list) and len(res) == self.dimension:
            self._hand_positions = res
        else:
            logger.warning(f"Getting hand {self.hand_ip} pos error: {res}")

        return self._hand_positions

    def set_positions(self, positions, wait_reply=False):
        if len(positions) != self.dimension:
            logger.error(f"Invalid positions: {positions}")
            return
        ret = self.dh.set_pos(self.hand_ip, positions)
        if ret != fdh.Ret.SUCCESS:
            logger.warning(f"{self.name}: failed to set pos {positions}")

    def get_tactile(self):
        if not self.use_tactile:
            return None
        tactile_index = [5, 0, 1, 2, 3, 4]
        data = self.dh.get_ts_matrix(self.hand_ip)

        finger_data = np.array([data[index] for index in tactile_index])

        return finger_data

    def reset(self):
        ret = self.dh.reboot(self.hand_ip)
        if ret != fdh.Ret.SUCCESS:
            logger.error(f"{self.name}: failed to reset")
        # pos = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        # self.hand.ctrl_set_position(pos)
        time.sleep(1)
        self.calibrate()
        time.sleep(1)
