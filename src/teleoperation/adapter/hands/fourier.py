import logging
import threading
import time
from copy import copy

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


# TODO: add a config for the tactile shape and order
TACTILE_SHAPE = (6, 12 * 8)
TACTILE_ORDER = [5, 0, 1, 2, 3, 4]
# TACTILE_NAMES = [中指，无名指，小指，拇指远，拇指近，食指]
TACTILE_NAMES = ["middle", "ring", "little", "thumb_proximal", "thumb_intermediate", "index"]


class FourierDexHand:
    def __init__(self, hand_ip: str, dimension: int = 6, use_tactile=False):
        self.hand: fdh.DexHand = FDHSingleton()

        self.init()

        self.freq = 60
        self.ip = hand_ip
        self.dimension = dimension
        self.use_tactile = use_tactile

        self._hand_positions = [0] * dimension

        assert TACTILE_SHAPE[0] == len(TACTILE_ORDER), "Tactile shape and order must have the same length"
        self._tactile_readings = np.empty(TACTILE_SHAPE, dtype=np.uint8)

        self._cmd = [0] * dimension
        self._cmd_lock = threading.Lock()
        self._stop_event = threading.Event()

        self._hand_pos_lock = threading.Lock()
        self.get_pos_thread = threading.Thread(target=self._get_positions, daemon=True)
        self.get_pos_thread.start()

        self.set_pos_thread = threading.Thread(target=self._set_positions, daemon=True)
        self.set_pos_thread.start()

        if self.use_tactile:
            self._sensor_lock = threading.Lock()
            logger.info(f"Using tactile sensor for hand {self.ip}")
            self.get_tactile_thread = threading.Thread(target=self._get_tactile, daemon=True)
            self.get_tactile_thread.start()

        logger.info(
            f"Calibrating dex hand: {self.name}; type: {self.type}; ip: {self.ip}; dimension: {dimension}, tactile: {use_tactile}"
        )

    @property
    def name(self):
        self.hand.get_name(self.ip)

    @property
    def type(self):
        self.hand.get_type(self.ip)

    def _get_positions(self):
        while True and not self._stop_event.is_set():
            start = time.perf_counter()
            res = self.hand.get_pos(self.ip)
            if isinstance(res, list) and len(res) == self.dimension:
                with self._hand_pos_lock:
                    self._hand_positions = res
            else:
                logger.warning(f"Getting hand {self.ip} pos error: {res}")
            # return self._hand_positions
            end = time.perf_counter()
            time.sleep(max(1 / self.freq - (end - start), 0))

    def _set_positions(self):
        while True and not self._stop_event.is_set():
            start = time.perf_counter()
            with self._cmd_lock:
                cmd = copy(self._cmd)
                if len(cmd) != self.dimension:
                    logger.error(f"Invalid positions: {cmd}")
                    continue
            res = self.hand.set_pos(self.ip, cmd)
            if res != fdh.Ret.SUCCESS:
                logger.warning(f"Setting hand {self.ip} pos error: {res}")
            end = time.perf_counter()
            time.sleep(max(1 / (self.freq * 1.5) - (end - start), 0))

    def _get_tactile(self):
        while True and not self._stop_event.is_set():
            start = time.perf_counter()
            data = self.hand.get_ts_matrix(self.ip)

            try:
                data = np.array([data[index] for index in TACTILE_ORDER], dtype=np.uint8)

                # data = np.reshape(data, TACTILE_SHAPE)

                if data.shape != TACTILE_SHAPE:
                    logger.error(f"Invalid tactile shape: {data.shape}, expected: {TACTILE_SHAPE}")
                    return
                with self._sensor_lock:
                    self._tactile_readings[:] = data
            except Exception as e:
                logger.error(f"Error getting tactile data: {e}")

            end = time.perf_counter()
            time.sleep(max(1 / 90 - (end - start), 0))

    def init(self):
        logger.debug("Initializing dex hand")
        ret = self.hand.init()

        if ret == fdh.Ret.SUCCESS:
            logger.info("Successfully initialized DexHand")
        else:
            logger.error(f"Failed to initialize DexHand, error code {ret}")
            raise Exception("Failed to initialize DexHand")

    def get_positions(self):
        with self._hand_pos_lock:
            return self._hand_positions

    def set_positions(self, positions, wait_reply=False):
        if len(positions) != self.dimension:
            logger.error(f"Invalid positions: {positions}")
            return
        with self._cmd_lock:
            self._cmd = list(positions)

    def get_tactile(self):
        if not self.use_tactile:
            logger.warning("Tactile sensor is not enabled")
            return None
        with self._sensor_lock:
            return self._tactile_readings

    def reset(self):
        res = self.hand.set_pos([0] * self.dimension)
        if res != fdh.Ret.SUCCESS:
            logger.warning(f"Setting hand {self.ip} pos error: {res}")
        time.sleep(1)

    def stop(self):
        self._stop_event.set()
        self.get_pos_thread.join()
        self.set_pos_thread.join()

        self.reset()
