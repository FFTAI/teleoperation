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


class FourierDexHand:
    def __init__(self, hand_ip: str, dimension: int = 6, use_tactile=False):
        self.hand: fdh.DexHand = fdh.DexHand()

        self.init()

        self.freq = 60
        self.ip = hand_ip
        self.dimension = dimension
        self.use_tactile = use_tactile

        self._hand_positions = [0] * dimension
        self._cmd = [0] * dimension
        self._cmd_lock = threading.Lock()
        self._stop_event = threading.Event()

        self._hand_pos_lock = threading.Lock()
        self.get_pos_thread = threading.Thread(target=self._get_positions, daemon=True)
        self.get_pos_thread.start()

        self.set_pos_thread = threading.Thread(target=self._set_positions, daemon=True)
        self.set_pos_thread.start()

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
            return None
        tactile_index = [5, 0, 1, 2, 3, 4]
        data = self.hand.get_ts_matrix(self.ip)

        data = np.array([data[index] for index in tactile_index])

        return data

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
