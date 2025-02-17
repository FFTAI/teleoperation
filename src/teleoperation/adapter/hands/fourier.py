import logging
import threading
import time
from copy import copy

from fourier_dhx.sdk.DexHand import DexHand

logger = logging.getLogger(__name__)


class FourierDexHand:
    def __init__(self, hand_ip: str, dimension: int = 6):
        self.hand = DexHand(hand_ip)
        self.freq = 60
        self.ip = hand_ip
        self.dimension = dimension
        self._hand_positions = [0] * dimension
        self._cmd = [0] * dimension
        self._cmd_lock = threading.Lock()
        self._stop_event = threading.Event()

        self._hand_pos_lock = threading.Lock()
        self.get_pos_thread = threading.Thread(target=self._get_positions, daemon=True)
        self.get_pos_thread.start()

        self.set_pos_thread = threading.Thread(target=self._set_positions, daemon=True)
        self.set_pos_thread.start()

    def _get_positions(self):
        while True and not self._stop_event.is_set():
            start = time.perf_counter()
            res = self.hand.get_angle()
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
            res = self.hand.set_angle(0, cmd)
            if res != 0:
                logger.warning(f"Setting hand {self.ip} pos error: {res}")
            end = time.perf_counter()
            time.sleep(max(1 / (self.freq * 1.5) - (end - start), 0))

    def init(self):
        pass

    def get_positions(self):
        with self._hand_pos_lock:
            return self._hand_positions

    def set_positions(self, positions, wait_reply=False):
        if len(positions) != self.dimension:
            logger.error(f"Invalid positions: {positions}")
            return
        with self._cmd_lock:
            self._cmd = list(positions)

    def reset(self):
        self.hand.set_pwm([-200] * self.dimension)
        time.sleep(2.0)
        self.hand.set_pwm([0] * self.dimension)

    def stop(self):
        self._stop_event.set()
        self.get_pos_thread.join()
        self.set_pos_thread.join()

        self.reset()


class FourierDexHand12dof:
    def __init__(self, hand_ip: str, dimension: int = 12):
        self.hand = DexHand(hand_ip)
        self.dimension = dimension
        self._hand_positions = [0] * dimension
        self.init()

    def init(self):
        self.hand.ctrl_set_disable()
        self.hand.ctrl_calibration()
        # wait for calibration
        time.sleep(4)
        self.hand.ctrl_set_position([0] * 12)
        time.sleep(1)

    def get_positions(self):
        keep_pos = [-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1]
        res = self.hand.ctrl_set_position(keep_pos)
        if isinstance(res, list) and len(res) == self.dimension:
            self._hand_positions = res
        else:
            logger.warning(f"Getting hand pos error: {res}")
        return self._hand_positions

    def set_positions(self, positions, wait_reply=False):
        if len(positions) != self.dimension:
            logger.error(f"Invalid positions: {positions}")
            return
        self.hand.ctrl_set_position(positions)

    def reset(self):
        pos = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.hand.ctrl_set_position(pos)
        time.sleep(0.5)

    def stop(self):
        pass
