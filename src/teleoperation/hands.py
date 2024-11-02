import logging
import time
from collections.abc import Sequence
from typing import Protocol

from fourier_dhx.sdk.DexHand import DexHand
from fourier_dhx.sdk.InspireHand import InspireHand

logger = logging.getLogger(__name__)


class Hand(Protocol):
    def init(self): ...
    def get_positions(self) -> Sequence[int]: ...

    def set_positions(self, positions: Sequence[int], wait_reply: bool = False): ...

    def reset(self): ...


class FourierDexHand:
    def __init__(self, hand_ip: str, dimension: int = 6):
        self.hand = DexHand(hand_ip)
        self.dimension = dimension
        self._hand_positions = [0] * dimension

    def init(self):
        return self._reset()

    def _reset(self):
        _back1, _forward, _stop = (
            [-200, -200, -200, -200, -200, -200],
            [200, 200, 200, 200, 200, 200],
            [0, 0, 0, 0, 0, 0],
        )
        for i in range(10):
            m_last_cnt = self.hand.get_cnt()
            if len(m_last_cnt) != 6:
                logger.warning("calibration communication failed, try again...")
                if i == 9:
                    logger.error("calibration failed")
                    return False
                continue
            logger.info("calibration start")
            break

        self.hand.set_pwm(_back1)
        time.sleep(2)
        go_back_counts = 0

        for i in range(500):
            m_cur_cnt = self.hand.get_cnt()

            if len(m_cur_cnt) != 6:
                continue

            if m_cur_cnt == m_last_cnt:
                go_back_counts += 1
                if go_back_counts > 5:
                    self.hand.set_pwm(_back1)
                    time.sleep(2)
                    self.hand.calibration()
                    time.sleep(0.1)
                    logger.info("calibration success")
                    return True
                self.hand.set_pwm(_forward)
            else:
                self.hand.set_pwm(_back1)

            m_last_cnt = m_cur_cnt
            time.sleep(0.01)

        self.hand.set_pwm(_stop)
        time.sleep(2)
        logger.error("calibration failed")
        return False

    def get_positions(self):
        res = self.hand.get_angle()
        if isinstance(res, list) and len(res) == self.dimension:
            self._hand_positions = res
        else:
            logger.warning(f"Getting hand pos error: {res}")
        return self._hand_positions

    def set_positions(self, positions, wait_reply=False):
        if len(positions) != self.dimension:
            logger.error(f"Invalid positions: {positions}")
            return
        self.hand.set_angle(0, positions)

    def reset(self):
        self.hand.set_pwm([-200] * self.dimension)
        time.sleep(2.0)
        self.hand.set_pwm([0] * self.dimension)


class InspireDexHand:
    def __init__(self, ip: str, port: int = 2333, timeout: float = 0.1):
        """Simple UDP client for Inspire Dex hand control

        Args:
            ip (str): Hand IP address, usually 192.168.137.19 and 192.168.137.39
            port (int, optional): Hand UDP port. Defaults to 2333.
            timeout (float, optional): UDP timeout. Defaults to 0.1.
        """
        self.hand = InspireHand(ip, timeout)

    def reset(self):
        self.set_positions([1000, 1000, 1000, 1000, 1000, 1000])

    def set_positions(self, positions: Sequence[int], wait_reply=False):
        self.hand.set_angle(positions)

    def get_positions(
        self,
    ):
        angles = self.hand.get_angle()
        return angles
