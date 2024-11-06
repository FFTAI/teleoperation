import logging
import time

from fourier_dhx.sdk.DexHand import DexHand

logger = logging.getLogger(__name__)


class FourierDexHand:
    def __init__(self, hand_ip: str, dimension: int = 6):
        self.hand = DexHand(hand_ip)
        self.dimension = dimension
        self._hand_positions = [0] * dimension

    def init(self):
        pass

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
