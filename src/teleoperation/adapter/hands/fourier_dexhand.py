import logging
import time
from fdh12 import DexHand

logger = logging.getLogger(__name__)


class FourierDexHand_12Dof:
    def __init__(self, hand_ip: str, dimension: int = 6):
        logger.info("load FourierDexHand_12Dof_dds")
        self.hand = DexHand(hand_ip)
        self.dimension = dimension
        self._hand_positions = [0] * dimension
        self.init()

    def init(self):
        logger.info("calibration...")
        self.hand.ctrl_set_disable()
        self.hand.ctrl_calibration()
        # wait for calibration
        time.sleep(4)
        # open the FourierDexHand_12Dof
        self.hand.ctrl_set_position([0]*12)  
        time.sleep(1)

    def get_positions(self):
        # TODO 
        return self._hand_positions

    def set_positions(self, positions, wait_reply=False):
        if len(positions) != self.dimension:
            logger.error(f"Invalid positions: {positions}")
            return
        positions = positions.tolist()
        self.hand.ctrl_set_position(positions)  

    def reset(self):
        # TODO
        logger.info("Exiting Dexterous hand")
        
