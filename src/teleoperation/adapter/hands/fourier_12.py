import logging
import time
import numpy as np

import dexhandpy.fdexhand as fdh

logger = logging.getLogger(__name__)


class FourierDexHand12dof:
    def __init__(self, hand_ip: str, dimension: int = 12):
        self.dh = fdh.DexHand()
        self.result = fdh.Ret
        self.ret = self.dh.init()
        if self.ret == self.result.SUCCESS:
            logger.info("Successfully initialized DexHand")
        else:
            logger.error(f"Failed to initialize DexHand, error code {self.ret}")
        
        self.hand_ip = hand_ip
        self.name = self.dh.get_name(self.hand_ip)
        self.type = self.dh.get_type(self.hand_ip)
        
        self.calibrate()
        time.sleep(1)
        
        
        self.pos = [-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1]
        self.dimension = dimension
        
        

    def calibrate(self):
        # calibrate all devices
        
        if self.dh.calibration(self.hand_ip) == self.result.SUCCESS:
            # time.sleep(1)
            logger.info("Calibrated successfully")
        else:
            logger.error("Failed to calibrate")

        

    def get_positions(self):
        pos = self.dh.get_pos(self.hand_ip)

        if self.ret != self.result.SUCCESS:
            logger.warning(f"Getting hand pos error: {self.ret}")
        else:
            self._hand_positions = pos
            
        return self._hand_positions

    def set_positions(self, positions, wait_reply=False):
        if len(positions) != self.dimension:
            logger.error(f"Invalid positions: {positions}")
            return
        ret = self.dh.set_pos(self.hand_ip, positions)
        if ret != self.result.SUCCESS:
            logger.warning(f"{self.name}: failed to set pos {positions}")

    def get_tactile(self, hand_ip):
        tactile_index = [5, 0, 1, 2, 3, 4]
        data = self.dh.get_ts_matrix(hand_ip)

        finger_data = np.array([data[index] for index in tactile_index])

        return finger_data
    
    def reset(self):
        ret = self.dh.reboot(self.hand_ip)
        if ret != self.result.SUCCESS:
            logger.error(f"{self.name}: failed to reset")
        # pos = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        # self.hand.ctrl_set_position(pos)
        time.sleep(1)
        self.calibrate()
        time.sleep(1)