import logging
import time
import numpy as np


from pathlib import Path
script_dir = Path(__file__).resolve().parent     # teleoperation_dds/src/teleoperation/adapter/robots
run_dir = Path(__file__).resolve().parent.parent.parent.parent # teleoperation_dds/src/

import sys
sys.path.append(str(run_dir/'arm_core_tele'))

# import sys
# sys.path.append("/data/teleoperation_dds/src/arm_core_tele")
from arm_core_tele.test import Robotic

logger = logging.getLogger(__name__)


class GR1Robot_DDS:
    def __init__(
        self,
        dds_cfg_path: str,
        encoders_path: str,
        ik_config_path: str,
        controlled_joint_indices: list,
        default_qpos: list,
        named_links: dict,
    ):
        self.client = Robotic(dds_cfg_path, encoders_path, ik_config_path)
        self.controlled_joint_indices = controlled_joint_indices
        self.default_qpos = default_qpos
        self.named_links = named_links

        logger.info(f"Initializing {self.__class__.__name__}...")
        logger.info(f"cfg_path: {ik_config_path}")
        logger.info(f"Config: {self.default_qpos}")

    @property
    def joint_positions(self):
        return np.deg2rad(self.client.get_all_current_pos_deg().copy()) 

    def connect(
        self,
    ):
        logger.info(f"Connecting to {self.__class__.__name__}...")
        time.sleep(1.0)
        self.client.start()
        time.sleep(1.0)
        self._move_to_default()
        logger.info(f"Connected to {self.__class__.__name__}.")

    def command_joints(self, positions, gravity_compensation=False):
        qpos = np.rad2deg(positions)
        qpos = self.client.default_pose_solver_.inverse(qpos)
        control_positions = [(joint_name, position.item()) for joint_name, position in zip(self.client.joints_name, qpos)]
        self.client.connector.move_joints(control_positions)

    # For safety use of interpolation move to the initial position
    def init_command_joints(self, positions):
        current_pos_deg = self.client.get_all_current_pos_deg()
        current_pos_deg = self.client.default_pose_solver_.inverse(current_pos_deg)
        goal_pos_deg = np.rad2deg(positions)
        goal_pos_deg = self.client.default_pose_solver_.inverse(goal_pos_deg)
        print("###################################################")
        print("Prepare to move the robot to the current position, please be safe")
        input('init_control_joints(press enter):')
        print("###################################################")
        self.client.do_interpolated_movement(current_pos_deg, goal_pos_deg, n_steps=150, exec=True, exec_time=2)

    def stop_joints(self):
        self.command_joints(self.joint_positions, gravity_compensation=False)

    def observe(self):
        return (np.deg2rad(self.client.get_all_current_pos_deg().copy()),)

    def disconnect(self):
        logger.info(f"Disconnecting from {self.__class__.__name__}...")
        # self._move_to_default(exec_time=2)
        self.client.stop()

    def _move_to_default(self):
        default_qpos_dds = np.hstack([np.zeros(12),self.default_qpos])
        self.client.move_to_defalut(default_home_position=default_qpos_dds)