from pathlib import Path
script_dir = Path(__file__).resolve().parent     # teleoperation_dds/src/arm_core_tele
run_dir = Path(__file__).resolve().parent.parent # teleoperation_dds/src

import sys
sys.path.append(script_dir)
sys.path.append(str(script_dir/'pydds'))
from parallel_joints_solver import PoseSolver

from state_machine import DDSPipeline
import json
import os
import time
import numpy as np
import argparse
from omegaconf import DictConfig, OmegaConf
from ik_test import IK_Robot


# 单位: 弧度
default_home_position = [0.0] * 32

class Robotic:
    def __init__(self,dds_cfg_path,encoders_path, ik_cfg_path, FREQUENCY=100, close: bool = False, use_imu: bool = False):
        self.close = close
        self.FREQUENCY = FREQUENCY
        self.config = self.load_config(cfg_path = dds_cfg_path)
        self.encoders_state_file_path = encoders_path
        joints = self.config["joints"]
        self.joints_name = list(joints.keys())
        self.enabled_joints_name = set([joint_name for joint_name in joints if joints[joint_name]["enable"]])
        self.connector = DDSPipeline(joints, self.config["encoders"], self.config["imu"], self.config["freq"], use_imu)
        self.default_home_position_rad = default_home_position
        self.group_ranges = {
            "left_leg": (0, 6),
            "right_leg": (6, 12),
            "waist": (12, 15),
            "head": (15, 18),
            "left_arm": (18, 25),
            "right_arm": (25, 32),
            "dual_arm": (18,32),
        }
        self.ik = IK_Robot(config=OmegaConf.load(ik_cfg_path))

    def load_config(self,cfg_path):
        with open(cfg_path, "r") as f:
            config = json.load(f)
        return config
    
    def init_encoders(self):
        encoders_state, integrality = self.connector.get_encoders_state()
        assert integrality, f"Error: Can not fetch the whole encoders_state."
        # encoders_state_file = self.config["encoders_state_file"]
        encoders_state_file = self.encoders_state_file_path
        assert os.path.exists(encoders_state_file), f"Not found encoders state file[{encoders_state_file}]. Calibration data must be exist."
        print(f"Load encoders state from {encoders_state_file}")
        self.encoders_state = json.load(open(encoders_state_file, "r"))
        if integrality:
            print(f"Save encoders poweron state to file {encoders_state_file}.")
            for name in encoders_state:
                angle = encoders_state[name].angle
                self.encoders_state[name]["poweron_pose"] = angle
            with open(encoders_state_file, "w") as f:
                json.dump(self.encoders_state, f, indent=4)

    def start(self):
        input('Init Robotic ... (press enter):')
        self.init_encoders()
        self.connector.set_joints_pid_param()
        self.connector.enable_joints()
        pvc_states, integrality = self.connector.get_pvc_states()
        assert integrality, f"Get all motor PVC states failed."
        self.default_pose_solver_  = PoseSolver(
                                            self.joints_name, 
                                            self.encoders_state, 
                                            self.config["encoders"], 
                                            self.config["joints"],
                                            pvc_states)
    
    def stop(self):
        input('Disable joints(press enter):')
        self.connector.disable_joints()
        self.connector.destroy()

    def move_to_defalut(self, n_steps = 100, exec = True, exec_time = 2,default_home_position = default_home_position):
        print(' ################### move_to_defalut ###################')
        current_pos_deg = self.get_all_current_pos_deg()
        current_pos_deg = self.default_pose_solver_.inverse(current_pos_deg)
        default_position_deg = np.rad2deg(default_home_position.copy()) 
        default_position_deg = self.default_pose_solver_.inverse(default_position_deg)
        self.do_interpolated_movement(current_pos_deg, default_position_deg, n_steps, exec=exec, exec_time=exec_time)

    # Execute the interpolation action from current_pos_deg to goal_pos_deg
    def do_interpolated_movement(self, current_pos_deg, goal_pos_deg, n_steps=100,exec=False,exec_time = 2):
        assert exec_time > 0, "exec_time error!"
        for t in np.linspace(0, 1, n_steps):
            joint_target_position_deg = current_pos_deg + t * (goal_pos_deg - current_pos_deg)
            control_positions = [(joint_name, position.item()) for joint_name, position in zip(self.joints_name, joint_target_position_deg) if joint_name in self.enabled_joints_name]
            if exec :
                start_time = time.time()
                self.connector.move_joints(control_positions)
                end_time = time.time()
                total_time = end_time - start_time
                if total_time > 0.01:
                    print(f"## out of time , use FREQUENCY :{self.FREQUENCY}")
                    time.sleep(1/self.FREQUENCY)
                else:
                    delay_time = exec_time/n_steps
                    time.sleep(delay_time)

    # Input parameters: Control group name, corresponding radian motor data (7/7/14)
    def move_joint_by_group(self,group_name,arm_joint_position_rad,n_steps=100,exec=True,exec_time=2):
        if group_name not in {'left_arm', 'right_arm','dual_arm'}:
            raise ValueError(f"Invalid group: '{group_name}'. Expected one of {'left_arm', 'right_arm','dual_arm'}.")
        if group_name != 'dual_arm' and len(arm_joint_position_rad) != 7:
            raise ValueError(f"Invalid joint length for '{group_name}'. Expected length: 7 , but got: {len(arm_joint_position_rad)}.")
        if group_name == 'dual_arm' and len(arm_joint_position_rad) != 14:
            raise ValueError(f"Invalid joint length for '{group_name}'. Expected length: 14 , but got: {len(arm_joint_position_rad)}.")
        if not all(-6.28 <= pos <= 6.28 for pos in arm_joint_position_rad):
                raise ValueError(f"Joint position (rad) is out of bounds.")
        
        print(f" #### move_joint_by_group :{group_name} ####")
        # Copy the default_home_position_rad  and change the corresponding index data based on the group_name passed in
        goal_position_rad = self.default_home_position_rad.copy()
        for robot_group_name,(robot_group_start_idx,robot_group_end_idx) in self.group_ranges.items():
            if group_name == robot_group_name:
                goal_position_rad[robot_group_start_idx:robot_group_end_idx] = arm_joint_position_rad.copy()

        current_pos_deg = self.get_all_current_pos_deg()
        current_pos_deg = self.default_pose_solver_.inverse(current_pos_deg)
        goal_position_deg = np.rad2deg(goal_position_rad.copy()) 
        goal_position_deg = self.default_pose_solver_.inverse(goal_position_deg)
        self.do_interpolated_movement(current_pos_deg, goal_position_deg, n_steps=n_steps, exec=exec,exec_time=exec_time)

    # Gets the current pvc state of the motor, returns the Angle value of the current 32 motors
    def get_all_current_pos_deg(self,):
        pvc_states, pvc_integrality = self.connector.get_pvc_states()
        assert pvc_integrality, f"Get all motor PVC states failed."
        joints_realtime_pose_angle = {key: pvc_states[key].position for key in pvc_states}
        joints_velocity            = {key: pvc_states[key].velocity for key in pvc_states}
        joints_current             = {key: pvc_states[key].current  for key in pvc_states}
        joints_pose, joints_velocity, joints_kinetic = self.default_pose_solver_.solve(joints_realtime_pose_angle, joints_velocity, joints_current)
        all_current_pos_deg = joints_pose
        return all_current_pos_deg
    
    # Returns the joint value（deg） of the current group motor based on group_name
    def get_group_current_pos_deg(self,group_name):
        all_current_pos_deg = self.get_all_current_pos_deg()
        for robot_group_name,(robot_group_start_idx,robot_group_end_idx) in self.group_ranges.items():
            if group_name == robot_group_name:
                group_current_pos_deg = all_current_pos_deg[robot_group_start_idx:robot_group_end_idx]
        # print(f"curre {group_name} joint value（deg）:\n{group_current_pos_deg}")
        return group_current_pos_deg

    def get_ee_pose(self, left_link="left_end_effector_link", right_link="right_end_effector_link", base_link="base_link"):
        left_ee_pose = self.ik.get_transform(left_link, base_link)
        right_ee_pose = self.ik.get_transform(right_link, base_link)
        left_ee_pose = self.ik.se3_to_xyzortho6d(left_ee_pose)
        right_ee_pose = self.ik.se3_to_xyzortho6d(right_ee_pose)
        return np.hstack([left_ee_pose, right_ee_pose])

    def get_head_pose(self, head_link="head_yaw_link", base_link="base_link"):
        head_pose = self.ik.get_transform(head_link, base_link)
        head_pose = self.ik.se3_to_xyzortho6d(head_pose)
        return head_pose

def parse_args():
    parser = argparse.ArgumentParser(description="Robot system control program")
    parser.add_argument('--close', action='store_true', help='Disable the joint')
    parser.add_argument('--test_get_state', type=bool, default=False)
    parser.add_argument('--test_move', type=bool, default=True)
    parser.add_argument('--test_fk', type=bool, default=False)
    parser.add_argument('--group_name', type=str, default="left_arm")
    parser.add_argument('--use_imu', action='store_true', help='The imu is not used by default')
    return parser.parse_args()

def main():
    args = parse_args()
    dds_cfg_path = script_dir/"cfgs/client_config.json"
    encoders_path = script_dir/"cfgs/encoders_state.json"
    ik_config_path = script_dir / "gr1.yaml"
    robot = Robotic(dds_cfg_path,encoders_path,ik_config_path,close=args.close, use_imu=args.use_imu)
    
    if args.close:
        robot.connector.disable_joints()
        robot.connector.destroy()
        print('Disable all motors')
    else:
        robot.start()
        
        if args.test_get_state:
            current_all_pos_deg = robot.get_all_current_pos_deg()
            group_current_pos_deg = robot.get_group_current_pos_deg(args.group_name)

        if args.test_move:
            robot.move_to_defalut()
            time.sleep(1)

            left_arm_position = [
                                    -0.10834163755741072, -0.07329939774949822, 0.06528929994794762, 
                                    -1.4866168673727456, -0.15687147335078633, -0.13071683482883256, 
                                    -0.17893611111972085  
                                ] # 左臂 (7)
            robot.move_joint_by_group("left_arm",left_arm_position)
            time.sleep(1)
            # right_arm_position =[ 
            #                         -0.5616275175421398, -0.16916011421349578, -0.009344942799520178, 
            #                         -1.0883991832631592, -0.14139464583384534, -0.22926271038737234,
            #                         0.37946845043488064
            #                     ] 
            # robot.move_joint_by_group("right_arm",right_arm_position,exec_time=1)
            # time.sleep(1)

            # dual_arm_position = [left_arm_position,right_arm_position]
            # dual_arm_position =  [position for sublist in dual_arm_position for position in sublist]
            # robot.move_joint_by_group("dual_arm",dual_arm_position)
            # time.sleep(1)

        if args.test_fk:
            left_link="left_end_effector_link"
            right_link="right_end_effector_link"
            head_link = "head_yaw_link"
            base_link="base_link"

            current_all_pos_deg = robot.get_all_current_pos_deg()
            robot.ik.q_real = np.deg2rad(current_all_pos_deg)
            left_ee_pose = robot.ik.get_transform(left_link, base_link)
            print(f"left_ee_pose:\n{left_ee_pose}")

        robot.stop()
        print("Done")
    
if __name__ == "__main__":
    main()
