import numpy as np
import time
import argparse

import sys
from pathlib import Path
from test import Robotic

def parse_args():
    parser = argparse.ArgumentParser(description="Robot system control program")
    parser.add_argument('--close', action='store_true', help='Disable the joint')
    parser.add_argument('--use_imu', action='store_true', help='The imu is not used by default')
    return parser.parse_args()


if __name__ == "__main__":
    script_dir = Path(__file__).resolve().parent
    args = parse_args()

    dds_cfg_path = script_dir/"cfgs/client_config.json"
    encoders_path = script_dir/"cfgs/encoders_state.json"
    ik_config_path = script_dir / "gr1.yaml"
    FREQUENCY = 400

    robot = Robotic(
                    dds_cfg_path = dds_cfg_path,
                    encoders_path = encoders_path,
                    ik_cfg_path = ik_config_path,
                    FREQUENCY = FREQUENCY,
                    close = args.close,
                    use_imu = args.use_imu
                    )

    if args.close:
        robot.connector.disable_joints()
        robot.connector.destroy()
        print('Disable all motors')
    else:
        robot.start()
        robot.move_to_defalut()
        # record from grx-client
        recorded_traj = np.load(script_dir/"record.npy", allow_pickle=True)
        target_traj = recorded_traj

        # Move to the first position of the trajectory
        start_pos_rad = target_traj[0]
        current_pos_deg = robot.get_all_current_pos_deg()
        current_pos_deg = robot.default_pose_solver_.inverse(current_pos_deg)
        start_position_deg = np.rad2deg(start_pos_rad.copy()) 
        start_position_deg = robot.default_pose_solver_.inverse(start_position_deg)
        robot.do_interpolated_movement(current_pos_deg, start_position_deg, n_steps=150,exec=True,exec_time=1)

        # Average execution delay per step
        exec_time = 17   
        delay_time = exec_time / len(target_traj[1:])    

        for step, joint_positions_rad in enumerate(target_traj[1:]) :
            joint_positions_deg = np.rad2deg(joint_positions_rad)
            joint_positions_deg = robot.default_pose_solver_.inverse(joint_positions_deg)
            control_positions = []
            for joint_name, position in zip(robot.joints_name, joint_positions_deg):
                if joint_name in robot.enabled_joints_name:
                    control_positions.append((joint_name,position.item() ))
            # print(f"control_positions:{control_positions}")
            robot.connector.move_joints(control_positions)
            time.sleep(1 / FREQUENCY)
            # time.sleep(delay_time)

            if step %50 == 0:
                print("*"*80)
                print(f"goal:\n{np.deg2rad(joint_positions_deg[18:25])}")
                current = robot.get_all_current_pos_deg()
                print(f"current:\n{np.deg2rad(current[18:25])}")
                print("*"*80)
            
            if step % 100 == 0:
                left_link="left_end_effector_link"
                right_link="right_end_effector_link"
                head_link = "head_yaw_link"
                base_link="base_link"
                robot.ik.q_real = np.deg2rad(robot.get_all_current_pos_deg())
                left_ee_pose = robot.ik.get_transform(left_link, base_link)
                print(f"left_ee_pose:\n{left_ee_pose}")

            if step == 0:
                start_time = time.time()
            if step == len(target_traj[1:]) - 1:
                end_time = time.time()

        print(f"target_traj ---> total_time = {end_time - start_time}")
        robot.stop()
        print("Replay Done")
