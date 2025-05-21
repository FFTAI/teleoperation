import logging
import time

import fourier_hardware_py
import numpy as np

logger = logging.getLogger(__name__)


class GR2Robot:
    def __init__(
        self,
        controlled_joint_indices: list,
        default_qpos: list,
        named_links: dict,
        hardware_config_path: str,
        controlled_groups: list,
        all_groups: list,
    ):
        logger.info(f"Initializing {self.__class__.__name__}...")
        # full_path = os.path.abspath(hardware_config_path)
        # logger.info(f"Hardware config path: {full_path}")

        self.client = fourier_hardware_py.FourierHardware(hardware_config_path)

        self.controlled_joint_indices = controlled_joint_indices
        self.default_qpos = default_qpos
        self.named_links = named_links
        self.all_groups = all_groups
        self.controlled_groups = controlled_groups

        logger.info(f"Config: {self.default_qpos}")
        logger.info(f"Controlled groups: {self.controlled_groups}")

    @property
    def joint_positions(self):
        positions = []
        for group in self.all_groups:
            positions.extend(self.client.getControlGroupState(group).q[7:])
        return np.array(positions).copy()

    def connect(
        self,
    ):
        logger.info(f"Connecting to {self.__class__.__name__}...")
        time.sleep(1.0)
        a, _ = self.loss_check()
        while not a:
            logger.error("actuator connection lost, trying to reconnect...")
            a = self.loss_check()

        for group in self.controlled_groups:
            self.client.enableControlGroup(group)

        # self.client.enableRobot()
        time.sleep(1.0)
        # move to default position
        self._move_to_default(init=True)
        logger.info(f"Connected to {self.__class__.__name__}.")

    def command_joints(self, positions, gravity_compensation=False):
        # logger.info(f"type of positions: {type(positions)}")
        # logger.info(f"positions: {positions}")

        segmented_positions = []
        # segmented_positions.append(positions[:6]) # 0-5
        # segmented_positions.append(positions[6:12]) # 6-11
        segmented_positions.append(positions[12])  # 12
        segmented_positions.append(positions[13:15])  # 13-14
        segmented_positions.append(positions[15:22])  # 15-21
        segmented_positions.append(positions[22:29])  # 22-28

        a, b = self.loss_check()

        for i in range(len(self.controlled_groups)):
            if segmented_positions[i].size == 1:
                segmented_positions[i] = np.array([segmented_positions[i]])

            if b is not None and self.controlled_groups[i] == b:
                logger.error(f"Lost connection in {self.controlled_groups[i]} group")
                continue
            # logger.info(f"Moving group {self.all_groups[i]} to {segmented_positions[i].size}")
            self.client.setControlGroupPosCmd(
                self.controlled_groups[i],
                np.array(segmented_positions[i]),
                np.zeros(segmented_positions[i].shape),
                np.zeros(segmented_positions[i].shape),
            )
            # self.client.setControlGroupPosCmd(self.groups[i],positions[i],np.zeros(len(positions[i])),np.zeros(len(positions[i])))

    def init_command_joints(self, positions):
        segmented_positions = []
        # segmented_positions.append(positions[:6]) # 0-5
        # segmented_positions.append(positions[6:12]) # 6-11
        segmented_positions.append(positions[12])  # 12
        segmented_positions.append(positions[13:15])  # 13-14
        segmented_positions.append(positions[15:22])  # 15-21
        segmented_positions.append(positions[22:29])  # 22-28

        for i in range(len(self.controlled_groups)):
            if segmented_positions[i].size == 1:
                segmented_positions[i] = np.array([segmented_positions[i]])
            # self.smoothed_move(segmented_positions[i], self.client.getControlGroupState(self.controlled_groups[i]).q[7:], self.controlled_groups[i])
            self.client.setControlGroupPosCmd(
                self.controlled_groups[i],
                segmented_positions[i],
                np.zeros(segmented_positions[i].size),
                np.zeros(segmented_positions[i].size),
            )
            # self.client.setControlGroupPosCmd(self.groups[i],positions[i],np.zeros(len(positions[i])),np.zeros(len(positions[i])))

    # def init_command_joints(self, positions):
    #     init_state = True
    #     control_fps=60
    #     duration=1
    #     current_pos = self.joint_positions[12:]
    #     target_pos = positions[12:]

    #     init_traj = fourier_hardware_py.bridgeTrajectoryWithPF(current_pos, target_pos, duration, control_fps)

    #     len_init = len(init_traj[1])

    #     while init_state:
    #         for i in range(len_init):
    #             pos = self.joint_positions[12:]
    #             one_step = init_traj[1][i]
    #             segmented_step = []
    #             segmented_step.append(one_step[0])
    #             segmented_step.append(one_step[1:3])
    #             segmented_step.append(one_step[3:10])
    #             segmented_step.append(one_step[10:17])

    #             for j in range(len(self.controlled_groups)):

    #                 if segmented_step[j].size == 1:
    #                     segmented_step[j] = np.array([segmented_step[j]])
    #                 # logger.info(f"sending pos {segmented_step[j]}, group: {self.controlled_groups[j]}")
    #                 # logger.info(f"current pos {pos}")
    #                 # logger.info(f"target pos {target_pos}")

    #                 self.client.setControlGroupPosCmd(self.controlled_groups[j], segmented_step[j], np.zeros(segmented_step[j].shape),np.zeros(segmented_step[j].shape))

    #             time.sleep(1/control_fps)

    #             # if i == len_init-1:
    #             #     for j in range(len(self.controlled_groups)):
    #             #         if segmented_step[j].size == 1:
    #             #             segmented_step[j] = np.array([segmented_step[j]])
    #             #         self.client.setControlGroupPosCmd(self.controlled_groups[j], segmented_step[j], np.zeros(segmented_step[j].shape),np.zeros(segmented_step[j].shape))

    #         init_state = False

    #     return np.array([0]*12 + list(init_traj[1][-1]))

    def stop_joints(self):
        stopped_at = self.joint_positions
        self.command_joints(stopped_at, gravity_compensation=False)
        time.sleep(0.01)
        # self.command_joints(stopped_at, gravity_compensation=False)
        # time.sleep(0.01)
        # self.command_joints(stopped_at, gravity_compensation=False)
        return stopped_at

    def observe(self):  # -> NDArray[Any]:
        positions = []
        for group in self.all_groups:
            positions.extend(self.client.getControlGroupState(group).q[7:])
        return (np.array(positions).copy(),)

    def disconnect(self):
        logger.info(f"Disconnecting from {self.__class__.__name__}...")
        self._move_to_default(init=False)

    def loss_check(self):
        a = self.client.getGroupsLoss()

        all_passed = True
        last_group = None

        for key, value in a.items():
            if value > 50:
                logger.info(f"Lost connection in {key} group")
                all_passed = False
                last_group = key
        return all_passed, last_group

    def smoothed_move(self, target, initial, group):
        traj = fourier_hardware_py.bridgeTrajectory(initial, target)

        if traj[0]:
            for i in range(len(traj[1])):
                self.client.setControlGroupPosCmd(
                    group, traj[1][i], np.zeros(len(traj[1][i])), np.zeros(len(traj[1][i]))
                )
                time.sleep(0.01)

    def _move_to_default(self, init=False):
        logger.info("Moving to the default position...")
        positions_0 = np.array([0, 0.1, 0, 0, 0, 0, 0, 0, -0.1, 0, 0, 0, 0, 0])  # initial position

        # current_pos = self.joint_positions
        # logger.info(f"Current position: {current_pos}")
        left_arm = self.client.getControlGroupState("left_manipulator").q[7:]
        right_arm = self.client.getControlGroupState("right_manipulator").q[7:]

        traj_left_0 = fourier_hardware_py.bridgeTrajectory(left_arm, self.default_qpos[3:10])
        traj_right_0 = fourier_hardware_py.bridgeTrajectory(right_arm, self.default_qpos[10:17])

        len_left_0 = len(traj_left_0[1])
        len_right_0 = len(traj_right_0[1])

        if traj_left_0[0] and traj_right_0[0]:
            for i in range(max(len_left_0, len_right_0)):
                if i < len_left_0:
                    left_pos = traj_left_0[1][i]
                    self.client.setControlGroupPosCmd(
                        "left_manipulator", left_pos, np.zeros(left_pos.shape), np.zeros(left_pos.shape)
                    )

                if i < len_right_0:
                    right_pos = traj_right_0[1][i]
                    self.client.setControlGroupPosCmd(
                        "right_manipulator", right_pos, np.zeros(right_pos.shape), np.zeros(right_pos.shape)
                    )

                time.sleep(1 / 400)
        time.sleep(0.5)

        # if init:
        #     traj_left_1 = fourier_hardware_py.bridgeTrajectory(positions_0[:7], self.default_qpos[3:10])
        #     traj_right_1 = fourier_hardware_py.bridgeTrajectory(positions_0[7:], self.default_qpos[10:17])

        #     len_left_1 = len(traj_left_1[1])
        #     len_right_1 = len(traj_right_1[1])

        #     if traj_left_1[0] and traj_right_1[0]:
        #         for i in range(max(len_left_1, len_right_1)):
        #             if i < len_left_1:
        #                 left_pos = traj_left_1[1][i]
        #                 self.client.setControlGroupPosCmd("left_manipulator", left_pos, np.zeros(left_pos.shape), np.zeros(left_pos.shape))

        #             if i < len_right_1:
        #                 right_pos = traj_right_1[1][i]
        #                 self.client.setControlGroupPosCmd("right_manipulator",right_pos, np.zeros(right_pos.shape), np.zeros(right_pos.shape))

        #             time.sleep(1/400)

        # time.sleep(0.5)

        logger.info("Moved to the default position.")
