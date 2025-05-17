import logging
import time

import hydra
import numpy as np
from omegaconf import DictConfig

from teleoperation.player import EvalRobot, PointCloudEvalRobot
from teleoperation.utils import (
    CONFIG_DIR,
)

logger = logging.getLogger(__name__)


@hydra.main(config_path=str(CONFIG_DIR), config_name="eval_lerobot", version_base="1.2")
def main(cfg: DictConfig):
    if cfg.policy.instance.policy_type == "idp3":
        robot = PointCloudEvalRobot(cfg)
    else:
        robot = EvalRobot(cfg)

    robot.init_control_joints()

    # wait for the robot to be ready
    input("Press Enter to start the robot...")

    try:
        while True:
            robot.update_display()

            action = robot.step()
            if action is None:
                continue

            logger.debug(action)

            hand_action = np.concatenate([action.get("left_hand", np.zeros(6)), action.get("right_hand", np.zeros(6))])
            arm_action = np.concatenate(
                [
                    action.get("left_leg", np.zeros(6)),
                    action.get("right_leg", np.zeros(6)),
                    action.get("waist", np.zeros(3)),
                    action.get("neck", np.zeros(3)),
                    action.get("left_arm", np.zeros(7)),
                    action.get("right_arm", np.zeros(7)),
                ],
            )

            # TODO: add pose state and action
            robot.control_hands(hand_action)
            robot.control_joints(arm_action)
            time.sleep(1 / cfg.frequency)

    except KeyboardInterrupt:
        logger.info("Exiting...")
        robot.pause_robot()
        time.sleep(1)

        robot.end()


if __name__ == "__main__":
    main()
