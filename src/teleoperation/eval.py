import logging
import time

import hydra
from omegaconf import DictConfig

from teleoperation.player import EvalRobot, PointCloudEvalRobot
from teleoperation.utils import (
    CONFIG_DIR,
)

logger = logging.getLogger(__name__)


@hydra.main(config_path=str(CONFIG_DIR), config_name="eval_idp3", version_base="1.2")
def main(cfg: DictConfig):
    if cfg.policy.instance.type == "idp3":
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

            for name, dim in cfg.eval.actions.items():
                if name == "hand_qpos":
                    robot.control_hands(action[dim[0] : dim[1]])
                elif name == "qpos":
                    robot.control_joints(action[dim[0] : dim[1]])
                elif name == "xyzquat":
                    raise NotImplementedError("Quat control is not implemented yet.")
                elif name == "ortho6d":
                    raise NotImplementedError("Ortho6d control is not implemented yet.")
                else:
                    raise ValueError(f"Unknown action type {name}")
            time.sleep(1 / cfg.frequency)

    except KeyboardInterrupt:
        logger.info("Exiting...")
        robot.pause_robot()
        time.sleep(1)

        robot.end()


if __name__ == "__main__":
    main()
