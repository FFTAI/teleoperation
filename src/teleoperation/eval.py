import logging
import time

import hydra
from omegaconf import DictConfig

from teleoperation.player import EvalRobot
from teleoperation.utils import (
    CONFIG_DIR,
)

logger = logging.getLogger(__name__)


@hydra.main(config_path=str(CONFIG_DIR), config_name="eval", version_base="1.2")
def main(cfg: DictConfig):
    robot = EvalRobot(cfg)  # type: ignore

    robot.init_control_joints()

    try:
        while True:
            robot.update_display()

            action = robot.step()
            if action is None:
                continue
            robot.control_joints(action[:20])
            robot.control_hands(action[-12:])
            time.sleep(1 / 20)

    except KeyboardInterrupt:
        logger.info("Exiting...")
        robot.pause_robot()
        time.sleep(1)

        robot.end()

        time.sleep(1.0)
        exit(0)


if __name__ == "__main__":
    main()
