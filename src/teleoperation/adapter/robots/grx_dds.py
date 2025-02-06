import logging
import time
from pathlib import Path

from omegaconf import DictConfig, OmegaConf

from teleoperation.utils import PROJECT_ROOT

logger = logging.getLogger(__name__)

try:
    from fourier_grx_dds.gravity_compensation import GravityCompensator
except ImportError as e:
    msg = "The `fourier_grx_dds` package is not installed. As the software is still in early alpha stage, please contact the maintainers regarding ways to install it OR use the legacy `fourier_grx` package and the legacy robot, e.g.`robot=gr1t2_legacy`."
    logger.error(msg)
    raise ImportError(msg) from e


def init_encoders(self):
    import os

    """Initialize the encoders state."""
    encoders_state, integrality = self.connector.get_encoders_state()
    assert integrality, "Error: Can not fetch the whole encoders_state."
    assert os.path.exists(self.encoders_state_path), f"Encoders state file[{self.encoders_state_path}] not founded."
    logger.info(f"Load encoders state from {self.encoders_state_path}")
    self.encoders_state = OmegaConf.load(self.encoders_state_path)
    if integrality:
        for name in encoders_state:
            angle = encoders_state[name].angle
            self.encoders_state[name]["poweron_pose"] = angle
            self.encoders_state[name]["calibration_pose"] = angle
        OmegaConf.save(self.encoders_state, self.encoders_state_path)
        logger.info(f"Encoders poweron state saved to {self.encoders_state_path}")
    logger.info("Encoders initialized.")


GravityCompensator.init_encoders = init_encoders


def create_from_template(template, path, force=False, base_path=PROJECT_ROOT.parent.parent):
    """Create a file from template."""
    with open(base_path / template) as f:
        template_content = f.read()

    if (base_path / path).exists() and not force:
        logger.info(f"File already exists at {path}. Use `force=True` to overwrite.")
        return

    with open(base_path / path, "w") as f:
        f.write(template_content)
    logger.info(f"File created at {path} from {template}")


class GR1Robot:
    def __init__(
        self, dds_cfg: DictConfig, controlled_joint_indices: list, default_qpos: list, named_links: dict, target_hz: int
    ):
        self.controlled_joint_indices = controlled_joint_indices
        self.default_qpos = default_qpos
        self.named_links = named_links

        # check if `dds_config.encoders_state_path` exists, if not create it from `server_config/dds/encoders_state.template.yaml`
        if (
            dds_cfg.robot.startswith("gr1")
            and not (PROJECT_ROOT.parent.parent / Path(dds_cfg.encoders_state_path)).exists()
        ):
            create_from_template("server_config/dds/encoders_state.template.yaml", dds_cfg.encoders_state_path)

        # for GR2 robot, there needs to be a `motor_gains.yaml` file, if not create it from `server_config/dds/motor_gains.template.yaml`
        if dds_cfg.robot.startswith("gr2"):
            if not (PROJECT_ROOT.parent.parent / Path(dds_cfg.motor_gains_path)).exists():
                with open(PROJECT_ROOT.parent.parent / "server_config/dds/motor_gains.template.yaml") as f:
                    motor_gains_template = f.read()
                with open(PROJECT_ROOT.parent.parent / Path(dds_cfg.motor_gains_path), "w") as f:
                    f.write(motor_gains_template)
                create_from_template("server_config/dds/motor_gains.template.yaml", dds_cfg.motor_gains_path)
                logger.warning("Please fill in the motor gains in `server_config/dds/motor_gains.yaml`.")
            # load motor gains and merge with dds_cfg.joints
            motor_gains = OmegaConf.load(PROJECT_ROOT.parent.parent / "server_config/dds/motor_gains.yaml")
            dds_cfg.joints = OmegaConf.merge(dds_cfg.joints, motor_gains)

        self.client = GravityCompensator(dds_cfg, target_hz=target_hz)

        logger.info(f"Initializing {self.__class__.__name__}...")

    @property
    def joint_positions(self):
        return self.client.joint_positions

    def connect(
        self,
    ):
        logger.info(f"Connecting to {self.__class__.__name__}...")
        time.sleep(1.0)
        self.client.enable()
        time.sleep(1.0)
        self.client.move_joints(self.client.control_group.UPPER_EXTENDED, self.default_qpos, duration=2.0)
        logger.info(f"Connected to {self.__class__.__name__}.")

    def command_joints(self, positions, gravity_compensation=False):
        self.client.move_joints(
            self.client.control_group.ALL, positions, duration=0.0, gravity_compensation=gravity_compensation
        )

    # For safety use of interpolation move to the initial position
    def init_command_joints(self, positions):
        self.client.move_joints(self.client.control_group.ALL, positions, duration=1.0)

    def stop_joints(self):
        stopped_at = self.joint_positions
        self.command_joints(stopped_at, gravity_compensation=False)
        return stopped_at

    def observe(self):
        return (self.client.joint_positions.copy(),)

    def disconnect(self):
        logger.info(f"Disconnecting from {self.__class__.__name__}...")
        self.stop_joints()
        # self.client.set_enable(False)
