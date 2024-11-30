import logging
import tempfile
import time
from pathlib import Path

from fourier_grx_dds.gravity_compensation import GravityCompensator
from fourier_grx_dds.utils import GR1ControlGroup
from omegaconf import DictConfig, OmegaConf

logger = logging.getLogger(__name__)


def init_encoders(self):
    import os

    """Initialize the encoders state."""
    encoders_state, integrality = self.connector.get_encoders_state()
    assert integrality, f"Error: Can not fetch the whole encoders_state."
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


class GR1Robot_DDS:
    def __init__(
        self,
        dds_cfg: DictConfig,
        controlled_joint_indices: list,
        default_qpos: list,
        named_links: dict,
        calibrate_encoders: bool = False,
    ):
        self.controlled_joint_indices = controlled_joint_indices
        self.default_qpos = default_qpos
        self.named_links = named_links

        # write dds_cfg to tempfile
        config_str = OmegaConf.to_yaml(dds_cfg, resolve=True)
        with tempfile.NamedTemporaryFile(delete=True, suffix=".yaml", mode="w") as temp_file:
            temp_file.write(config_str)

            self.client = GravityCompensator(Path(temp_file.name), target_hz=120)

            logger.info(f"Initializing {self.__class__.__name__}...")
            logger.info(f"cfg_path: {temp_file.name}")
            logger.info(f"Config: {self.default_qpos}")

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
        self.client.move_joints(GR1ControlGroup.UPPER_EXTENDED, self.default_qpos, duration=2.0)
        logger.info(f"Connected to {self.__class__.__name__}.")

    def command_joints(self, positions, gravity_compensation=False):
        self.client.move_joints(GR1ControlGroup.ALL, positions, duration=0.0, gravity_compensation=gravity_compensation)

    # For safety use of interpolation move to the initial position
    def init_command_joints(self, positions):
        self.client.move_joints(GR1ControlGroup.ALL, positions, duration=1.0)

    def stop_joints(self):
        self.command_joints(self.joint_positions, gravity_compensation=False)

    def observe(self):
        return (self.client.joint_positions.copy(),)

    def disconnect(self):
        logger.info(f"Disconnecting from {self.__class__.__name__}...")
        self.client.set_enable(False)
