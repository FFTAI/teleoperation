import logging

import cv2
from omegaconf import DictConfig

logger = logging.getLogger(__name__)


try:
    from lerobot.common.datasets.lerobot_dataset import LeRobotDataset, LeRobotDatasetMetadata
    from lerobot.common.datasets.utils import dataset_to_policy_features
    from lerobot.common.policies.factory import get_policy_class, make_policy_config
    from lerobot.configs.types import FeatureType

    LEROBOT_AVAILABLE = True
except ImportError:
    logger.warning("LeRobot not installed.")
    LEROBOT_AVAILABLE = False

try:
    import torch
except ImportError:
    logger.warning("Torch not installed.")
    torch = None


class LerobotPolicy:
    def __init__(self, repo_id: str, type: str, pretrained_path: str, policy_config: DictConfig):
        if not LEROBOT_AVAILABLE or torch is None:
            raise ImportError("LeRobot not installed.")
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        logger.info(f"Device: {self.device}")
        logger.info(f"Loading policy {type} from {pretrained_path}")

        ds_meta = LeRobotDatasetMetadata(repo_id)

        cfg = make_policy_config(type, **policy_config)

        kwargs = {}
        features = dataset_to_policy_features(ds_meta.features)
        kwargs["dataset_stats"] = ds_meta.stats

        cfg.output_features = {key: ft for key, ft in features.items() if ft.type is FeatureType.ACTION}
        cfg.input_features = {key: ft for key, ft in features.items() if key not in cfg.output_features}
        kwargs["config"] = cfg

        self.policy = get_policy_class(type).from_pretrained(pretrained_path, **kwargs)

        # self.policy = torch.compile(self.policy, mode="reduce-overhead")
        self.policy.eval()
        self.policy.to(self.device)

        logger.info(f"Policy {type} loaded from {pretrained_path}.")

    def select_action(self, batch):
        batch["observation.images.top"] = cv2.resize(
            batch["observation.images.top"], (256, 256), interpolation=cv2.INTER_LINEAR
        ).transpose(2, 0, 1)
        batch["observation.images.top"] = torch.from_numpy(batch["observation.images.top"]).unsqueeze(0).to(self.device)
        batch["observation.state"] = torch.from_numpy(batch["observation.state"]).unsqueeze(0).to(self.device)
        action = self.policy.select_action(batch=batch)
        action = action.cpu().numpy()
        action = action.squeeze(0)
        return action
