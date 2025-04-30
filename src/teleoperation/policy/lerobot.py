import json
import logging
from pathlib import Path

import cv2
import numpy as np
from omegaconf import DictConfig

logger = logging.getLogger(__name__)


try:
    from lerobot.common.datasets.compute_stats import aggregate_stats
    from lerobot.common.datasets.lerobot_dataset import LeRobotDataset, LeRobotDatasetMetadata
    from lerobot.common.datasets.utils import dataset_to_policy_features, load_episodes_stats, load_stats
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


DEFAULT_MODALITY = {
    "state": {
        "waist": {"start": 0, "end": 3},
        "neck": {"start": 3, "end": 6},
        "left_arm": {"start": 6, "end": 13},
        "right_arm": {"start": 13, "end": 20},
        "left_hand": {"start": 20, "end": 26},
        "right_hand": {"start": 26, "end": 32},
    },
    "action": {
        "waist": {"start": 0, "end": 3},
        "neck": {"start": 3, "end": 6},
        "left_arm": {"start": 6, "end": 13},
        "right_arm": {"start": 13, "end": 20},
        "left_hand": {"start": 20, "end": 26},
        "right_hand": {"start": 26, "end": 32},
    },
    "video": {"top": {"original_key": "observation.images.top"}},
    "annotation": {"human.action.task_description": {}},
}


def load_modality(root_path: str | Path):
    modality_path = Path(root_path) / "meta/modality.json"
    modality = None
    if not modality_path.exists():
        # raise FileNotFoundError(f"Modality file not found at {modality_path}")
        logger.warning(f"Modality file not found at {modality_path}, using default modality.")
        modality = DEFAULT_MODALITY
    else:
        with open(modality_path) as f:
            modality = json.load(f)

    return modality


class LerobotPolicy:
    def __init__(
        self,
        repo_id: str,
        policy_type: str,
        pretrained_path: str,
        policy_config: DictConfig | dict | None = None,
    ):
        if policy_config is None:
            policy_config = {}
        if not LEROBOT_AVAILABLE or torch is None:
            raise ImportError("LeRobot not installed.")
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        logger.info(f"Device: {self.device}")
        logger.info(f"Loading policy {policy_type} from {pretrained_path}")

        ds_path = Path(repo_id)
        dataset_stats = load_stats(
            ds_path
        )  # TODO: the naming could be confusing but since we are mainly using offline datasets, it would be more convenient to use the dataset local path

        if not dataset_stats:
            episodes_stats = load_episodes_stats(ds_path)
            dataset_stats = aggregate_stats(list(episodes_stats.values()))
        logger.debug(f"Loadeded dataset stats: {dataset_stats}")

        self.modality = load_modality(ds_path)
        logger.debug(f"Loadeded modality: {self.modality}")

        self._state_dim = max([m["end"] for m in self.modality["state"].values()])
        self._action_dim = max([m["end"] for m in self.modality["action"].values()])

        self.policy = get_policy_class(policy_type).from_pretrained(pretrained_path, dataset_stats=dataset_stats)

        # self.policy = torch.compile(self.policy, mode="reduce-overhead")
        self.policy.eval()
        self.policy.to(self.device)

        logger.info(f"Policy {type} loaded from {pretrained_path}.")

    def prepare_observation(self, observation_dict):
        """Prepare observation for the policy. Convert from modality dict to array based on loaded modality config."""

        assert set(
            observation_dict.keys()
        ).issuperset(
            set(self.modality["state"].keys())
        ), f"Observation dict keys {observation_dict.keys()} do not match modality config keys {self.modality['state'].keys()}"
        obs = np.zeros((self._state_dim,), dtype=np.float32)
        for key, value in observation_dict.items():
            if key not in self.modality["state"]:
                continue
            start = self.modality["state"][key]["start"]
            end = self.modality["state"][key]["end"]
            obs[start:end] = value

        return obs

    def _convert_action(self, action_array):
        """Convert action array to dict based on loaded modality config."""
        action_dict = {}
        for key, value in self.modality["action"].items():
            start = value["start"]
            end = value["end"]
            action_dict[key] = action_array[start:end]
        return action_dict

    def select_action(self, batch):
        for key in batch.keys():
            if not key.startswith("observation.images"):
                continue
            batch[key] = batch[key][:, 240:-240, :]
            batch[key] = cv2.resize(batch[key], (256, 256), interpolation=cv2.INTER_LINEAR).transpose(2, 0, 1)
            batch[key] = torch.from_numpy(batch[key]).unsqueeze(0).to(self.device, dtype=torch.float32)

        batch["observation.state"] = (
            torch.from_numpy(batch["observation.state"]).unsqueeze(0).to(self.device, dtype=torch.float32)
        )

        action = self.policy.select_action(batch=batch)
        action = action.cpu().numpy()
        action = action.squeeze(0)

        return self._convert_action(action)
