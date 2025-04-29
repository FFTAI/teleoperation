import logging
import threading
import time
from queue import Queue

import cv2
import numpy as np

from teleoperation.service.gr00t import RobotInferenceClient

logger = logging.getLogger(__name__)


class Gr00tPolicy:
    def __init__(self, host: str, port: int, chunk_size: int, execute_size: int, **kwargs):
        self.chunk_size = chunk_size
        self.execute_size = execute_size
        self._action_queue = Queue()
        logger.info(f"Connecting to server at {host}:{port}")
        self.policy_client = RobotInferenceClient(host=host, port=port)

        logger.info("Available modality config available:")
        self.modality_configs = self.policy_client.get_modality_config()
        logger.info(self.modality_configs)

        self.modality = {
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
        self._state_dim = max([m["end"] for m in self.modality["state"].values()])
        self._action_dim = max([m["end"] for m in self.modality["action"].values()])

        self.batch = None
        self._lock = threading.Lock()

        self._get_action_worker_thread = threading.Thread(
            target=self._get_action_worker, args=(chunk_size - execute_size,), daemon=True
        )
        self._get_action_worker_thread.start()

    def _get_action_worker(self, overlap):
        while True:
            if self._action_queue.qsize() > overlap:
                time.sleep(1 / 100)
                continue
            with self._lock:
                if self.batch is None or self.batch["observation.images.top"] is None:
                    time.sleep(1 / 100)
                    continue
                obs = self._make_observation(self.batch)
            action_dict = self.policy_client.get_action(obs)

            actions = np.concatenate(
                [
                    np.zeros((self.chunk_size, 6)),
                    action_dict["action.left_arm"],
                    action_dict["action.right_arm"],
                    action_dict["action.left_hand"],
                    action_dict["action.right_hand"],
                ],
                axis=1,
            )
            # [: self.execute_size, ...]

            logger.debug(f"Remaining queue size: {self._action_queue.qsize()}")
            # empty the queue

            actual_overlap = overlap
            while not self._action_queue.empty():
                actual_overlap -= 1
                self._action_queue.get()

            actual_overlap = max(0, actual_overlap)
            for action in actions[actual_overlap:]:
                self._action_queue.put(action)

            time.sleep(1 / 100)

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

    def _make_observation(self, batch):
        """batch = {
        "observation.state": obs,
        "observation.images.top": images_top,
        "task": [self.eval_cfg.prompt],
        """

        img = batch["observation.images.top"].copy()
        img = img[:, 240:-240, :]
        img = cv2.resize(img, (256, 256), interpolation=cv2.INTER_LINEAR)
        # if rr:
        #     rr.log("/observation/images/top", rr.Image(img))
        img = img[None, :, :, :].astype(np.uint8)

        obs = {
            "video.ego_view": img,  # (1, 256, 256, 3)
            "state.waist": batch["observation.state"][None, :3].copy(),
            "state.head": batch["observation.state"][None, 3:6].copy(),
            "state.left_arm": batch["observation.state"][None, 6:13].copy(),
            "state.right_arm": batch["observation.state"][None, 13:20].copy(),
            "state.left_hand": batch["observation.state"][None, 20:26].copy(),
            "state.right_hand": batch["observation.state"][None, 26:32].copy(),
            "annotation.human.action.task_description": [batch["task"]],
        }
        return obs

    def select_action(self, batch):
        with self._lock:
            self.batch = batch

        logger.debug(f"Queue size: {self._action_queue.qsize()}")

        return self._action_queue.get()
