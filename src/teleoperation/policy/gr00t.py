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
