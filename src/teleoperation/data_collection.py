from __future__ import annotations

import functools
import logging
import os
from dataclasses import asdict, dataclass, field
from glob import glob

import numpy as np
from filelock import FileLock
from omegaconf import DictConfig

from teleoperation.utils import format_episode_id, get_timestamp_utc

logger = logging.getLogger(__name__)


def get_episode_id(session_path: str) -> int:
    """glob existing episodes and extract their IDs, and return the next episode ID"""
    episodes = glob(f"{session_path}/*.hdf5")
    if not episodes:
        return 0
    return max([int(ep.split("_")[-1].split(".")[0]) for ep in episodes]) + 1


@functools.lru_cache(maxsize=1)
def get_camera_names(cfg):
    if cfg.camera.instance.get("key", None) is not None:
        return [cfg.camera.instance.key]
    elif cfg.camera.instance.get("keys", {}).keys():
        return list(cfg.camera.instance.get("keys", {}).keys())
    elif cfg.camera.instance.get("camera_configs", {}).keys():
        return list(cfg.camera.instance.get("camera_configs", {}).keys())
    else:
        raise ValueError("No camera keys found in config.")


@dataclass
class RecordingInfo:
    episode_id: int
    session_path: str
    episode_path: str
    video_path: str
    lock: FileLock | None = None

    @classmethod
    def from_session_path(cls, session_path: str):
        episode_id = get_episode_id(session_path)
        episode_path = os.path.join(session_path, f"episode_{episode_id:09d}.hdf5")
        video_path = os.path.join(session_path, f"episode_{episode_id:09d}")
        return cls(episode_id, session_path, episode_path, video_path)

    def __post_init__(self):
        os.makedirs(
            self.session_path,
            exist_ok=True,
        )

    def acquire(self):
        lock_path = self.video_path + ".lock"
        logger.debug(f"Acquiring lock for {lock_path}.")
        if self.lock and self.lock.is_locked:
            logger.warning(f"Lock already acquired for {self.lock.lock_file}.")
            self.lock.release()

        self.lock = FileLock(lock_path)
        self.lock.acquire(timeout=10)

    def release(self):
        if self.lock:
            if self.lock.is_locked:
                logger.debug(f"Releasing lock for {self.lock.lock_file}.")
                self.lock.release()
            self.lock = None
        else:
            logger.warning("No lock to release.")

    def increment(self):
        self.episode_id += 1
        self.episode_path = os.path.join(self.session_path, f"episode_{self.episode_id:09d}.hdf5")
        self.video_path = os.path.join(self.session_path, f"episode_{self.episode_id:09d}")

    def save_episode(self, data_dict: EpisodeDataDict, cfg: DictConfig):
        import h5py

        try:
            with h5py.File(self.episode_path, "w", rdcc_nbytes=1024**2 * 2) as f:
                state = f.create_group("state")
                action = f.create_group("action")

                f.create_dataset("timestamp", data=data_dict.timestamp)

                for name, data in asdict(data_dict.state).items():
                    state.create_dataset(name, data=np.asanyarray(data))

                for name, data in asdict(data_dict.action).items():
                    action.create_dataset(name, data=np.asanyarray(data))

                f.attrs["episode_id"] = format_episode_id(self.episode_id)
                f.attrs["task_name"] = str(cfg.recording.task_name)
                f.attrs["camera_names"] = get_camera_names(cfg)
                f.attrs["episode_length"] = data_dict.length
                f.attrs["episode_duration"] = data_dict.duration
                f.attrs["pilot"] = cfg.recording.pilot
                f.attrs["operator"] = cfg.recording.operator

        except Exception as e:
            logger.error(f"Error saving episode: {e}")
            import pickle

            pickle.dump(data_dict, open(self.episode_path + ".pkl", "wb"))
            exit(1)

    def emergency_save(self, data_dict: EpisodeDataDict, cfg: DictConfig):
        import pickle

        pickle.dump(data_dict, open(self.episode_path + ".pkl", "wb"))
        pickle.dump(cfg, open(self.episode_path + "_cfg.pkl", "wb"))


@dataclass
class TimestampMixin:
    timestamp: list[float] = field(default_factory=list)

    def stamp(self, timestamp: float | None = None):
        if timestamp is None:
            timestamp = get_timestamp_utc().timestamp()
        self.timestamp.append(timestamp)


@dataclass
class StateData:
    hand: list[np.ndarray] = field(default_factory=list)
    robot: list[np.ndarray] = field(default_factory=list)
    pose: list[np.ndarray] = field(default_factory=list)
    tactile: list[np.ndarray] = field(default_factory=list)


@dataclass
class ActionData:
    hand: list[np.ndarray] = field(default_factory=list)
    robot: list[np.ndarray] = field(default_factory=list)
    pose: list[np.ndarray] = field(default_factory=list)


@dataclass
class EpisodeMetaData:
    id: int = -1
    task_name: str = field(default_factory=str)
    camera_names: list[str] = field(default_factory=list)


@dataclass
class EpisodeDataDict(EpisodeMetaData, TimestampMixin):
    state: StateData = field(default_factory=StateData)
    action: ActionData = field(default_factory=ActionData)

    @property
    def duration(self):
        if not self.timestamp:
            return -1
        return self.timestamp[-1] - self.timestamp[0]

    @property
    def length(self):
        return len(self.timestamp)

    def add_state(self, hand: np.ndarray, robot: np.ndarray, pose: np.ndarray, tactile: np.ndarray | None = None):
        self.state.hand.append(hand)
        self.state.robot.append(robot)
        self.state.pose.append(pose)

        if tactile is not None:
            self.state.tactile.append(tactile)

    def add_action(self, hand: np.ndarray, robot: np.ndarray, pose: np.ndarray):
        self.action.hand.append(hand)
        self.action.robot.append(robot)
        self.action.pose.append(pose)

    def to_dict(self):
        return {
            "id": self.id,
            "camera_names": self.camera_names,
            "timestamp": self.timestamp,
            "state": {
                "hand": self.state.hand,
                "robot": self.state.robot,
                "pose": self.state.pose,
                "tactile": self.state.tactile,
            },
            "action": {
                "hand": self.action.hand,
                "robot": self.action.robot,
                "pose": self.action.pose,
            },
        }

    @classmethod
    def new(cls, episode_id: int, camera_names: list[str]):
        return cls(
            id=episode_id,
            camera_names=camera_names,
        )


@dataclass
class FrameData:
    timestamp: float
    episode_id: int
    frame_id: int
    state_hands: np.ndarray
    state_robot: np.ndarray
    state_pose: np.ndarray
    action_hands: np.ndarray
    action_robot: np.ndarray
    action_pose: np.ndarray


def make_data_dict():
    # TODO: make this configurable
    camera_names = ["left", "right"]
    depth_camera_names = ["left"]
    data_dict = {
        "timestamp": [],
        "obs": {"qpos": [], "hand_qpos": [], "ee_pose": [], "head_pose": []},
        "action": {
            "joints": [],
            "hands": [],
            "ee_pose": [],
        },
    }

    for cam in camera_names:
        data_dict["obs"][f"camera_{cam}"] = []

    for cam in depth_camera_names:
        data_dict["obs"][f"depth_{cam}"] = []

    return data_dict
