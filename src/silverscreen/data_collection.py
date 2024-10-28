import os
from dataclasses import dataclass, field
from glob import glob

import numpy as np

from teleoperation.utils import get_timestamp_utc


def get_episode_id(session_path: str) -> int:
    """glob existing episodes and extract their IDs, and return the next episode ID"""
    episodes = glob(f"{session_path}/*.hdf5")
    if not episodes:
        return 0
    return max([int(ep.split("_")[-1].split(".")[0]) for ep in episodes]) + 1


@dataclass
class RecordingInfo:
    episode_id: int
    session_path: str
    episode_path: str
    video_path: str

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
        # os.makedirs(
        #     self.images_path,
        #     exist_ok=True,
        # )

    def increment(self):
        self.episode_id += 1
        self.episode_path = os.path.join(self.session_path, f"episode_{self.episode_id:09d}.hdf5")
        self.video_path = os.path.join(self.session_path, f"episode_{self.episode_id:09d}")


@dataclass
class TimestampMixin:
    timestamp: list[float] = field(default_factory=list)

    def stamp(self, timestamp: float | None = None):
        if timestamp is None:
            self.timestamp.append(get_timestamp_utc().timestamp())
        else:
            self.timestamp.append(timestamp)


@dataclass
class StateData:
    hand: list[np.ndarray] = field(default_factory=list)
    robot: list[np.ndarray] = field(default_factory=list)
    pose: list[np.ndarray] = field(default_factory=list)


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

    def add_state(self, hand: np.ndarray, robot: np.ndarray, pose: np.ndarray):
        self.state.hand.append(hand)
        self.state.robot.append(robot)
        self.state.pose.append(pose)

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
