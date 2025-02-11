import logging
import subprocess
import time
from collections import OrderedDict, defaultdict
from copy import deepcopy
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
from numba import jit
from scipy.spatial.transform import Rotation as R

logger = logging.getLogger(__name__)
try:
    from pynput import keyboard
except ImportError:
    logger.warning("pynput import failed. KeyboardListener will not work.")


PROJECT_ROOT = Path(__file__).resolve().parent
ASSET_DIR = PROJECT_ROOT.parent.parent / "assets"
CONFIG_DIR = PROJECT_ROOT.parent.parent / "configs"
SERVER_CONFIG_DIR = PROJECT_ROOT.parent.parent / "server_config"
DATA_DIR = PROJECT_ROOT.parent.parent / "data"
RECORD_DIR = DATA_DIR / "recordings"
LOG_DIR = DATA_DIR / "logs"
CERT_DIR = PROJECT_ROOT.parent.parent / "certs"


def format_episode_id(episode_id):
    return f"{episode_id:09d}"


def get_timestamp_utc():
    return datetime.now(timezone.utc)


def datetime_to_iso(dt: datetime) -> str:
    """
    Convert a datetime object to an ISO 8601 filename with microseconds.
    Args:
        dt (datetime): The datetime object to convert.
    Returns:
        str: ISO 8601 filename-safe string with microseconds.
    """
    return dt.replace(tzinfo=timezone.utc).strftime("%Y-%m-%dT%H-%M-%S_%f")


def posix_to_iso(posix: float) -> str:
    """
    Convert a POSIX timestamp to an ISO 8601 filename with microseconds.
    Args:
        posix (float): The POSIX timestamp to convert.
    Returns:
        str: ISO 8601 filename-safe string with microseconds.
    """
    dt = datetime.fromtimestamp(posix, tz=timezone.utc)
    return datetime_to_iso(dt)


def iso_to_datetime(filename: str) -> datetime:
    """
    Convert an ISO 8601 filename with microseconds back to a datetime object.
    Args:
        filename (str): The filename to parse, including or excluding the file extension.
    Returns:
        datetime: Parsed datetime object.
    """
    if "." in filename:
        base_name = filename.split(".")[0]  # Remove file extension
    else:
        base_name = filename
    return datetime.strptime(base_name, "%Y-%m-%dT%H-%M-%S_%f").replace(tzinfo=timezone.utc)


def se3_to_xyzortho6d(se3):
    """
    Convert SE(3) to continuous 6D rotation representation.
    """
    so3 = se3[:3, :3]
    xyz = se3[:3, 3]
    ortho6d = so3_to_ortho6d(so3)
    return np.concatenate([xyz, ortho6d])


def xyzortho6d_to_se3(xyzortho6d):
    """
    Convert continuous 6D rotation representation to SE(3).
    """
    xyz = xyzortho6d[:3]
    ortho6d = xyzortho6d[3:]
    so3 = ortho6d_to_so3(ortho6d)
    se3 = np.eye(4)
    se3[:3, :3] = so3
    se3[:3, 3] = xyz
    return se3


def so3_to_ortho6d(so3):
    """
    Convert to continuous 6D rotation representation adapted from
    On the Continuity of Rotation Representations in Neural Networks
    https://arxiv.org/pdf/1812.07035.pdf
    https://github.com/papagina/RotationContinuity/blob/master/sanity_test/code/tools.py
    """
    return so3[:, :2].transpose().reshape(-1)


@jit
def ortho6d_to_so3(ortho6d):
    """
    Convert from continuous 6D rotation representation to SO(3), adapted from
    On the Continuity of Rotation Representations in Neural Networks
    https://arxiv.org/pdf/1812.07035.pdf
    https://github.com/papagina/RotationContinuity/blob/master/sanity_test/code/tools.py
    """
    x_raw = ortho6d[:3]
    y_raw = ortho6d[3:6]

    x = x_raw / np.linalg.norm(x_raw)
    z = np.cross(x, y_raw)
    z = z / np.linalg.norm(z)
    y = np.cross(z, x)
    return np.column_stack((x, y, z))


def ortho6d_to_R(ortho6d):
    return R.from_matrix(ortho6d_to_so3(ortho6d))


def R_to_ortho6d(R):
    return so3_to_ortho6d(R.as_matrix())


def se3_to_xyzquat(se3):
    se3 = np.asanyarray(se3).astype(float)
    if se3.shape != (4, 4):
        raise ValueError("Input must be a 4x4 matrix")
    return _se3_to_xyzquat(se3)


def xyzquat_to_se3(xyzquat):
    xyzquat = np.asanyarray(xyzquat).astype(float)
    if xyzquat.shape != (7,):
        raise ValueError("Input must be a 7-element array")
    return _xyzquat_to_se3(xyzquat)


def _se3_to_xyzquat(se3):
    translation = se3[:3, 3]
    rotmat = se3[:3, :3]

    quat = R.from_matrix(rotmat).as_quat()

    xyzquat = np.concatenate([translation, quat])
    return xyzquat


def _xyzquat_to_se3(xyzquat):
    translation = xyzquat[:3]
    quat = xyzquat[3:]

    rotmat = R.from_quat(quat).as_matrix()

    se3 = np.eye(4)
    se3[:3, :3] = rotmat
    se3[:3, 3] = translation

    return se3


def mat_update(prev_mat, mat):
    if np.linalg.det(mat) == 0:
        return prev_mat
    else:
        return mat


def fast_mat_inv(mat):
    mat = np.asarray(mat)
    ret = np.eye(4)
    ret[:3, :3] = mat[:3, :3].T
    ret[:3, 3] = -mat[:3, :3].T @ mat[:3, 3]
    return ret


def remap(x, old_min, old_max, new_min, new_max, clip=True):
    old_min = np.array(old_min)
    old_max = np.array(old_max)
    new_min = np.array(new_min)
    new_max = np.array(new_max)
    x = np.array(x)
    tmp = (x - old_min) / (old_max - old_min)
    if clip:
        tmp = np.clip(tmp, 0, 1)
    return new_min + tmp * (new_max - new_min)


class KeyboardListener:
    def __init__(self):
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self._key_pressed = defaultdict(bool)
        logger.debug("Keyboard listener initialized")

    @property
    def key_pressed(self):
        out_key_pressed = deepcopy(self._key_pressed)
        self._key_pressed = defaultdict(bool)
        return out_key_pressed

    @property
    def space_pressed(self):
        return self._key_pressed.get("space", False)

    def start(self):
        self.listener.start()

    def on_press(self, key):
        try:
            if isinstance(key, keyboard.KeyCode):
                self._key_pressed[key.char] = True
            elif isinstance(key, keyboard.Key):
                self._key_pressed[key.name] = True

        except AttributeError:
            pass

    def on_release(self, key):
        try:
            if key == keyboard.Key.space:
                self._space_pressed = False
        except AttributeError:
            pass

    def stop(self):
        self.listener.stop()


def encode_video_frames(
    imgs_dir: Path,
    video_path: Path,
    fps: int,
    vcodec: str = "libsvtav1",
    pix_fmt: str = "yuv420p",
    g: int | None = 2,
    crf: int | None = 30,
    fast_decode: int = 0,
    log_level: str | None = "error",
    overwrite: bool = False,
    start_frame: int = 0,
    end_frame: int = -1,
) -> None:
    """More info on ffmpeg arguments tuning on `benchmark/video/README.md`"""
    video_path = Path(video_path)
    video_path.parent.mkdir(parents=True, exist_ok=True)

    if start_frame > 0 or end_frame != -1:
        # ffmpeg_inputs = []
        # images = imgs_dir.glob("rgb_frame_*.png")
        # images = sorted(images, key=lambda x: int(x.stem.split("_")[-1]))
        # for img in images:
        #     id = int(img.stem.split("_")[-1])  # noqa: A001
        #     if id > start_frame and (end_frame == -1 or id < end_frame):
        #         ffmpeg_inputs.append(("-i", str(img)))

        # # os.remove("ffmpeg_input.txt")

        # file_path = video_path.parent / "ffmpeg_input.txt"
        # file_path.unlink(missing_ok=True)

        # print(file_path.absolute())
        # with open(str(file_path.resolve()), "wb") as outfile:
        #     for img in images:
        #         id = int(img.stem.split("_")[-1])  # noqa: A001
        #         if id > start_frame and (end_frame == -1 or id < end_frame):
        #             ffmpeg_inputs.append(("-i", str(img)))
        #             outfile.write(f"file '{str(img)}'\n".encode())

        # ffmpeg_args = OrderedDict(
        #     [
        #         ("-f", "image2"),
        #         ("-r", str(fps)),
        #         ("-i", str(file_path.resolve())),
        #         # ("-vcodec", vcodec),
        #         ("-pix_fmt", pix_fmt),
        #     ]
        # )
        ffmpeg_args = OrderedDict(
            [
                ("-f", "image2"),
                ("-r", str(fps)),
                ("-pattern_type", "sequence"),
                ("-start_number", f"{start_frame}"),
                # ("-end_number", f"{end_frame}"),
                ("-i", str(Path(imgs_dir) / "rgb_frame_%09d.png")),
                # ("-vcodec", vcodec),
                ("-pix_fmt", pix_fmt),
            ]
        )
    else:
        ffmpeg_args = OrderedDict(
            [
                ("-f", "image2"),
                ("-r", str(fps)),
                ("-i", str(Path(imgs_dir) / "rgb_frame_%09d.png")),
                # ("-vcodec", vcodec),
                ("-pix_fmt", pix_fmt),
            ]
        )

    if g is not None:
        ffmpeg_args["-g"] = str(g)

    if crf is not None:
        ffmpeg_args["-crf"] = str(crf)

    if fast_decode:
        key = "-svtav1-params" if vcodec == "libsvtav1" else "-tune"
        value = f"fast-decode={fast_decode}" if vcodec == "libsvtav1" else "fastdecode"
        ffmpeg_args[key] = value

    if log_level is not None:
        ffmpeg_args["-loglevel"] = str(log_level)

    ffmpeg_args = [item for pair in ffmpeg_args.items() for item in pair]
    if overwrite:
        ffmpeg_args.append("-y")

    ffmpeg_cmd = ["ffmpeg"] + ffmpeg_args + [str(video_path)]
    # redirect stdin to subprocess.DEVNULL to prevent reading random keyboard inputs from terminal
    subprocess.run(ffmpeg_cmd, check=True, stdin=subprocess.DEVNULL)

    if not video_path.exists():
        raise OSError(
            f"Video encoding did not work. File not found: {video_path}. "
            f"Try running the command manually to debug: `{''.join(ffmpeg_cmd)}`"
        )


def match_timestamps(candidate, ref):
    closest_indices = []
    # candidate = np.sort(candidate)
    already_matched = set()
    for t in ref:
        idx = np.searchsorted(candidate, t, side="left")
        if idx > 0 and (idx == len(candidate) or np.fabs(t - candidate[idx - 1]) < np.fabs(t - candidate[idx])):
            idx = idx - 1
        if idx not in already_matched:
            closest_indices.append(idx)
            already_matched.add(idx)
        else:
            # print(f"Duplicate timestamp found: {t} and {candidate[idx]} trying to use next closest timestamp")
            if idx + 1 not in already_matched:
                closest_indices.append(idx + 1)
                already_matched.add(idx + 1)

    # print("closest_indices: ", len(closest_indices))
    return np.array(closest_indices)


if __name__ == "__main__":
    k = KeyboardListener()
    k.start()
    while True:
        print(k.key_pressed)
        time.sleep(0.1)
