from __future__ import annotations

from .camera_base import CameraBase
from .realsense import CamRealsense
from .zed import CamZed


def make_camera(camera_type: str, **kwargs) -> CameraBase:
    if camera_type == "zed":
        return CamZed(**kwargs)
    elif camera_type == "realsense":
        return CamRealsense(**kwargs)
    else:
        raise ValueError(f"Unknown camera type: {camera_type}")
