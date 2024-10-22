from __future__ import annotations

from loguru import logger

from .camera_base import CameraBase

try:
    from .oak import CamOak
except ImportError:
    logger.warning("OAK-D camera not available")

try:
    from .realsense import CamRealsense
except ImportError:
    logger.warning("RealSense camera not available")

try:
    from .zed import CamZed
except ImportError:
    logger.warning("ZED camera not available")


def make_camera(camera_type: str, **kwargs) -> CameraBase:
    if camera_type == "zed":
        return CamZed(**kwargs).with_display("stereo", (720, 1280), (0, 0, 0, 0)).start()
    # elif camera_type == "realsense":
    #     return CamRealsense(**kwargs).with_display("mono", (720, 1280), (0, 0, 0, 0))
    elif camera_type == "oak":
        return CamOak(**kwargs).with_display("stereo", (720, 1280), (0, 0, 0, 0)).start()
    else:
        raise ValueError(f"Unknown camera type: {camera_type}")
