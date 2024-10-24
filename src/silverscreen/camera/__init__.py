from __future__ import annotations

import logging

from .camera_base import CameraBase

logger = logging.getLogger(__name__)


try:
    from .oak import CamOak
except ImportError:
    logger.warning("OAK-D camera SDK not available")

try:
    from .realsense import CamRealsense
except ImportError:
    logger.warning("RealSense camera SDK not available")

try:
    from .zed import CamZed
except ImportError:
    logger.warning("ZED camera SDK not available")
