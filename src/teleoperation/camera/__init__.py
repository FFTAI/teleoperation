from __future__ import annotations

import logging

logger = logging.getLogger(__name__)


try:
    from .camera_oak import CameraOak
except ImportError:
    logger.warning("OAK-D camera SDK not available")

try:
    from .camera_realsense import CameraRealsense
except ImportError:
    logger.warning("RealSense camera SDK not available")

try:
    from .camera_zed import CameraZed
except ImportError:
    logger.warning("ZED camera SDK not available")
