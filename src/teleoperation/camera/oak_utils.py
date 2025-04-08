import logging
import math

import depthai as dai
import numpy as np

from teleoperation.camera.utils import CameraInfo

logger = logging.getLogger(__name__)


# resize intrinsics on host doesn't seem to work well for RGB 12MP
def resizeIntrinsicsFW(intrinsics, width, height, destWidth, destHeight, keepAspect=True):
    scaleH = destHeight / height
    scaleW = destWidth / width
    if keepAspect:
        scaleW = max(scaleW, scaleH)
        scaleH = scaleW

    scaleMat = np.array([[scaleW, 0, 0], [0, scaleH, 0], [0, 0, 1]])
    scaledIntrinscs = scaleMat @ intrinsics

    if keepAspect:
        if scaleW * height > destHeight:
            scaledIntrinscs[1][2] -= (height * scaleW - destHeight) / 2.0
        elif scaleW * width > destWidth:
            scaledIntrinscs[0][2] -= (width * scaleW - destWidth) / 2.0

    return scaledIntrinscs


def getHFov(intrinsics, width):
    fx = intrinsics[0][0]
    fov = 2 * 180 / (math.pi) * math.atan(width * 0.5 / fx)
    return fov


def getVFov(intrinsics, height):
    fy = intrinsics[1][1]
    fov = 2 * 180 / (math.pi) * math.atan(height * 0.5 / fy)
    return fov


def getDFov(intrinsics, w, h):
    fx = intrinsics[0][0]
    fy = intrinsics[1][1]
    return np.degrees(2 * np.arctan(np.sqrt(w * w + h * h) / (fx + fy)))


# https://docs.luxonis.com/software/depthai/multi-device-setup/
def find_cameras(raise_when_empty=True, type_str="oak-d-w-97") -> dict[str, CameraInfo]:
    logger.info("Searching for cameras...")
    cameras = {}
    for device in dai.Device.getAllAvailableDevices():
        logger.info(f"{device.getMxId()} {device.state}")
        print(f"{device}")
        with dai.Device(device) as d:
            calib = d.readCalibration()
            cam = dai.CameraBoardSocket.CAM_A

            M, width, height = calib.getDefaultIntrinsics(cam)
            M = np.array(M)
            d = np.array(calib.getDistortionCoefficients(cam))

        cameras[device.getMxId()] = CameraInfo(
            serial_number=device.getMxId(),
            name=device.name,
            type=type_str,
            calibration={
                "camera_matrix": M.tolist(),
                "distortion_coefficients": d.tolist(),
                "resolution": (width, height),
            },
        )

    if not cameras and raise_when_empty:
        raise OSError("Not a single camera was detected. Try re-plugging.")

    logger.info(f"Found {len(cameras)} cameras.")
    for camera in cameras.values():
        logger.info(f"Camera: {camera.serial_number}")
        logger.info(f"Name: {camera.name}")
        logger.info(f"Calibration: {camera.calibration}")
    return cameras
