import logging

import depthai

logger = logging.getLogger(__name__)


def find_cameras(raise_when_empty=True) -> list[dict]:
    logger.info("Searching for cameras...")
    cameras = []
    for device in depthai.Device.getAllAvailableDevices():
        logger.info(f"{device.getMxId()} {device.state}")
        print(f"{device}")
        cameras.append(
            {
                "serial_number": device.getMxId(),
                "name": device.name,
            }
        )

    if not cameras and raise_when_empty:
        raise RuntimeError("No cameras found.")

    return cameras


if __name__ == "__main__":
    cameras = find_cameras()
    logger.info(cameras)
    print(cameras)
