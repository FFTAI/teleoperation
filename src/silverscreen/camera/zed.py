import cv2
import numpy as np
import pyzed.sl as sl

from . import CameraBase

# resolution = (720, 1280)
# crop_size_w = 1
# crop_size_h = 0
# resolution_cropped = (resolution[0] - crop_size_h, resolution[1] - 2 * crop_size_w)


class CamZed(CameraBase):
    def __init__(
        self,
        index: int = 0,
        fps: int = 60,
        resolution: tuple[int, int] = (720, 1280),
        crop_size_h: int = 0,
        crop_size_w: int = 1,
        open=True,
    ):
        super().__init__(index, fps, resolution, crop_size_h, crop_size_w, open)

        if open:
            # Create a InitParameters object and set configuration parameters
            init_params = sl.InitParameters()
            init_params.camera_resolution = (
                sl.RESOLUTION.HD720
            )  # Use HD720 opr HD1200 video mode, depending on camera type.
            init_params.camera_fps = fps  # Set fps at 60
            self.zed = sl.Camera()
            err = self.zed.open(init_params)

            if err != sl.ERROR_CODE.SUCCESS:
                print("Camera Open : " + repr(err) + ". Exit program.")
                exit()

            self.image_left = sl.Mat()
            self.image_right = sl.Mat()
            self.runtime_parameters = sl.RuntimeParameters()

    def start_recording(self, output_path: str):
        recording_params = sl.RecordingParameters(output_path, sl.SVO_COMPRESSION_MODE.H264)
        err = self.zed.enable_recording(recording_params)
        self.is_recording = True

    def stop_recording(self):
        self.zed.disable_recording()
        self.is_recording = False

    def grab(self, gray=False, enable_depth=False) -> tuple[float, np.ndarray]:
        if self.zed.grab(self.runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            self.zed.retrieve_image(self.image_left, sl.VIEW.LEFT)
            self.zed.retrieve_image(self.image_right, sl.VIEW.RIGHT)
            timestamp = self.zed.get_timestamp(sl.TIME_REFERENCE.IMAGE)
        else:
            raise Exception("Failed to grab image")

        bgr = np.hstack(
            (
                self.image_left.numpy()[self.crop_size_h :, self.crop_size_w : -self.crop_size_w],
                self.image_right.numpy()[self.crop_size_h :, self.crop_size_w : -self.crop_size_w],
            )
        )

        if gray:
            gray = cv2.cvtColor(bgr, cv2.COLOR_BGRA2GRAY)
            rgb = cv2.cvtColor(gray, cv2.COLOR_GRAY2RGB)
        else:
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGRA2RGB)

        return timestamp.get_milliseconds(), rgb

    def close(self):
        self.image_left.free(sl.MEM.CPU)
        self.image_right.free(sl.MEM.CPU)
        self.zed.close()
