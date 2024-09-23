import cv2
import numpy as np
import pyrealsense2 as rs

from . import CameraBase


class CamRealsense(CameraBase):
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
            # Configure RealSense pipeline
            self.pipeline = rs.pipeline()
            config = rs.config()

            # Set resolution and frame rate for the RealSense camera
            config.enable_stream(rs.stream.color, resolution[1], resolution[0], rs.format.bgr8, fps)

            # Start streaming
            self.pipeline.start(config)

    def start_recording(self, output_path: str):
        # Define the codec and create VideoWriter object
        fourcc = cv2.VideoWriter_fourcc(*"XVID")
        self.out = cv2.VideoWriter(output_path, fourcc, 20.0, (self.img_width, self.img_height))
        self.is_recording = True

    def stop_recording(self):
        self.out.release()
        self.is_recording = False

    def grab(self, gray=False, enable_depth=False) -> tuple[int, np.ndarray | tuple[np.ndarray, np.ndarray]]:
        # Wait for a coherent pair of frames: depth and color
        frames = self.pipeline.wait_for_frames()

        depth_array = None
        if enable_depth:
            align_to = rs.stream.depth
            align = rs.align(align_to)
            aligned_frames = align.process(frames)
            timestamp = aligned_frames.get_timestamp()
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()
            color_array = np.asanyarray(color_frame.get_data())
            depth_array = np.asanyarray(depth_frame.get_data())

        else:
            timestamp = frames.get_timestamp()
            color_frame = frames.get_color_frame()

        if not color_frame:
            raise Exception("Failed to grab image")

        # Convert images to numpy arrays
        color_array = np.asanyarray(color_frame.get_data())[self.crop_size_h :, self.crop_size_w : -self.crop_size_w]

        if gray:
            gray = cv2.cvtColor(color_array, cv2.COLOR_BGR2GRAY)
            rgb = cv2.cvtColor(gray, cv2.COLOR_GRAY2RGB)
        else:
            rgb = cv2.cvtColor(color_array, cv2.COLOR_BGR2RGB)

        if enable_depth and depth_array is not None:
            return timestamp, (rgb, depth_array)

        return timestamp, rgb

    def close(self):
        # Stop the RealSense pipeline
        self.pipeline.stop()
        cv2.destroyAllWindows()


def main():
    # Initialize the Camera object
    camera = CamRealsense(index=0, resolution=(720, 1280), crop_size_h=0, crop_size_w=1, open=True)

    # Loop to continuously grab frames
    while True:
        try:
            # Grab a frame from the camera
            timestamp, frame = camera.grab(gray=False)  # Set gray=True if you want grayscale

            # Show the frame in a window
            cv2.imshow("Camera Feed", frame)

            # Check if 'q' is pressed to break the loop
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

        except Exception as e:
            print(f"An error occurred: {e}")
            break

    # Release the camera resources
    camera.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
