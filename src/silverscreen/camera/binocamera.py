import cv2
import numpy as np


class Camera:
    def __init__(
        self,
        camera_index: int = 0,
        resolution: tuple[int, int] = (720, 1280),
        crop_size_h: int = 0,
        crop_size_w: int = 1,
        open=True,
    ):
        self.is_recording = False

        self.crop_size_h = crop_size_h
        self.crop_size_w = crop_size_w
        self.resolution_cropped = (
            resolution[0] - crop_size_h,
            resolution[1] - 2 * crop_size_w,
        )
        self.img_shape = (self.resolution_cropped[0], self.resolution_cropped[1], 3)
        self.img_height, self.img_width = self.resolution_cropped[:2]

        if open:
            # Open the camera using cv2
            self.cap = cv2.VideoCapture(camera_index)

            if not self.cap.isOpened():
                print("Error: Could not open camera.")
                exit()

            # Set the desired resolution
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, resolution[1])
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, resolution[0])

    def start_recording(self, output_path: str):
        # Define the codec and create VideoWriter object
        fourcc = cv2.VideoWriter_fourcc(*"XVID")
        self.out = cv2.VideoWriter(output_path, fourcc, 20.0, (self.img_width, self.img_height))
        self.is_recording = True

    def stop_recording(self):
        self.out.release()
        self.is_recording = False

    def grab(self, gray=False) -> tuple[int, np.ndarray]:
        ret, frame = self.cap.read()

        if not ret:
            raise Exception("Failed to grab image")

        # Crop the frame
        cropped_frame = frame[self.crop_size_h :, self.crop_size_w : -self.crop_size_w]

        if gray:
            # Convert to grayscale if requested
            gray_img = cv2.cvtColor(cropped_frame, cv2.COLOR_BGR2GRAY)
            rgb = cv2.cvtColor(gray_img, cv2.COLOR_GRAY2RGB)

        # Get a timestamp (optional: use more accurate timing if needed)
        timestamp = cv2.getTickCount()

        return timestamp, rgb

    def release(self):
        # Release the camera resource
        self.cap.release()
        cv2.destroyAllWindows()


def main():
    # Initialize the Camera object
    camera = Camera(camera_index=0, resolution=(720, 1280), crop_size_h=0, crop_size_w=0, open=True)

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
