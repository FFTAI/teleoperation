import cv2
import pyrealsense2 as rs
import numpy as np

class Camera:
    def __init__(self, camera_type):
        print("Camera init...")
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        if camera_type not in {'l515', 'd435i'}:
            raise ValueError(f"Invalid group: '{camera_type}'. Expected one of {'l515', 'd435i'}.")
        self.camera_type = camera_type
        if self.camera_type == "l515":
            self.config.enable_stream(rs.stream.depth, 320, 240, rs.format.z16, 30)
            self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            self.pipeline_profile = self.pipeline.start(self.config)
            self.align_to = rs.stream.color
            self.align = rs.align(self.align_to)
            self.set_short_range_mode()
        elif self.camera_type == "d435i":
            self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            self.pipeline.start(self.config)
            self.align_to = rs.stream.color
            self.align = rs.align(self.align_to)
        
        #self.device = self.pipeline_profile.get_device()
        self.intrinsics = self.get_camera_intrinscis()

    def get_frames(self):
        self.frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(self.frames)
        return aligned_frames

    def get_camera_intrinscis(self):
        aligned_frames = self.get_frames()
        color_intrinsics = aligned_frames.profile.as_video_stream_profile().intrinsics
        camera_intrinsics = np.array(
            [[color_intrinsics.fx, 0, color_intrinsics.ppx],
            [0, color_intrinsics.fy, color_intrinsics.ppy],
            [0, 0, 1]])
        return camera_intrinsics

    def get_depth_img(self):
        aligned_frames = self.get_frames()
        depth_frame = aligned_frames.get_depth_frame()
        if not depth_frame:
            raise RuntimeError("Could not retrieve depth frame.")
        depth_image = np.asanyarray(depth_frame.get_data())
        new_depth_image = np.zeros((480, 640), dtype=depth_image.dtype)
        new_depth_image[:, :480] =  depth_image[:, :480] 
        depth_image = new_depth_image
        return depth_image

    def get_rgb_img(self):
        aligned_frames = self.get_frames()
        color_frame = aligned_frames.get_color_frame()
        if not color_frame:
            raise RuntimeError("Could not retrieve color frame.")
        color_image = np.asanyarray(color_frame.get_data())
        return color_image

    def set_short_range_mode(self):
        profile = self.pipeline.get_active_profile()
        device = profile.get_device()
        sensor = device.query_sensors()[0]
        sensor.set_option(rs.option.laser_power, 0)  
        sensor.set_option(rs.option.min_distance, 0.1)  
        laser_power = sensor.get_option(rs.option.laser_power)
        min_distance = sensor.get_option(rs.option.min_distance)

    def show_rgbd_img(self):
        rgb_image = self.get_rgb_img()
        depth_image = self.get_depth_img()
        cv2.imshow('RGB Image', rgb_image)
        depth_image_normalized = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)
        depth_image_colored = cv2.applyColorMap(depth_image_normalized.astype(np.uint8), cv2.COLORMAP_JET)
        cv2.imshow('Depth Image', depth_image_colored)
        if cv2.waitKey(1) & 0xFF == ord('q'):  
            self.pipeline.stop()
            cv2.destroyAllWindows()
        

if __name__ == "__main__":
    # camera = Camera("l515")
    camera = Camera("d435i")
    while True:
        depth_img = camera.get_depth_img()
        rgb_img = camera.get_rgb_img()
        cv2.imshow('rgb Image', rgb_img)
        cv2.imshow('depth Image', depth_img)
        print("# Click the mouse on the display window, Press the 'q' key to exit")
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cv2.destroyAllWindows()
