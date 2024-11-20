import cv2
import pyzed.sl as sl
import numpy as np

class Camera:
    def __init__(self):
        print("######## Zed Camera init ########")
        self.zed = sl.Camera()
        self.init_params = sl.InitParameters()
        self.init_params.camera_resolution = sl.RESOLUTION.HD720  
        self.init_params.camera_fps = 60  
        self.init_params.depth_mode = sl.DEPTH_MODE.ULTRA  
        self.init_params.coordinate_units = sl.UNIT.METER 
        self.runtime_params = sl.RuntimeParameters()

        if self.zed.open(self.init_params) != sl.ERROR_CODE.SUCCESS:
            print("Unable to open the ZED camera")
            return

        self.rgb_image = sl.Mat()
        self.depth_image = sl.Mat()
        self.intrinsics = self.get_camera_intrinscis()

    def get_camera_intrinscis(self):
        calibration_params = self.zed.get_camera_information().camera_configuration.calibration_parameters.left_cam
        fx = calibration_params.fx  
        fy = calibration_params.fy  
        cx = calibration_params.cx  
        cy = calibration_params.cy  
        camera_intrinsics = np.array([[fx, 0, cx],
                                      [0, fy, cy],
                                      [0, 0, 1]], dtype=np.float64)
        return camera_intrinsics

    def get_depth_img(self):
        if self.zed.grab(self.runtime_params) == sl.ERROR_CODE.SUCCESS:
            self.zed.retrieve_measure(self.depth_image, sl.MEASURE.DEPTH)
            depth_array = self.depth_image.get_data()
            return depth_array
        else:
            print("Failed to retrieve depth image")
            return None

    def get_rgb_img(self):
        if self.zed.grab(self.runtime_params) == sl.ERROR_CODE.SUCCESS:
            self.zed.retrieve_image(self.rgb_image, sl.VIEW.LEFT)
            rgb_array = self.rgb_image.get_data()[:, :, :3] 
            return rgb_array
        else:
            print("Failed to retrieve RGB image")
            return None



if __name__ == "__main__":
    camera = Camera()
    while True:
        depth_img = camera.get_depth_img()
        rgb_img = camera.get_rgb_img()
        cv2.imshow('rgb Image', rgb_img)
        cv2.imshow('depth Image', depth_img)
        print("# Click the mouse on the display window, Press the 'q' key to exit")
        # Press the 'q' key to exit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cv2.destroyAllWindows()
