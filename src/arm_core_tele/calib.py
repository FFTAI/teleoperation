from state_machine import DDSPipeline
import json
import os
import time

class Robotic:
    def __init__(self):
        self.config = self.load_config()
        joints = self.config["joints"]
        self.connector = DDSPipeline(
            joints, self.config["encoders"], self.config["imu"], self.config["freq"]
        )

    def init_encoders(self):
        encoders_state, integrality = self.connector.get_encoders_state()
        assert integrality, f"Error: Can not fetch the whole encoders_state."

        encoders_state_file = self.config["encoders_state_file"]
        assert os.path.exists(encoders_state_file), f"Not found encoders state file[{encoders_state_file}]. Calibration data must be exist."

        print(f"Load encoders state from {encoders_state_file}")
        self.encoders_state = json.load(open(encoders_state_file, "r"))

        if integrality:
            print(f"Save encoders calibration state to file {encoders_state_file}.")
            for name in encoders_state:
                angle = encoders_state[name].angle
                self.encoders_state[name]["calibration_pose"] = angle

            with open(encoders_state_file, "w") as f:
                json.dump(self.encoders_state, f, indent=4)

    def start(self):
        self.init_encoders()
        
    def stop(self):
        self.connector.destroy()

    def load_config(self):
        with open("cfgs/client_config.json", "r") as f:
            config = json.load(f)
        return config

def main():
    """
    主函数，用于启动机器人程序。
    """
    robot = Robotic()
    robot.start()
    robot.stop()
    
if __name__ == "__main__":
    main()
