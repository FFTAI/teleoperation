'''
Author: Alvin_kalu guanwu.li@fftai.com
Date: 2024-09-25 13:21:22
LastEditors: Alvin_kalu guanwu.li@fftai.com
LastEditTime: 2024-10-15 08:42:36
FilePath: /data/robot_system/sim/pydds/__init__.py
Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
'''

class Context:
    def __init__(self, domain:int = 0):
        ...

class SystemStateRequest:
    def __init__(self, target:str):
        ...

class SystemStateResponse:
    def __init__(self, source:str, timestamp:int, state:int, status:str):
        ...

class SystemStateRequestPublisher:
    def __init__(self, context:Context, topic_name:str, wait_for_matched:bool, wait_timeout_ms:int):
        ...

class SystemStateResponseSubscriber:
    def __init__(self, context:Context, topic_name:str, wait_for_matched:bool, wait_timeout_ms:int):
        ...


class PVCStateRequest:
    def __init__(self, target:str, position:bool, velocity:bool, current:bool):
        ...

class PVCStateResponse:
    def __init__(self, source:str, timestamp:int, status:str, position:float, velocity:float, current:float):
        ...

class PVCStateRequestPublisher:
    def __init__(self, context:Context, topic_name:str, wait_for_matched:bool, wait_timeout_ms:int):
        ...

class PVCStateResponseSubscriber:
    def __init__(self, context:Context, topic_name:str, wait_for_matched:bool, wait_timeout_ms:int):
        ...


class PositionControlRequest:
    def __init__(self, target:str, position:float):
        ...

class PositionControlResponse:
    def __init__(self, source:str, timestamp:int, status:str, position:float, velocity:float, current:float, current_id:int):
        ...

class PositionControlRequestPublisher:
    def __init__(self, context:Context, topic_name:str, wait_for_matched:bool, wait_timeout_ms:int):
        ...

class PositionControlResponseSubscriber:
    def __init__(self, context:Context, topic_name:str, wait_for_matched:bool, wait_timeout_ms:int):
        ...


class MotorControlRequest:
    def __init__(self, target:str, control_word:int):
        ...

class MotorControlResponse:
    def __init__(self, source:str, timestamp:int, status:str):
        ...

class MotorControlRequestPublisher:
    def __init__(self, context:Context, topic_name:str, wait_for_matched:bool, wait_timeout_ms:int):
        ...

class MotorControlResponseSubscriber:
    def __init__(self, context:Context, topic_name:str, wait_for_matched:bool, wait_timeout_ms:int):
        ...


class OperationModeRequest:
    def __init__(self, target:str, mode_of_operation:int):
        ...

class OperationModeResponse:
    def __init__(self, source:str, timestamp:int, status:str):
        ...

class OperationModeRequestPublisher:
    def __init__(self, context:Context, topic_name:str, wait_for_matched:bool, wait_timeout_ms:int):
        ...

class OperationModeResponseSubscriber:
    def __init__(self, context:Context, topic_name:str, wait_for_matched:bool, wait_timeout_ms:int):
        ...


class IMUStateRequest:
    def __init__(self, target:str):
        ...

class IMUStateResponse:
    def __init__(self, source:str, timestamp:int, status:str, frame_type:str,
                 temperature:float, pressure:float, system_time_ms:int, sync_time:float, 
                 roll:float, pitch:float, yaw:float,
                 acceleration_x:float, acceleration_y:float, acceleration_z:float,
                 gyroscope_x:float, gyroscope_y:float, gyroscope_z:float,
                 magnetometer_x:float, magnetometer_y:float, magnetometer_z:float,
                 quaternion_x:float, quaternion_y:float, quaternion_z:float, quaternion_w:float):
        ...

class IMUStateRequestPublisher:
    def __init__(self, context:Context, topic_name:str, wait_for_matched:bool, wait_timeout_ms:int):
        ...

class IMUStateResponseSubscriber:
    def __init__(self, context:Context, topic_name:str, wait_for_matched:bool, wait_timeout_ms:int):
        ...


class EncoderStateRequest:
    def __init__(self, target:str):
        ...

class EncoderStateResponse:
    def __init__(self, source:str, timestamp:int, status:str, angle:float, radian:float):
        ...

class EncoderStateRequestPublisher:
    def __init__(self, context:Context, topic_name:str, wait_for_matched:bool, wait_timeout_ms:int):
        ...

class EncoderStateResponseSubscriber:
    def __init__(self, context:Context, topic_name:str, wait_for_matched:bool, wait_timeout_ms:int):
        ...


class PIDIMMSetRequest:
    def __init__(self, target:str, control_position_kp_imm:float, control_velocity_kp_imm:float, control_velocity_ki_imm:float):
        ...

class PIDIMMSetResponse:
    def __init__(self, source:str, timestamp:int, status:str):
        ...

class PIDIMMSetRequestPublisher:
    def __init__(self, context:Context, topic_name:str, wait_for_matched:bool, wait_timeout_ms:int):
        ...

class PIDIMMSetResponseSubscriber:
    def __init__(self, context:Context, topic_name:str, wait_for_matched:bool, wait_timeout_ms:int):
        ...


class PIDIMMGetRequest:
    def __init__(self, target:str):
        ...

class PIDIMMGetResponse:
    def __init__(self, source:str, timestamp:int, status:str, control_position_kp_imm:float, control_velocity_kp_imm:float, control_velocity_ki_imm:float, control_current_kp_imm:float, control_current_ki_imm:float):
        ...

class PIDIMMGetRequestPublisher:
    def __init__(self, context:Context, topic_name:str, wait_for_matched:bool, wait_timeout_ms:int):
        ...

class PIDIMMGetResponseSubscriber:
    def __init__(self, context:Context, topic_name:str, wait_for_matched:bool, wait_timeout_ms:int):
        ...


from .libpydds import *