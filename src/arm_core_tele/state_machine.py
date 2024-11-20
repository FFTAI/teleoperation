'''
Author: Alvin_kalu guanwu.li@fftai.com
Date: 2024-09-26 16:09:35
LastEditors: Alvin_kalu guanwu.li@fftai.com
LastEditTime: 2024-10-16 03:59:18
FilePath: /data/robot_system/sim/state_machine.py
Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
'''
import pydds
import time

class StateMachine:
    def __init__(self, dds_context, targets):
        self.dds_context = dds_context
        self.targets     = targets
        self.request_messages = dict()
        self.subscriber = self.make_subscriber(dds_context)
        for target in self.targets:
            self.request_messages[target] = self.make_message(target)

    def get_name(self):
        return "StateMachine"

    def get_latencys(self):
        states = dict()
        for target in self.targets:
            states[target] = self.subscriber.message_latency_ns(target)
        return states

    def get_states(self):
        states = dict()
        for target in self.targets:
            states[target] = self.subscriber.current_message(target)
        return states

class PVCStateMachine(StateMachine):
    def get_name(self):
        return "PVCState"

    def make_subscriber(self, dds_context):
        return pydds.PVCStateResponseSubscriber(dds_context, "/fftai/gr1t2/pvc_state/response", True, 5000)

    def make_message(self, target):
        return pydds.PVCStateRequest(target, True, True, True)

class IMUStateMachine(StateMachine):
    def get_name(self):
        return "IMUState"
    
    def make_subscriber(self, dds_context):
        return pydds.IMUStateResponseSubscriber(dds_context, "/fftai/gr1t2/imu_state/response", True, 5000)

    def make_message(self, target):
        return pydds.IMUStateRequest(target)

class SendOnly:
    def __init__(self, publisher, subscriber):
        self.messages   = dict()
        self.publisher  = publisher
        self.subscriber = subscriber

    def push(self, message):
        self.messages[message.target] = message

    def responses(self, targets):
        return {target : self.subscriber.current_message(target) for target in targets}

    def have_responses(self, targets):
        return {target : self.subscriber.have_new_message(target) for target in targets}

    def response(self, target):
        return self.subscriber.current_message(target)

    def have_response(self, target):
        return self.subscriber.have_new_message(target)

    def emit(self):
        messages = self.messages
        self.messages = dict() # Clean old

        for target in messages:
            # print(f"Send message to target [{target}]. msg: {messages[target]}")
            self.publisher.publish(messages[target])

class MotorControl(SendOnly):
    def __init__(self, dds_context):
        super().__init__(
            pydds.MotorControlRequestPublisher(dds_context, "/fftai/gr1t2/motor_control/request", True, 5000), 
            pydds.MotorControlResponseSubscriber(dds_context, "/fftai/gr1t2/motor_control/response", True, 5000)
        )
    
    def push(self, target, control_word):
        super().push(pydds.MotorControlRequest(target, control_word))

    def pushs(self, targets, control_word):
        for target in targets:
            super().push(pydds.MotorControlRequest(target, control_word))

class Encoder(SendOnly):
    def __init__(self, dds_context):
        super().__init__(
            pydds.EncoderStateRequestPublisher(dds_context, "/fftai/gr1t2/encoder_state/request", True, 5000),
            pydds.EncoderStateResponseSubscriber(dds_context, "/fftai/gr1t2/encoder_state/response", True, 5000)
        )
    
    def push(self, target):
        super().push(pydds.EncoderStateRequest(target))

    def pushs(self, targets):
        for target in targets:
            super().push(pydds.EncoderStateRequest(target))

class OperationMode(SendOnly):
    def __init__(self, dds_context):
        super().__init__(
            pydds.OperationModeRequestPublisher(dds_context, "/fftai/gr1t2/operation_mode/request", True, 5000), 
            pydds.OperationModeResponseSubscriber(dds_context, "/fftai/gr1t2/operation_mode/response", True, 5000)
        )

    def push(self, target, mode_of_operation):
        super().push(pydds.OperationModeRequest(target, mode_of_operation))

    def pushs(self, targets, mode_of_operation):
        for target in targets:
            super().push(pydds.OperationModeRequest(target, mode_of_operation))

class PositionControlMode(SendOnly):
    def __init__(self, dds_context):
        super().__init__(
            pydds.PositionControlRequestPublisher(dds_context, "/fftai/gr1t2/position_control/request", True, 5000), 
            pydds.PositionControlResponseSubscriber(dds_context, "/fftai/gr1t2/position_control/response", True, 5000)
        )

    def push(self, target, pos):
        super().push(pydds.PositionControlRequest(target, pos))

    def pushs(self, targets, pos):
        for target in targets:
            super().push(pydds.PositionControlRequest(target, pos))

class PIDIMMSetMode(SendOnly):
    def __init__(self, dds_context):
        super().__init__(
            pydds.PIDIMMSetRequestPublisher(dds_context, "/fftai/gr1t2/pid_imm_set/request", True, 5000), 
            pydds.PIDIMMSetResponseSubscriber(dds_context, "/fftai/gr1t2/pid_imm_set/response", True, 5000)
        )

    def push(self, target, control_position_kp_imm, control_velocity_kp_imm, control_velocity_ki_imm):
        super().push(pydds.PIDIMMSetRequest(target, control_position_kp_imm, control_velocity_kp_imm, control_velocity_ki_imm))

    def pushs(self, targets, control_position_kp_imm, control_velocity_kp_imm, control_velocity_ki_imm):
        for target in targets:
            super().push(pydds.PIDIMMSetRequest(target, control_position_kp_imm, control_velocity_kp_imm, control_velocity_ki_imm))

class DDSPipeline:
    def __init__(self, joints:dict, encoders:list, imu:str, freq:int=50, use_imu:bool=False):
        self.context    = pydds.Context()
        self.joints     = joints
        self.encoder_names = list(encoders.keys())
        self.encoders      = encoders
        self.imu_name      = imu
        self.use_imu        = use_imu
        self.freq          = freq
        self.joint_names      = list(joints.keys())
        self.encoder_control  = Encoder(self.context)
        self.motor_control    = MotorControl(self.context)
        self.operation_mode   = OperationMode(self.context)
        self.position_control = PositionControlMode(self.context)
        self.pidimm_control   = PIDIMMSetMode(self.context)
        self.pvc_state        = PVCStateMachine(self.context, self.joint_names)
        if self.use_imu:
            self.imu_state    = IMUStateMachine(self.context, [imu])
        
    def destroy(self):
        if self.use_imu:
            del self.imu_state
        del self.pvc_state
        del self.pidimm_control
        del self.position_control
        del self.operation_mode
        del self.motor_control
        del self.encoder_control
        del self.context

    def set_control_mode(self):
        self.operation_mode.pushs(self.joint_names, 0x01)
        self.operation_mode.emit()
        time.sleep(0.01)

    def enable_joints(self):
        time.sleep(0.1)
        joint_states, integrality = self.get_pvc_states()
        
        if not integrality:
            for j, v in joint_states.items():
                if v.status != "OK":
                    print(f'{j}: {v.status}')
        assert integrality, f"PVC states is not integrality"
        
        joint_pos = []

        for k, v in joint_states.items():
            joint_pos.append([k, v.position])
        
        self.move_joints(joint_pos)
        time.sleep(0.1)

        self.operation_mode.pushs(self.joint_names, 0x01)
        self.motor_control.pushs(self.joint_names, 0x0F)
        self.operation_mode.emit()
        self.motor_control.emit()
        time.sleep(0.01)
    
    def disable_joints(self):
        self.motor_control.pushs(self.joint_names, 0x06)
        self.motor_control.emit()
        time.sleep(0.01)

    def set_joints_pid_param(self):
        for joint_name in self.joints:
            joint = self.joints[joint_name]
            if joint["enable"]:
                position_kp = joint["position_kp"]
                velocity_kp = joint["velocity_kp"]
                print(f"Set joint [{joint_name}] pid: position_kp = {position_kp}, velocity_kp = {velocity_kp}")
                self.pidimm_control.push(joint_name, position_kp, velocity_kp, 0.0)
        self.pidimm_control.emit()

    def move_joints(self, pairs_joint_and_position):
        for joint, position in pairs_joint_and_position:
            self.position_control.push(joint, position)
        self.position_control.emit()

    def get_pvc_latencys(self):
        return self.pvc_state.get_latencys()

    def get_pvc_states(self):
        states = self.pvc_state.get_states()
        integrality = all([states[target].status == "OK" for target in self.joint_names])
        return states, integrality

    def get_encoders_state(self, retry=5):
        integrality = False
        responses   = None
        while retry >= 0:
            self.encoder_control.pushs(self.encoder_names)
            self.encoder_control.emit()
            time.sleep(0.3)
            have_responses = self.encoder_control.have_responses(self.encoder_names)
            have_response  = [have_responses[key] for key in have_responses]
            
            if all(have_response):
                responses      = self.encoder_control.responses(self.encoder_names)
                status_ok      = [responses[key].status == "OK" for key in responses]
                integrality    = all(status_ok)
            
                if integrality:
                    break
                
                keys = list(responses.keys())
                error_message = ""
                for i, v in enumerate(status_ok):
                    if not v:
                        key = keys[i]
                        error_message += f"{key}.status is [{responses[key].status}]\n"
                
                for i, v in enumerate(have_response):
                    if not v:
                        key = keys[i]
                        error_message += f"{key} have no response\n"

                error_message = error_message.strip()
            else:
                error_message = "Failed."

            print(f"Warning[retry = {retry}]: Can not fetch the whole encoders_state. Details are as below:\n{error_message}")
            time.sleep(0.2)
            retry -= 1
        return responses, integrality