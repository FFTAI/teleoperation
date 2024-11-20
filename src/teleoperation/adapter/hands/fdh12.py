from dexhand import *

class DexHand():
    def __init__(self, ip, timeout = 0.1):
        self.ctrl_port = 2333
        self.comm_port = 2334
        self.fast_port = 2335
        self.s = HandSocket(ip, timeout)
        
    def ctrl_set_position(self, position):
        if (isinstance(position, list) == False and len(position) == 12):
            print("ctrl_set_position(): param check failed")
            return False
        data = {
            "method": "SET",
            "cmd": "/pos",
            "position": position
            }
        data = json.dumps(data)
        return self.s.CtrlSendRecv(data.encode())
    
    def ctrl_set_enable(self):
        data = {
            "method": "SET",
            "cmd": "/enable"
            }
        data = json.dumps(data)
        return self.s.CtrlSendRecv(data.encode())
    
    def calibration(self):
        print("[INFO] [fhh12.py][calibration()] Start ", end="")
        data = {
            "method": "SET",
            "cmd": "/calibration"
            }
        data = json.dumps(data)
        reply = self.s.CtrlSendRecv(data.encode())
        if reply == None:
            return False
        for _ in range(50):
            cpu0, cpu1 =  self.get_errorcode()
            if cpu0 == 0xFFFFFFFF:
                print(".", end="")
                time.sleep(0.3)
                continue
            elif cpu0 == 0x01:
                print(".")
                print("[Error] [fhh12.py][calibration()] calibration failed")
                return False
            elif cpu0 == 0x00:
                print(".")
                print("[INFO] [fhh12.py][calibration()] calibration successfully")
                return True
        return False
            
    def get_errorcode(self):
        data = {
            "method": "GET",
            "cmd": "/errorcode"
            }
        data = json.dumps(data)
        data = self.s.CommSendRecv(data.encode())
        if data != None:
            data = json.loads(data)
            return data["errorcode"][0], data["errorcode"][1] 
        return None
    
    def ctrl_set_disable(self):
        data = {
            "method": "SET",
            "cmd": "/disable"
            }
        data = json.dumps(data)
        return self.s.CtrlSendRecv(data.encode())
    
    def ctrl_set_reboot(self):
        data = {
            "method": "SET",
            "cmd": "/reboot"
            }
        data = json.dumps(data)
        return self.s.CtrlSendRecv(data.encode())
    
    def ctrl_clear_errorcode(self):
        data = {
            "method": "SET",
            "cmd": "/clear_error"
            }
        data = json.dumps(data)
        return self.s.CtrlSendRecv(data.encode())
    
    def ctrl_set_config(self, config):
        data = {
            "method": "SET",
            "cmd": "/config",
            "pid_idx": config.get("pid_idx"),
            "pid": config.get("pid")
            }
        data = json.dumps(data)
        return self.s.CtrlSendRecv(data.encode())
    
    def ctrl_set_duty(self, duty, id = [1,2,3,4,5,6,7,8,9,10,11,12]):
        if (isinstance(duty, list) == False and len(duty) == 12):
            print("ctrl_set_position(): param check failed")
            return False
        data = {
            "method": "SET",
            "cmd": "/pwm_control",
            "fingeridx": id,
            "fingerduty": duty
            }
        data = json.dumps(data)
        return self.s.CtrlSendRecv(data.encode())
    
    def ctrl_calibration(self):
        data = {
            "method": "SET",
            "cmd": "/calibration"
            }
        data = json.dumps(data)
        return self.s.CtrlSendRecv(data.encode())
    
    def comm_get_config(self):
        data = {
            "method": "GET",
            "cmd": "/config"
        }
        data = json.dumps(data)
        return self.s.CommSendRecv(data.encode())
    
    def comm_get_errorcode(self):
        data = {
            "method": "GET",
            "cmd": "/errorcode"
        }
        data = json.dumps(data)
        return self.s.CommSendRecv(data.encode())
    
    def comm_set_config(self, config):
        data = {
            "method": "SET",
            "cmd": "/config",
            "param": "test"
        }
        data = json.dumps(data)
        return self.s.CommSendRecv(data.encode())
    
    def comm_get_pva(self):
        data = {
            "method": "GET",
            "cmd": "/pva"
        }
        data = json.dumps(data)
        return self.s.CommSendRecv(data.encode())
    
    def comm_get_matrix(self):
        data = {
            "method": "GET",
            "cmd": "/matrix"
        }
        data = json.dumps(data)
        fdb_data = self.s.CommSendRecv(data.encode())
        json_obj = json.loads(fdb_data.decode("utf-8"))
        if json_obj:
            matrix = json_obj.get("matrix")
            packed_data = []
            for num in matrix:
                packed_data.extend([(num >> 16) & 0xFF, (num >> 8) & 0xFF, num & 0xFF])
            new_list = [packed_data[i:i + 96] for i in range(0, len(packed_data), 96)]
            return new_list
        
    def comm_get_tashan(self):
        data = {
            "method": "GET",
            "cmd": "/tashan"
        }
        data = json.dumps(data)
        return self.s.CommSendRecv(data.encode())
    
    
        
