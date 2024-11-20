import sys
import os
import time
import socket
import platform
import logging
import struct, json

class FunctionResult:
    SUCCESS = 0
    FAIL = -1
    RUNNING = 1
    PREPARE = 2
    EXECUTE = 3
    NOT_EXECUTE = 4
    TIMEOUT = 5

class SocketResult:
    SOCKET_SEND_FAILED = -10001,
    SOCKET_SEND_SIZE_WRONG = -10002,
    SOCKET_RECEIVE_FAILED = -10003,
    SOCKET_RECEIVE_SIZE_WRONG = -10004,


class HandSocket:
    def __init__(self, ip, timeout = 0.1):
        self.ip = ip
        self.ctrl_port = 2333
        self.comm_port = 2334
        self.fast_port = 2335
        self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.s.settimeout(timeout)
        self.s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

        
    def SendData(self, send_data, port):
        ret = self.s.sendto(send_data, (self.ip, port))
        if ret < 0:
            return SocketResult.SOCKET_SEND_FAILED
        if ret != len(send_data):
            return SocketResult.SOCKET_SEND_SIZE_WRONG
        return FunctionResult.SUCCESS
    
    def ReceiveData(self, size):
        try:
            data, address = self.s.recvfrom(size)
            return FunctionResult.SUCCESS, data, address
        except socket.timeout:
            return FunctionResult.TIMEOUT, None, None
        except:
            return FunctionResult.FAIL, None, None
        
    def SendRecvData(self, data, port):
        self.SendData(data, port)
        _, data, _ = self.ReceiveData(1500)
        return data
    
    def CtrlSendRecv(self, data):
        return self.SendRecvData(data, self.ctrl_port)
    
    def CommSendRecv(self, data):
        return self.SendRecvData(data, self.comm_port)
    
    def FastSend(self, data):
        self.SendData(data, self.fast_port)
        





    
        
    