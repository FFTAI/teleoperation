import socket
import time
from collections.abc import Sequence
from typing import Protocol

from fourier_dhx.sdk.DexHand import DexHand
from loguru import logger


class Hand(Protocol):
    def init(self): ...
    def get_positions(self) -> Sequence[int]: ...

    def set_positions(self, positions: Sequence[int], wait_reply: bool = False): ...

    def reset(self): ...


class FourierDexHand:
    def __init__(self, hand_ip: str):
        self.hand = DexHand(hand_ip)

        self._hand_positions = [0] * 6

    def init(self):
        return self._reset()

    def _reset(self):
        _back1, _forward, _stop = (
            [-200, -200, -200, -200, -200, -200],
            [200, 200, 200, 200, 200, 200],
            [0, 0, 0, 0, 0, 0],
        )
        for i in range(10):
            m_last_cnt = self.hand.get_cnt()
            if len(m_last_cnt) != 6:
                logger.warning("calibration communication failed, try again...")
                if i == 9:
                    logger.error("calibration failed")
                    return False
                continue
            logger.info("calibration start")
            break

        self.hand.set_pwm(_back1)
        time.sleep(2)
        go_back_counts = 0

        for i in range(500):
            m_cur_cnt = self.hand.get_cnt()

            if len(m_cur_cnt) != 6:
                continue

            if m_cur_cnt == m_last_cnt:
                go_back_counts += 1
                if go_back_counts > 5:
                    self.hand.set_pwm(_back1)
                    time.sleep(2)
                    self.hand.calibration()
                    time.sleep(0.1)
                    logger.info("calibration success")
                    return True
                self.hand.set_pwm(_forward)
            else:
                self.hand.set_pwm(_back1)

            m_last_cnt = m_cur_cnt
            time.sleep(0.01)

        self.hand.set_pwm(_stop)
        time.sleep(2)
        logger.error("calibration failed")
        return False

    def get_positions(self):
        res = self.hand.get_angle()
        if isinstance(res, list) and len(res) == 6:
            self._hand_positions = res
        else:
            logger.warning(f"Getting hand pos error: {res}")
        return self._hand_positions

    def set_positions(self, positions, wait_reply=False):
        self.hand.set_angle(0, positions)

    def reset(self):
        self.hand.set_pwm([-200] * 6)
        time.sleep(2.0)
        self.hand.set_pwm([0] * 6)


class InspireDexHand:
    def __init__(self, ip: str, port: int = 2333, timeout: float = 0.1):
        """Simple UDP client for Inspire Dex hand control

        Args:
            ip (str): Hand IP address, usually 192.168.137.19 and 192.168.137.39
            port (int, optional): Hand UDP port. Defaults to 2333.
            timeout (float, optional): UDP timeout. Defaults to 0.1.
        """
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(timeout)
        self.ip = ip
        self.port = port

        self._hand_positions = [0] * 6

    def reset(self):
        self.set_positions([1000, 1000, 1000, 1000, 1000, 1000])

    def set_positions(self, positions: Sequence[int], wait_reply=True):
        """Set positions of the hand.

        Args:
            positions (Sequence[int]): 6 DOF positions, 1000 being fully open and 0 being fully closed.
            id (int, optional): Defaults to 1.

        Returns:
            _type_: _description_
        """
        send_data = bytearray()
        send_data.append(0xEB)  # 包头
        send_data.append(0x90)  # 包头
        send_data.append(1)  # 灵巧手 ID 号
        send_data.append(0x0F)  # 该帧数据部分长度 12 + 3
        send_data.append(0x12)  # 写寄存器命令标志
        send_data.append(0xCE)  # 寄存器起始地址低八位
        send_data.append(0x05)  # 寄存器起始地址高八位

        # Append val1 to val6 as little-endian
        positions = [int(pos) for pos in positions]
        for pos in positions:
            send_data.append(pos & 0xFF)
            send_data.append((pos >> 8) & 0xFF)

        # Calculate checksum
        checksum = sum(send_data[2:19]) & 0xFF
        send_data.append(checksum)

        self.sock.sendto(send_data, (self.ip, self.port))
        if not wait_reply:
            return None
        try:
            res, _ = self.sock.recvfrom(1024)
        except Exception as e:
            print(e)
            return None

        return res

    def get_positions(self, id: int = 1):
        send_data = bytearray()
        send_data.append(0xEB)  # 包头
        send_data.append(0x90)  # 包头
        send_data.append(int(id))
        send_data.append(0x04)
        send_data.append(0x11)  # kCmd_Handg3_Read
        send_data.append(0x0A)
        send_data.append(0x06)
        send_data.append(0x0C)

        checksum = sum(send_data[2:8]) & 0xFF
        send_data.append(checksum)

        self.sock.sendto(send_data, (self.ip, self.port))
        try:
            data, _ = self.sock.recvfrom(1024)
            received_checksum = data[19]
            calculated_checksum = sum(data[2:19]) & 0xFF

            if received_checksum != calculated_checksum:
                raise ValueError("Checksum mismatch")

            # print(data)
            pos = [
                data[7] | (data[8] << 8),
                data[9] | (data[10] << 8),
                data[11] | (data[12] << 8),
                data[13] | (data[14] << 8),
                data[15] | (data[16] << 8),
                data[17] | (data[18] << 8),
            ]

            self._hand_positions = pos
            return pos

        except Exception as e:
            print(e)
            return self._hand_positions
