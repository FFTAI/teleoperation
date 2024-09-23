import ipaddress
import json
import socket
import struct
import time

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.settimeout(0.01)
s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

fdh_comm_port = 2334


def broadcast():
    """
    发送广播消息，等待接收响应并返回解析后的响应数据以及响应地址。
    Args:
        无参数。
    Returns:
        tuple: 包含两个元素的元组,第一个元素为解析后的响应数据(dict类型), 第二个元素为响应地址(str类型)
            如果未接收到任何响应数据或发生异常，则返回 (None, None)。

    """
    s.sendto(str.encode("broadcast"), ("192.168.137.255", fdh_comm_port))
    while True:
        try:
            data, address = s.recvfrom(1024)
            datajson = json.loads(data)
            if datajson["type"] == "FDH-6R" or datajson["type"] == "FDH-6L":
                print(datajson)
                print("Found :", datajson["type"], address[0])
                return datajson, address[0]
        except TimeoutError:
            print("Didn't receive anymore data! [Timeout]")
            return None, None
        except:
            print("fi_dh.get_root() except")
            return None, None


def set_ip(com, ip):
    ip_address = ipaddress.IPv4Address(ip)
    int_ip = int(ip_address)
    bytes_ip = int_ip.to_bytes(4, byteorder="big")

    tx_messages = struct.pack(">B", 0x23)
    tx_messages += bytes_ip
    s.sendto(tx_messages, (str(com), fdh_comm_port))


def set_type(ip, type):
    tx_messages = struct.pack(">BB", 0x22, int(type))
    s.sendto(tx_messages, (str(ip), fdh_comm_port))


def set_sn(ip, sn):
    tx_messages = struct.pack(">B32s", 0x25, sn.encode())
    s.sendto(tx_messages, (str(ip), fdh_comm_port))


def reboot(fdh_ip):
    tx_messages = struct.pack(">B", 0xF1)
    s.sendto(tx_messages, (fdh_ip, fdh_comm_port))


def get_sn(ip):
    tx_messages = struct.pack(">B", 0x16)
    s.sendto(tx_messages, (str(ip), fdh_comm_port))
    try:
        data, address = s.recvfrom(1024)
        data = json.loads(data)
        data = data["sn"]
        chars = [chr(code) for code in data]
        data_str = "".join(chars)
        # print(data_str)
        return data_str

    except TimeoutError:  # fail after 1 second of no activity
        print("Didn't receive anymore data! [Timeout]")
        return " "
    except:
        print("fi_dh.get_root() except")
        return " "


def get_type(ip):
    tx_messages = struct.pack(">B", 0x16)
    s.sendto(tx_messages, (str(ip), fdh_comm_port))
    try:
        data, address = s.recvfrom(1024)
        data = json.loads(data)
        data = data["sn"]
        chars = [chr(code) for code in data]
        data_str = "".join(chars)
        # print(data_str)
        return data_str

    except TimeoutError:  # fail after 1 second of no activity
        print("Didn't receive anymore data! [Timeout]")
        return " "
    except:
        print("fi_dh.get_root() except")
        return " "


def main():
    bro, fdh_ip = broadcast()
    if bro == None:
        print("通信失败: 1. 检查电脑ip是否正确配置; 2. 检查线路是否正常")
        return

    ####请在此定义要设置的参数######################
    hand_type_left = 0
    hand_type_right = 1  # 设置左右手， 左手：0， 右手： 1

    ip_addr = "192.168.137.39"

    sn = "FDH6L24JL2600001"
    ############################################

    while True:
        print("请输入 0 / 1 / 2 / 3 / 4    :   0:退出循环   1 : 设置ip; 2: 设置sn; 3: 设置为左手; 4: 设置为右手")
        key = input()
        if key == "0":
            break

        if key == "1":
            set_ip(fdh_ip, ip_addr)
            time.sleep(1)

        if key == "2":
            set_sn(fdh_ip, sn)
            time.sleep(1)

        if key == "3":
            set_type(fdh_ip, hand_type_left)
            time.sleep(1)

        if key == "4":
            set_type(fdh_ip, hand_type_right)
            time.sleep(1)

    print("设置完成，设备正在重启中...")
    reboot(fdh_ip)
    time.sleep(3)
    print("重启完成,正在打印目前的参数 : [ ip  |  sn ]  ")
    bro, fdh_ip = broadcast()
    if bro == None:
        print("通信失败: 1. 检查电脑ip是否正确配置; 2. 检查线路是否正常")
        return
    print(fdh_ip)
    print(get_sn(fdh_ip))


if __name__ == "__main__":
    main()
