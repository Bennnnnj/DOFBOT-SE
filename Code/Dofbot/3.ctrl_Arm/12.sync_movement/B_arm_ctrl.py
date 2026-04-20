#coding=utf-8
# B_arm_ctrl.py — TCP client 端（运行在主控电脑 / 另一台 Raspberry Pi 上）
# 作用：实时读取本机机械臂的6个舵机角度，通过 TCP 发给 A_arm_follow
# 效果：两台机械臂同步运动 —— 手动掰动 B 这台，A 那台跟着动（"影随"模式）
# 通信链路：STM32 → UART → Arm_Lib → B_arm_ctrl → TCP → A_arm_follow → Arm_Lib → UART → STM32

import socket
import os
import time
from Arm_Lib import Arm_Device          # 导入 UART 驱动，读取本机舵机角度



global g_sock
Arm = Arm_Device()                      # 实例化本机机械臂（B 机），用于读取角度


# TCP client：连接 A 机的 server，持续读取本机角度并发送
def connect_tcp_server(ip, port):
    global g_sock
    print("Connecting server...")
    g_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # TCP socket
    g_sock.connect((ip, port))          # 连接 A_arm_follow 的 TCP server
    print("Connected!")
    time.sleep(1)
    Arm.Arm_serial_set_torque(0)        # 关闭 B 机舵机扭矩，让舵机可以被手动掰动（零阻力模式）
    last_angle = [0, 0, 0, 0, 0, 0]    # 上一帧有效角度，用于读取失败时的 fallback
    while True:
        angle = [0, 0, 0, 0, 0, 0]
        for i in range(6):              # 依次读取 servo 1~6 的当前角度
            id = i + 1
            angle[i] = Arm.Arm_serial_servo_read(id)
            if angle[i] == None:        # 读取失败（UART 超时或校验错），重试一次
                time.sleep(.001)
                angle[i] = Arm.Arm_serial_servo_read(id)
                if angle[i] == None:
                    angle[i] = last_angle[i]  # 重试仍失败，用上一帧的值保持连续性

            last_angle[i] = angle[i]    # 更新 fallback 缓存
            time.sleep(.001)

        # 把6个浮点角度格式化为3位补零字符串，拼成协议帧
        # 格式：$20AAA BBB CCC DDD EEE FFF#，总长22字符
        pos1str = "%03d" % angle[0]     # e.g. 90 → "090"
        pos2str = "%03d" % angle[1]
        pos3str = "%03d" % angle[2]
        pos4str = "%03d" % angle[3]
        pos5str = "%03d" % angle[4]
        pos6str = "%03d" % angle[5]
        data = "$20"+pos1str+pos2str+pos3str+pos4str+pos5str+pos6str+"#"
        print(data)
        b_data = bytes(data, encoding = "utf8")  # 字符串转 bytes，TCP 只能发 bytes
        g_sock.send(b_data)             # 发送给 A_arm_follow
        time.sleep(.1)                  # 10Hz 发送频率（每100ms一帧）



# 关闭 socket 并恢复 B 机舵机扭矩（Ctrl+C 触发时清理）
def waitClose(sock):
    sock.close()
    Arm.Arm_serial_set_torque(1)        # 恢复扭矩，防止 B 机断电后舵机自由垂落


if __name__ == '__main__':
    #根据服务器的IP地址修改以下参数
    ip = '192.168.2.100'               # A 机（运行 A_arm_follow.py）的 IP 地址
    port = 6100                        # 与 A_arm_follow 约定的端口
    try:
        connect_tcp_server(ip, port)
    except KeyboardInterrupt:
        waitClose(g_sock)               # Ctrl+C → 关闭连接，恢复扭矩
        print(" Program closed! ")
        pass
