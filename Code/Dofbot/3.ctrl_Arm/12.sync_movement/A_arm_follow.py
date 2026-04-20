#coding=utf-8
# A_arm_follow.py — TCP server 端（运行在机械臂 Raspberry Pi 上）
# 作用：监听来自 B_arm_ctrl.py 的网络指令，解析后驱动本机械臂跟随运动
# 通信链路：B_arm_ctrl (主机) → TCP socket → A_arm_follow (Pi) → Arm_Lib → UART → STM32 → 舵机

import socket
import os
import time
from Arm_Lib import Arm_Device          # 导入 UART 驱动，控制本机舵机

global g_init
g_init = False
Arm = Arm_Device()                      # 实例化机械臂，打开 /dev/myserial


# 获取本机 IP 地址（优先有线 eth0，fallback 无线 wlan0）
def getLocalip():
    ip = os.popen("/sbin/ifconfig eth0 | grep 'inet' | awk '{print $2}'").read()
    ip = ip[0 : ip.find('\n')]          # 去掉末尾换行符
    if(ip == ''):
        # 有线没有 IP，尝试读取 WiFi 的 IP
        ip = os.popen("/sbin/ifconfig wlan0 | grep 'inet' | awk '{print $2}'").read()
        ip = ip[0 : ip.find('\n')]
        if(ip == ''):
            ip = 'x.x.x.x'             # 两个都没有，返回占位符
    return ip


# 协议解析：把收到的字符串指令解析成6个舵机角度，发给 Arm_Lib
# 协议格式：$20AAA BBB CCC DDD EEE FFF#
#           $  = 起始符
#           20 = 指令类型标识（固定，表示"6轴角度控制"）
#           AAA～FFF = servo 1~6 角度，各占3位，范围 000~180（或270）
#           #  = 结束符
# 例：$20090090090090090090# → 6个舵机都转到90°
def Analysis(socket, cmd):
    print(cmd)
    try:
        check = cmd[1:3]                # 提取类型字段，即 '20'
        if check == '20' and len(cmd) == 22:
            # 按固定偏移切片，每3个字符一个角度值
            s1_angle = int(cmd[3:6])    # servo 1 角度
            s2_angle = int(cmd[6:9])    # servo 2 角度
            s3_angle = int(cmd[9:12])   # servo 3 角度
            s4_angle = int(cmd[12:15])  # servo 4 角度
            s5_angle = int(cmd[15:18])  # servo 5 角度
            s6_angle = int(cmd[18:21])  # servo 6 角度（最后一位是 # 结束符）

            # 6轴同时运动，time=500ms 完成
            Arm.Arm_serial_servo_write6(s1_angle, s2_angle, s3_angle, s4_angle, s5_angle, s6_angle, 500)
        else:
            print("cmd data error! continue...")
    except:
        print("cmd data error! continue...")


# 启动 TCP server，持续监听 B_arm_ctrl 发来的指令
def start_tcp_server(ip, port):
    global g_init, g_socket
    g_init = True
    print('start_tcp_server')

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # TCP socket（有连接、可靠传输）
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) # 允许端口复用，避免重启时 "Address already in use"
    sock.bind((ip, port))              # 绑定本机 IP 和端口
    sock.listen(5)                     # 最多允许5个连接排队等待

    while True:
        conn, address = sock.accept()  # 阻塞等待客户端连接（B_arm_ctrl）
        g_socket = conn
        print("client connected:" + str(address))

        while True:
            cmd = g_socket.recv(1024).decode(encoding="utf-8")  # 接收最多 1024 bytes，解码为字符串
            if not cmd:
                print("Client Disconnected!")
                break                  # 客户端断开，退出内层循环，重新等待新连接

            # 从收到的数据中提取 $ ... # 之间的有效帧（防粘包/多余数据干扰）
            index1 = cmd.find("$")    # 找起始符位置
            index2 = cmd.find("#")    # 找结束符位置
            if index1 >= 0 and index2 > index1:
                Analysis(g_socket, cmd[index1:index2+1])  # 截取完整帧再解析


# 关闭 socket（Ctrl+C 触发时清理资源）
def waitClose():
    global g_init, g_socket
    g_init = False
    g_socket.close()


if __name__ == '__main__':
    try:
        port = 6100
        if g_init == False:
            # 等待网络就绪（IP 不是占位符才继续）
            while(True):
                ip = getLocalip()
                print("%s:%d" % (ip, port))
                if(ip == "x.x.x.x"):
                    continue           # 网络未就绪，继续等待
                if(ip != "x.x.x.x"):
                    break              # 获取到真实 IP，退出等待
        start_tcp_server(ip, port)     # 启动 TCP server，开始接收指令
    except KeyboardInterrupt:
        waitClose()                    # Ctrl+C → 关闭 socket
        print(" Program closed! ")
        pass
