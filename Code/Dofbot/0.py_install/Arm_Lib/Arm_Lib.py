#!/usr/bin/env python3
#coding: utf-8

# 通信链路：Python → UART → STM32 firmware → PWM → servo
# Command frame 格式：[0xFF, 0xFC, LEN, CMD, data..., checksum]

from time import sleep
import serial   # UART serial port library
import struct   # 解析 binary data（类似 C 的 struct）


class Arm_Device(object):

    def __init__(self, com="/dev/myserial"):
        # com: serial port 路径，Linux 下通常是 /dev/ttyUSB0
        # default parameter，不传就用 "/dev/myserial"

        # --- Protocol constants ---
        self.__HEAD       = 0xFF        # frame header sync byte，0xFF = 1111 1111，标志一帧开始
        self.__DEVICE_ID  = 0xFC        # host→STM32 方向的 device ID，0xFC = 1111 1100
        self.__COMPLEMENT = 257 - self.__DEVICE_ID  # = 5，checksum 累加的初始值

        # --- Function codes（STM32 reply 时携带，接收端靠它区分数据类型）---
        self.FUNC_UART_SERVO   = 0x0A  # single servo angle reply
        self.FUNC_UART_SUBS    = 0x0B  # subscribe data reply
        self.FUNC_UART_NUM     = 0x22  # action group count reply
        self.FUNC_UART_STATE   = 0x33  # device state reply
        self.FUNC_UART_VERSION = 0x01  # firmware version reply
        self.FUNC_UART_result  = 0x2A  # speech recognition result reply

        # --- Receive buffer（STM32 reply 解析后的值暂存在这里，等上层来取）---
        self.servo_H      = 0    # servo position high byte（16-bit pulse value 的高 8 位）
        self.servo_L      = 0    # servo position low byte（16-bit pulse value 的低 8 位）
        self.id           = 0    # servo ID from reply（用来核对回的是哪个 servo）
        self.subs_H       = 0    # subscribe data high byte
        self.subs_L       = 0    # subscribe data low byte
        self.version      = -1   # firmware version，-1 = not read yet
        self.num          = 0    # saved action frame count
        self.state        = 0    # device state
        self.speech_state = 0    # speech recognition state

        self.ser = serial.Serial(com, 115200, timeout=.2)  # 打开 serial port，baud rate 115200
        # timeout=.2 → read() 最多等 0.2 秒，超时返回空，防止永久阻塞
        sleep(.2)   # 等待 serial port hardware 稳定，类似 HAL_Delay(200)


    def __parse_data(self, ext_type, ext_data):
        # 根据 function code 把数据存到对应的 member variable
        # struct.unpack('B', bytes) → 把 1 byte 解析成 uint8_t，返回 tuple，[0] 取出值
        # ext_data[0:1] → slice 语法，取 index 0 的 1 个 byte（含头不含尾）
        if ext_type == self.FUNC_UART_SERVO:
            self.servo_H = struct.unpack('B', bytearray(ext_data[0:1]))[0]  # high byte of position
            self.servo_L = struct.unpack('B', bytearray(ext_data[1:2]))[0]  # low byte of position
            self.id      = struct.unpack('B', bytearray(ext_data[2:3]))[0]  # which servo replied
        elif ext_type == self.FUNC_UART_SUBS:
            self.subs_H  = struct.unpack('B', bytearray(ext_data[0:1]))[0]
            self.subs_L  = struct.unpack('B', bytearray(ext_data[1:2]))[0]
        elif ext_type == self.FUNC_UART_NUM:
            self.num     = struct.unpack('B', bytearray(ext_data[0:1]))[0]
        elif ext_type == self.FUNC_UART_VERSION:
            self.version = struct.unpack('B', bytearray(ext_data[0:1]))[0]
        elif ext_type == self.FUNC_UART_STATE:
            self.state   = struct.unpack('B', bytearray(ext_data[0:1]))[0]
        elif ext_type == self.FUNC_UART_result:
            self.speech_state = struct.unpack('B', bytearray(ext_data[0:1]))[0]


    def __receive_data(self):
        # Reply frame（STM32→Jetson）：[0xFF, 0xFB, LEN, TYPE, data..., checksum]
        #                               header  ID   len  func             verify
        head1 = bytearray(self.ser.read())[0]              # 读 1 byte，bytearray()[0] 转成 int
        if head1 == self.__HEAD:                            # 等到 sync byte 0xFF，否则丢弃
            head2 = bytearray(self.ser.read())[0]
            check_sum    = 0
            rx_check_num = 0
            if head2 == self.__DEVICE_ID - 1:              # reply ID = 0xFB（发送用 0xFC，reply 减 1 区分方向）
                ext_len  = bytearray(self.ser.read())[0]   # length（从自身开始数的总 byte 数）
                ext_type = bytearray(self.ser.read())[0]   # function code，决定数据是什么类型
                ext_data = []                               # 空 list，准备收 payload，类似 vector<uint8_t>
                check_sum = ext_len + ext_type             # checksum 从 LEN 和 TYPE 开始累加
                data_len  = ext_len - 2                    # 剩余待读 bytes = LEN - LEN本身(1) - TYPE(1)
            while len(ext_data) < data_len:                # len() 返回 list 当前长度
                value = bytearray(self.ser.read())[0]
                ext_data.append(value)                     # append() 向 list 末尾追加，类似 push_back()
                if len(ext_data) == data_len:
                    rx_check_num = value                   # 最后一个 byte 是 checksum，不加入累加
                else:
                    check_sum += value                     # 其余 byte 累加进 checksum
            if check_sum % 256 == rx_check_num:            # % 256 取低 8 位，与接收到的 checksum 比对
                self.__parse_data(ext_type, ext_data)      # 验证通过，解析数据
            else:
                print("check sum error:", ext_len, ext_type, ext_data)  # 传输出错，丢弃这帧


    def Arm_serial_servo_write(self, id, angle, time):
        # 控制单个 servo 转到指定角度
        # frame 结构：[0xFF, 0xFC, 0x07, 0x10+id, posH, posL, timeH, timeL, checksum]
        #              header+ID   LEN   CMD        ←pos 16bit→  ←time 16bit→
        sleep(0.01)                                        # 稳定延时，防止连续命令太快导致丢包

        if id == 0:                                        # id=0 = broadcast，所有 servo 同时动
            self.Arm_serial_servo_write6(angle, angle, angle, angle, angle, angle, time)

        elif id == 2 or id == 3 or id == 4:
            angle = 180 - angle                            # servo 2/3/4 物理安装方向相反，angle 翻转
            pos     = int((3100 - 900) * angle / 180 + 900)  # linear mapping：0°→900, 180°→3100
            value_H = (pos >> 8) & 0xFF                    # right shift 8 bits → 取 high byte
            value_L = pos & 0xFF                           # bitwise AND 0xFF mask → 取 low byte
            time_H  = (time >> 8) & 0xFF                  # time 同样拆成 high/low byte
            time_L  = time & 0xFF
            try:
                cmd = [0xFF, 0xFC, 0x07, 0x10 + id, value_H, value_L, time_H, time_L]
                # 0xFF 0xFC = frame header + device ID（固定）
                # 0x07      = length，从自身数共 7 bytes
                # 0x10+id   = command code，id=1→0x11, id=2→0x12 ... id=6→0x16
                checksum = sum(cmd, 5) & 0xff              # 所有 bytes 累加（初始值 5），& 0xff 取低 8 bits
                cmd.append(checksum)                       # checksum 追加到 frame 末尾
                self.ser.write(cmd)                        # 通过 UART 把整个 frame 发给 STM32
            except:
                print('Arm_serial_servo_write serial error')

        elif id == 5:
            pos     = int((3700 - 380) * angle / 270 + 380)  # servo 5 range 不同：0°→380, 270°→3700
            value_H = (pos >> 8) & 0xFF
            value_L = pos & 0xFF
            time_H  = (time >> 8) & 0xFF
            time_L  = time & 0xFF
            try:
                cmd = [0xFF, 0xFC, 0x07, 0x10 + id, value_H, value_L, time_H, time_L]
                checksum = sum(cmd, 5) & 0xff
                cmd.append(checksum)
                self.ser.write(cmd)
            except:
                print('Arm_serial_servo_write serial error')

        else:                                              # servo 1/6：正常方向，0°→900, 180°→3100
            pos     = int((3100 - 900) * angle / 180 + 900)
            value_H = (pos >> 8) & 0xFF
            value_L = pos & 0xFF
            time_H  = (time >> 8) & 0xFF
            time_L  = time & 0xFF
            try:
                cmd = [0xFF, 0xFC, 0x07, 0x10 + id, value_H, value_L, time_H, time_L]
                checksum = sum(cmd, 5) & 0xff
                cmd.append(checksum)
                self.ser.write(cmd)
            except:
                print('Arm_serial_servo_write serial er')


    def Arm_serial_servo_write6(self, s1, s2, s3, s4, s5, s6, time):
        # 一帧同时控制全部 6 个 servo（比逐个发送更同步）
        # frame：[0xFF, 0xFC, 0x11, 0x1d, pos1H, pos1L, ..., pos6H, pos6L, timeH, timeL, checksum]
        #         header+ID   LEN   CMD   ←─────────── 6×servo position ───────────→  time  verify
        #         0x11 = length 17（从自身数）；0x1d = command code for "sync write 6 servos"
        if s1 > 180 or s2 > 180 or s3 > 180 or s4 > 180 or s5 > 270 or s6 > 180:
            print("参数传入范围不在0-180之内！")
            return                                         # 超出范围直接退出，不发命令

        try:
            pos = int((3100 - 900) * s1 / 180 + 900)      # servo 1：正常方向，linear mapping
            value1_H = (pos >> 8) & 0xFF                   # high byte
            value1_L = pos & 0xFF                          # low byte

            s2 = 180 - s2                                  # servo 2：安装方向相反，先翻转再映射
            pos = int((3100 - 900) * s2 / 180 + 900)
            value2_H = (pos >> 8) & 0xFF
            value2_L = pos & 0xFF

            s3 = 180 - s3                                  # servo 3：同上
            pos = int((3100 - 900) * s3 / 180 + 900)
            value3_H = (pos >> 8) & 0xFF
            value3_L = pos & 0xFF

            s4 = 180 - s4                                  # servo 4：同上
            pos = int((3100 - 900) * s4 / 180 + 900)
            value4_H = (pos >> 8) & 0xFF
            value4_L = pos & 0xFF

            pos = int((3700 - 380) * s5 / 270 + 380)      # servo 5：range 270°，pulse range 380-3700
            value5_H = (pos >> 8) & 0xFF
            value5_L = pos & 0xFF

            pos = int((3100 - 900) * s6 / 180 + 900)      # servo 6：正常方向
            value6_H = (pos >> 8) & 0xFF
            value6_L = pos & 0xFF

            time_H = (time >> 8) & 0xFF
            time_L = time & 0xFF

            cmd = [0xFF, 0xFC, 0x11, 0x1d,                # list 括号内可换行，Python 自动续行
                   value1_H, value1_L, value2_H, value2_L,
                   value3_H, value3_L, value4_H, value4_L,
                   value5_H, value5_L, value6_H, value6_L,
                   time_H, time_L]
            checksum = sum(cmd, 5) & 0xff                  # 所有 bytes 累加（初始值 5），取低 8 bits 防溢出
            cmd.append(checksum)                           # checksum 追加到 frame 末尾
            self.ser.write(cmd)                            # 发出完整 19-byte frame
        except:
            print('Arm_serial_servo_write6 serial error')


    def Arm_serial_servo_write_any(self, id, angle, time):
        # 控制任意 ID 的 bus servo（ID range 1-250，不限机械臂的 6 个）
        # id=0 时 broadcast 给所有 servo
        if id != 0:
            pos     = int((3100 - 900) * angle / 180 + 900)
            value_H = (pos >> 8) & 0xFF
            value_L = pos & 0xFF
            time_H  = (time >> 8) & 0xFF
            time_L  = time & 0xFF
            try:
                cmd = [0xFF, 0xFC, 0x08, 0x19, id & 0xFF, value_H, value_L, time_H, time_L]
                # 0x19 = write single servo by ID，frame 中额外带一个 id byte
                checksum = sum(cmd, 5) & 0xff
                cmd.append(checksum)
                self.ser.write(cmd)
            except:
                print('Arm_serial_servo_write_any serial error')
        elif id == 0:
            pos     = int((3100 - 900) * angle / 180 + 900)
            value_H = (pos >> 8) & 0xFF
            value_L = pos & 0xFF
            time_H  = (time >> 8) & 0xFF
            time_L  = time & 0xFF
            try:
                cmd = [0xFF, 0xFC, 0x07, 0x17, id & 0xFF, value_H, value_L, time_H, time_L]
                # 0x17 = broadcast write，所有 servo 同时响应
                checksum = sum(cmd, 5) & 0xff
                cmd.append(checksum)
                self.ser.write(cmd)
            except:
                print('Arm_serial_servo_write_any serial error')


    def Arm_serial_servo_write_offset_switch(self, id):
        # 手动把 servo 掰到中位后调用此函数，STM32 记录当前位置为零点 offset
        # id=1~6 设置对应 servo，id=0 恢复出厂 offset
        try:
            if id > 0 and id < 7:
                cmd = [0xFF, 0xFC, 0x04, 0x1c, id]        # 0x1c = set offset，data = servo id
            elif id == 0:
                cmd = [0xFF, 0xFC, 0x04, 0x1c, 0x00]      # 0x00 = reset all offsets
            checksum = sum(cmd, 5) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
        except:
            print('Arm_serial_servo_write_offset_switch serial error')


    def Arm_serial_servo_write_offset_state(self):
        # 查询 offset 操作是否成功。返回：0=找不到ID，1=成功，2=超出range
        try:
            cmd = [0xFF, 0xFC, 0x03, 0x1b]                # 0x1b = query offset state，无 data byte
            checksum = sum(cmd, 5) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            self.__receive_data()                          # 阻塞等待 STM32 reply，结果存入 self.state
            sleep(0.002)                                   # 给 STM32 处理时间
            return self.state                              # 返回 state，调用方检查结果
        except:
            print('Arm_serial_servo_write_offset_state serial error')
        return None                                        # 出错返回 None，调用方需判断


    def Arm_serial_servo_write6_array(self, joints, time): #例如为设计好姿态使用
        # write6 的 array 版本，joints = [s1, s2, s3, s4, s5, s6]
        # 与 write6 功能相同，只是 interface 不同（list 输入 vs 6 个独立参数）
        s1, s2, s3, s4, s5, s6 = joints[0], joints[1], joints[2], joints[3], joints[4], joints[5]
        # 上面是 Python 多重赋值（tuple unpacking），一行拆出 list 中多个值
        if s1 > 180 or s2 > 180 or s3 > 180 or s4 > 180 or s5 > 270 or s6 > 180:
            print("参数传入范围不在0-180之内！The parameter input range is not within 0-180!")
            return
        try:
            pos = int((3100 - 900) * s1 / 180 + 900)      # servo 1：正常方向
            value1_H = (pos >> 8) & 0xFF
            value1_L = pos & 0xFF

            s2 = 180 - s2                                  # servo 2：安装方向相反，翻转
            pos = int((3100 - 900) * s2 / 180 + 900)
            value2_H = (pos >> 8) & 0xFF
            value2_L = pos & 0xFF

            s3 = 180 - s3                                  # servo 3：同上
            pos = int((3100 - 900) * s3 / 180 + 900)
            value3_H = (pos >> 8) & 0xFF
            value3_L = pos & 0xFF

            s4 = 180 - s4                                  # servo 4：同上
            pos = int((3100 - 900) * s4 / 180 + 900)
            value4_H = (pos >> 8) & 0xFF
            value4_L = pos & 0xFF

            pos = int((3700 - 380) * s5 / 270 + 380)      # servo 5：range 270°
            value5_H = (pos >> 8) & 0xFF
            value5_L = pos & 0xFF

            pos = int((3100 - 900) * s6 / 180 + 900)      # servo 6：正常方向
            value6_H = (pos >> 8) & 0xFF
            value6_L = pos & 0xFF

            time_H = (time >> 8) & 0xFF
            time_L = time & 0xFF

            cmd = [0xFF, 0xFC, 0x11, 0x1e,                # 0x1e = array mode sync write（与 0x1d 区别在 firmware）
                   value1_H, value1_L, value2_H, value2_L,
                   value3_H, value3_L, value4_H, value4_L,
                   value5_H, value5_L, value6_H, value6_L,
                   time_H, time_L]
            checksum = sum(cmd, 5) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
        except:
            print('Arm_serial_servo_write6 serial error')


    def Arm_RGB_set(self, red, green, blue):
        # 设置 RGB LED 颜色，red/green/blue 各 0-255
        try:
            cmd = [self.__HEAD, self.__DEVICE_ID, 0x06, 0x02,
                   red & 0xff, green & 0xff, blue & 0xff]  # & 0xff 确保值在 0-255 范围内
            checksum = sum(cmd, self.__COMPLEMENT) & 0xff  # RGB frame 用 __COMPLEMENT(=5) 作初始值
            cmd.append(checksum)
            self.ser.write(cmd)
        except:
            print('Arm_RGB_set serial error')


    def Arm_Buzzer_On(self, delay=0xff):
        # 打开 buzzer，delay=0xff = 持续响；delay=其他值由 firmware 决定时长
        # default parameter：不传 delay 时默认持续响
        try:
            if delay != 0:                                 # delay=0 等同于关闭，这里跳过
                cmd = [0xFF, 0xFC, 0x04, 0x06, delay & 0xff]  # 0x06 = buzzer command
                checksum = sum(cmd, 5) & 0xff
                cmd.append(checksum)
                self.ser.write(cmd)
        except:
            print('Arm_Buzzer_On serial error')


    def Arm_Buzzer_Off(self):
        # 关闭 buzzer，发 delay=0x00 给 STM32
        try:
            cmd = [0xFF, 0xFC, 0x04, 0x06, 0x00]          # 0x00 = turn off
            checksum = sum(cmd, 5) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
        except:
            print('Arm_Buzzer_On serial error')


    def Arm_serial_servo_read(self, id):
        # 读取 servo 当前角度（度）
        # 流程：send request → receive reply → parse 16-bit pulse → reverse mapping → angle
        if id < 1 or id > 6:
            print("id must be 1 - 6")
            return None                                    # 提前退出，None = C++ 的 nullptr
        try:
            cmd = [0xFF, 0xFC, 0x03, id + 0x30]           # 0x30+id = read servo command（id=1→0x31 ...）
            checksum = sum(cmd, 5) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            sleep(0.001)                                   # 等 STM32 处理
            self.__receive_data()                          # 接收并解析 reply，结果存入 servo_H/L/id

            cmd = [0xFF, 0xFC, 0x03, id + 0x30]           # 发两次提高可靠性（防单包丢失）
            checksum = sum(cmd, 5) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            sleep(0.001)
            self.__receive_data()

            i = 0
            d = 0
            if self.id == (id + 0x30):                    # 核对 reply 是否对应我们 request 的 servo
                d = self.servo_H * 256 + self.servo_L     # 将2个byte拼回 16-bit pulse value：high×256 + low
            else:
                self.__receive_data()                      # 没收到就再试一次
                i += 1                                     # i += 1 等价于 i++（Python 没有 ++）
            if i > 3:
                i = 0
                return None                               # 重试超过 3 次，放弃
        except:
            print('Arm_serial_servo_read serial error')
            return None

        if d == 0:
            return None                                    # pulse value 为 0，说明读取失败

        # pulse value 反算 angle（写入公式的逆运算）
        if id == 5:
            d = int((270 - 0) * (d - 380) / (3700 - 380)) # pulse(380-3700) → angle(0-270)
            if d > 270 or d < 0: return None              # 超出合理范围，丢弃
        else:
            d = int((180 - 0) * (d - 900) / (3100 - 900)) # pulse(900-3100) → angle(0-180)
            if d > 180 or d < 0: return None

        if id == 2 or id == 3 or id == 4:
            d = 180 - d                                    # 安装方向相反，读回后再翻转还原
        return d


    def Arm_Button_Mode(self, mode):
        # 设置 K1 按键的工作模式
        # mode: 0=default，1=learning mode（进入后可录制动作）
        try:
            cmd = [0xFF, 0xFC, 0x04, 0x03, mode & 0xff]   # 0x03 = button mode command
            checksum = sum(cmd, 5) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
        except:
            print('Arm_Button_Mode serial error')


    def Arm_Action_Study(self):
        # learning mode 下，把当前所有 servo 位置记录为一帧动作
        # 需要先调用 Arm_Button_Mode(1) 进入 learning mode
        try:
            cmd = [0xFF, 0xFC, 0x04, 0x24, 0x01]          # 0x24 = record action，0x01 = do it
            checksum = sum(cmd, 5) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
        except:
            print('Arm_Action_Study serial error')


    def Arm_Read_Action_Num(self):
        # 读取 Flash 中已保存的 action frame 数量，返回 int
        try:
            cmd = [0xFF, 0xFC, 0x04, 0x22, 0x01]          # 0x22 = query action count
            checksum = sum(cmd, 5) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            sleep(0.001)
            self.__receive_data()                          # reply 解析后存入 self.num
            return self.num
        except:
            print('Arm_Read_Action_Num serial error')


    def Arm_Action_Mode(self, mode):
        # 控制已录制动作组的播放
        # mode: 0=stop，1=play once，2=loop
        try:
            cmd = [0xFF, 0xFC, 0x04, 0x20, mode & 0xff]   # 0x20 = action play mode command
            checksum = sum(cmd, 5) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
        except:
            print('Arm_Clear_Action serial error')


    def Arm_Clear_Action(self):
        # 清空 Flash 中所有已录制的 action frames
        try:
            cmd = [0xFF, 0xFC, 0x04, 0x23, 0x01]          # 0x23 = clear action command
            checksum = sum(cmd, 5) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            sleep(.5)                                      # Flash erase 较慢，需等 500ms
        except:
            print('Arm_Clear_Action serial error')


    def get_version():  # BUG: 缺少 self parameter，调用会报错
        try:
            cmd = [0xFF, 0xFC, 0x03, 0x01]                # 0x01 = get firmware version
            checksum = sum(cmd, 5) & 0xff
            cmd.append(checksum)
            ser.write(cmd)          # BUG: 应为 self.ser.write(cmd)
            self.__receive_data()   # BUG: self 未定义（缺少 self parameter）
            sleep(0.002)
            return self.version     # BUG: 同上
        except:
            print('version error')
            return None


    def Arm_serial_servo_read_any(self, id):
        # 读取任意 ID bus servo 的当前角度（ID range 1-250，不限机械臂的 6 个）
        if id < 1 or id > 250:
            print("id must be 1 - 250")
            return None
        try:
            cmd = [0xFF, 0xFC, 0x04, 0x37, id]            # 0x37 = read any servo by ID
            checksum = sum(cmd, 5) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            sleep(.001)
            self.__receive_data()                          # reply 解析后存入 servo_H / servo_L
            sleep(0.002)
            d = self.servo_H * 256 + self.servo_L          # 拼回 16-bit pulse value
        except:
            print('version error')
            return None

        d = (d >> 8 & 0xff) | (d << 8 & 0xff00)           # byte order swap（该协议的特殊处理）
        d = int((180 - 0) * (d - 900) / (3100 - 900))     # pulse → angle 逆映射
        return d


    def Arm_ping_servo(self, id):
        # 检测 servo 是否在线。返回：0xda=正常，0x00=无响应，其他=错误
        # BUG：value 始终被重置为 0（应在循环内 receive reply），且 time 应为 times
        data = int(id)                                     # int() cast，确保是整数
        if data > 0 and data <= 250:
            reg = 0x38                                     # 0x38 = ping command
            cmd = [0xFF, 0xFC, 0x04, reg, data]
            checksum = sum(cmd, 5) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            sleep(.001)
            value = 0
            times = 0
            while value == 0 and times < 5:               # 最多 retry 5 次
                cmd = [0xFF, 0xFC, 0x04, reg, data]
                checksum = sum(cmd, 5) & 0xff
                cmd.append(checksum)
                self.ser.write(cmd)
                sleep(.001)
                value = 0                                  # BUG：应在这里 receive reply 赋值给 value
                times += 1                                 # times++ （Python 写法：+=1）
                if time >= 5:                              # BUG：time 是 built-in module，应为 times
                    return None
            return value
        else:
            return None


    def Arm_serial_set_torque(self, onoff):
        # Torque 开关
        # onoff=1：enable torque，servo 锁定保持位置
        # onoff=0：disable torque，可手动掰动，用于示教或调试
        try:
            cmd = [0xFF, 0xFC, 0x04, 0x1A, 0x01 if onoff == 1 else 0x00]
            # 0x01 if onoff == 1 else 0x00 → Python 三元运算符，等价于 C++ 的 onoff==1 ? 0x01 : 0x00
            checksum = sum(cmd, 5) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            sleep(.001)
        except:
            print('Arm_serial_set_torque serial error')


    def Arm_serial_set_id(self, id):
        # 修改 bus servo 的 ID 编号
        # 注意：改后原 ID 立即失效，需断电重连才能用新 ID 通信
        try:
            cmd = [0xFF, 0xFC, 0x04, 0x18, id & 0xFF]     # 0x18 = set ID command
            checksum = sum(cmd, 5) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            sleep(.001)
        except:
            print('Arm_serial_set_id serial error')


    def Arm_Product_Select(self, index):
        # 选择产品颜色方案（index: 1-6），RGB LED 亮对应颜色，用于区分不同型号
        try:
            cmd = [0xFF, 0xFC, 0x04, 0x04, index & 0xFF]  # 0x04 = product select command
            checksum = sum(cmd, 5) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            sleep(.001)
        except:
            print('Arm_Product_Select serial erro')


    def Arm_reset(self):
        # 重启 STM32 control board（software reset）
        try:
            cmd = [0xFF, 0xFC, 0x04, 0x05, 0x01]          # 0x05 = reset command，0x01 = execute
            checksum = sum(cmd, 5) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            sleep(.001)
        except:
            print('Arm_reset serial error')


    def Arm_PWM_servo_write(self, id, angle):
        # 控制 PWM servo（与 bus servo 不同：无 feedback，不能读取当前角度）
        # id=0 broadcast 所有 PWM servo，id=1-6 控制单个
        try:
            if id == 0:
                cmd = [0xFF, 0xFC, 0x04, 0x57, angle & 0xff]       # 0x57 = broadcast all PWM servos
            else:
                cmd = [0xFF, 0xFC, 0x04, 0x50 + id, angle & 0xff]  # 0x51~0x56 各对应一个 PWM servo
            checksum = sum(cmd, 5) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            sleep(.001)
        except:
            print('Arm_PWM_servo_write serial error')


    def Arm_voied_write(self):
        # 向 STM32 发出查询，触发 speech recognition module 状态上报
        try:
            cmd = [0xFF, 0xFC, 0x03, 0x2A]                # 0x2A = query speech state，无 data byte
            checksum = sum(cmd, 5) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            sleep(.001)
        except:
            print('Arm_PWM_servo_write serial error')


    def Arm_serial_speech_read(self, id):
        # 从 serial buffer 读取 speech recognition 结果
        # 注意：parameter id 传入但未使用（原代码遗留，不影响功能）
        try:
            self.__receive_data()                          # 阻塞等待 reply，超时由 serial timeout(0.2s) 控制
            sleep(0.001)
            return self.speech_state                       # 返回上一次解析到的 speech state
        except:
            return 0                                       # 出错返回 0，表示无识别结果


    def Arm_ask_speech(self, id):
        # 向 speech module 发 query request
        # 0x60+id 对应不同 query type（由 firmware 定义）
        try:
            cmd = [0xFF, 0xFC, 0x03, 0x60 + id]
            checksum = sum(cmd, 5) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            sleep(.001)
        except:
            print("iic error")
