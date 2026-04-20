#!/usr/bin/env python
# coding: utf-8
import PID
import Arm_Lib


class color_follow_move:
    def __init__(self):
        self.sbus = Arm_Lib.Arm_Device()
        self.cur_anglex = 90  # 设置x方向的1号舵机默认角度
        self.cur_angley3 = 0  # 设置y方向的3号舵机默认角度
        self.cur_angley4 = 0  # 设置y方向的4号舵机默认角度
        self.offset_dead = 0.1  # 设置偏移量的死区参数(可调节)
        self.move_time = 200  # 设置舵机移动时间参数(可调节)
        self.num = 0

    def read_joint(self):
        '''
        循环读取舵机的当前角度
        '''
        cur_joint = []
        num = 0
        for i in range(1, 7):
            while 1:
                # 读取舵机角度
                joint = self.sbus.Arm_serial_servo_read(i)
                # 每十次读取不到数据,便打印警告
                if num % 10 == 0: print("请检查!!!!!接触不良!!!!!")
                # 每当读取到数据时,跳出循环,返回结果
                if joint != None: break
                num += 1
            cur_joint.append(joint)
        # 更新舵机为当前角度
        self.cur_anglex = cur_joint[0]
        self.cur_angley3 = cur_joint[2]
        self.cur_angley4 = cur_joint[3]
        # print("当前关节角度 : {}".format(cur_joint))
        del cur_joint

    def move(self, pos1, pos2):
        xservo_pid = PID.PositionalPID(0.19, 0.01, 0.01)
        yservo_pid = PID.PositionalPID(0.19, 0.01, 0.01)
        # 读取并更新当前角度值
        self.read_joint()
        # 计算X方向与y方向的偏移量
        xservo_pid.SystemOutput = pos1
        xservo_pid.SetStepSignal(320)
        xservo_pid.SetInertiaTime(0.02, 0.006)
        target_valuex = self.cur_anglex + xservo_pid.SystemOutput
        yservo_pid.SystemOutput = pos2
        yservo_pid.SetStepSignal(240)
        yservo_pid.SetInertiaTime(0.02, 0.006)
        target_valuey = (yservo_pid.SystemOutput) / 2
        last_angley3 = self.cur_angley3 + target_valuey
        last_angley4 = self.cur_angley4 + target_valuey
        if self.num % 2 == 0:
            print("中心坐标 : ", (pos1, pos2))
            print("偏移jiaodu: ", target_valuex)
            print("---------------------------")
        if target_valuex < 0: target_valuex = 0
        if target_valuex > 180: target_valuex = 180
        if last_angley3 < 0: last_angley3 = 0
        if last_angley3 > 180: last_angley3 = 180
        if last_angley4 < 0: last_angley4 = 0
        if last_angley4 > 180: last_angley4 = 180
        self.sbus.Arm_serial_servo_write(1, target_valuex, self.move_time)
        self.sbus.Arm_serial_servo_write(3, last_angley3, self.move_time)
        self.sbus.Arm_serial_servo_write(4, last_angley4, self.move_time)
        self.num += 1
