"""
    Omni Bot 全向轮 运动控制模块
    by: DBin_K
"""

import time
import math
from machine import Pin, PWM  # type: ignore


class Encoder:
    def __init__(self, a_pin, b_pin):
        """
        初始化编码器对象
        @param a_pin: 编码器A相位输入引脚
        @param b_pin: 编码器B相位输入引脚
        """
        self.encoder_a = Pin(a_pin, Pin.IN, Pin.PULL_UP)  # 编码器A相位引脚
        self.encoder_b = Pin(b_pin, Pin.IN, Pin.PULL_UP)  # 编码器B相位引脚

        # 初始化编码器脉冲计数和速度
        self.motor_dir_flag = 0  # 电机当前方向标志位，用于计算速度
        self.last_time = None  # 初始化上次脉冲的时间 time_diff()要用到
        self.speed = 0  # 速度

        # 设置编码器引脚的中断
        self.encoder_a.irq(trigger=Pin.IRQ_RISING, handler=self.encoder_callback)

    def time_diff(self):
        current_time = time.ticks_us()  # 获取当前时间（单位：微秒）

        if self.last_time is None:  # 如果是第一次调用
            self.last_time = current_time
            return 0.000_001  # 防止除零错误

        else:  # 计算时间差
            diff = time.ticks_diff(current_time, self.last_time)  # 计算时间差
            self.last_time = current_time  # 更新上次调用时间
            return diff  # 返回时间差us

    def encoder_callback(self, pin):
        # 编码器脉冲计数
        dt = self.time_diff()  # 计算上一次循环和这一次的时间差

        if self.encoder_b.value():  # 如果B相位为高，表示A相位上升沿
            self.motor_dir_flag = 1
        else:  # 否则，表示A相位下降沿
            self.motor_dir_flag = -1

        # 计算速度（脉冲数/时间差）
        if dt > 0:  # 防止除以零
            self.speed = self.motor_dir_flag / (dt / 1_000_000)  # 转换为脉冲/秒

    def get_speed(self):
        # 获取当前速度
        return self.speed


class Motor:
    def __init__(self, speed_pin, dir_pin, PWM_LIMIT=(200, 1000)):
        """
        初始化电机对象
        @param speed_pin: 电机速度控制引脚
        @param dir_pin: 电机方向控制引脚
        @param PWM_LIMIT: PWM输出的上下限，默认为(100, 800)
        """
        # 初始化电机控制对象
        self.PWM_LIMIT = PWM_LIMIT
        self.motor_dir = Pin(dir_pin, Pin.OUT)            # 方向控制引脚
        self.motor_speed = PWM(Pin(speed_pin), freq=500, duty=0)   # 速度控制引脚

    def limit_value(self, value, min_value, max_value):
        """限制输入的值在给定的范围内。"""
        return min(max(value, min_value), max_value)
    
    def set_speed(self, rate):
        """
        设置电机的速度
        @param rate: 速度百分比，范围[-100, 100]
        """
        min_val = self.PWM_LIMIT[0]
        max_val = self.PWM_LIMIT[1]
        
        pwm_val = max_val - int(rate/100 * (max_val - min_val)) 
        
        print(f"pwm_val: {pwm_val}")

        # 设置电机的速度和方向
        if pwm_val > 0:
            self.motor_dir.value(1)  # 正转
            self.motor_speed.duty(abs(pwm_val))  # 设置速度
        elif pwm_val < 0:
            self.motor_dir.value(0)  # 反转
            self.motor_speed.duty(abs(pwm_val))  # 设置速度
        else:
            self.motor_dir.value(0)  # 随便设置一个方向
            self.motor_speed.duty(self.PWM_LIMIT[0])  # 停止电机


class RobotController:
    def __init__(self):
        self.wheel_distance = 10 # 轮子距离机器人中心的距离, 作为旋转速度系数
        self.motor_l = Motor(speed_pin=2, dir_pin=3)  # 左电机
        self.motor_r = Motor(speed_pin=6, dir_pin=9)  # 右电机
        self.motor_b = Motor(speed_pin=11, dir_pin=12)  # 后电机

    def move(self, v_y, v_x, v_w):
        """
        移动机器人，参数为速度控制值 
        """
        # 逆运动学解算, 计算每个电机的速度
        R = self.wheel_distance
        Vl = -v_y + R * v_w
        Vr = (
              v_x * math.cos(math.radians(30))
            + v_y * math.sin(math.radians(30))
            + R * v_w
        )
        Vb = (
             -v_x * math.cos(math.radians(30))
            + v_y * math.sin(math.radians(30))
            + R * v_w
        )

        # 设置电机速度
        self.motor_l.set_speed(Vl)
        self.motor_r.set_speed(Vr)
        self.motor_b.set_speed(Vb)

    def go_forward(self, pwm_val):
        self.move(pwm_val, 0, 0)

    def go_backward(self, pwm_val):
        self.move(-pwm_val, 0, 0)

    def go_left(self, pwm_val):
        self.move(0, -pwm_val, 0)

    def go_right(self, pwm_val):
        self.move(0, pwm_val, 0)

    def turn_left(self, pwm_val):
        self.move(0, 0, pwm_val)

    def turn_right(self, pwm_val):
        self.move(0, 0, -pwm_val)

    def stop(self):
        self.move(0, 0, 0)

    def motor_l_test(self, pwm_val):
        self.motor_l.set_speed(pwm_val)
    
    def motor_r_test(self, pwm_val):
        self.motor_r.set_speed(pwm_val)

    def motor_b_test(self, pwm_val):
        self.motor_b.set_speed(pwm_val)


if __name__ == "__main__":
    # 测试所有功能
    robot = RobotController()

    robot.motor_l_test(100)
    robot.motor_r_test(100)
    robot.motor_b_test(100)
    
    # robot.move(0,500,0)

    # while True:
    #     robot.go_forward(50)
    #     robot.go_backward(50)
    #     robot.go_left(50)
    #     robot.go_right(50)
    #     robot.turn_left(50)
    #     robot.turn_right(50)
    #     robot.stop()
