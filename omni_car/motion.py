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

        # print(f"编码器A相位引脚状态: {self.encoder_a.value()}")

        # 计算速度（脉冲数/时间差）
        if dt > 0:  # 防止除以零
            self.speed = self.motor_dir_flag / (dt / 1_000_000)  # 转换为脉冲/秒
            print(f"针脚 {pin} 速度: {self.speed}")

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
    
    def map_value(self, value, original_block, target_block):
        """将给定的值映射到给定的目标范围。"""
        original_min, original_max = original_block
        target_min, target_max     = target_block
        # 计算映射后的值
        mapped_value = target_max + (value - original_min) * (target_min - target_max) / (original_max - original_min)

        return mapped_value
    
    def set_speed(self, rate):
        """
        设置电机的速度
        @param rate: 速度百分比，范围[-100, 100]
        """
        if rate > 0:
            pwm_val = int(self.map_value( rate, (0, 100), self.PWM_LIMIT))
            pwm_val = self.limit_value(pwm_val, *self.PWM_LIMIT)  # 限制值
            self.motor_dir.value(1)         # 正转
            self.motor_speed.duty(pwm_val)  # 设置速度

        elif rate < 0:
            pwm_val = int(self.map_value(-rate, (0, 100), self.PWM_LIMIT))
            pwm_val = self.limit_value(pwm_val, *self.PWM_LIMIT)  # 限制值
            self.motor_dir.value(0)         # 反转
            self.motor_speed.duty(pwm_val)  # 设置速度

        else:
            self.motor_dir.value(0)   # 随便设置一个方向
            self.motor_speed.duty(0)  # 停止电机


class RobotController:
    def __init__(self):
        self.motor_l = Motor(speed_pin=2, dir_pin=3)  # 左电机
        self.motor_r = Motor(speed_pin=6, dir_pin=9)  # 右电机
        self.motor_b = Motor(speed_pin=11, dir_pin=12)  # 后电机

    def limit_speed(self, Vl, Vr, Vb):
        """
        限制速度，确保每个输入电机的速度值不超过100
        """
        max_speed = 100  # 检查是否有速度超过最大值
        
        if abs(Vl) > max_speed or abs(Vr) > max_speed or abs(Vb) > max_speed:
            
            max_current_speed = max(abs(Vl), abs(Vr), abs(Vb))  # 计算当前速度的最大绝对值
            scale = max_speed / max_current_speed  # 计算缩放因子
            Vl *= scale  # 等比例缩小速度
            Vr *= scale
            Vb *= scale
            
        return Vl, Vr, Vb  # 返回处理后的速度值

    def move(self, v_y, v_x, v_w):
        """
        移动机器人，参数为速度控制值 , 实验得到: v_x, v_y, v_w 速度数值调整区间约为[-58, 58]
        """
        SQRT3 = 1.732051  # 根号3
        R =  1.7  # 轮子距离中心的距离, 作为旋转速度系数, 实验得1.7时, 和v_x, v_y, v_w 速度数值调整区间接近

        # 逆运动学解算, 计算每个电机的速度
        Vl = -(v_x/2) - (v_y*SQRT3) + (v_w*R)  # 计算左轮速度
        Vr = -(v_x/2) + (v_y*SQRT3) + (v_w*R)  # 计算右轮速度
        Vb = v_x + R*v_w                       # 计算后轮速度
        print(f"原始输入 Vl:{Vl}, Vr:{Vr}, Vb:{Vb}")

        Vl, Vr, Vb = self.limit_speed(Vl, Vr, Vb)
        print(f"缩放处理后 Vl:{Vl}, Vr:{Vr}, Vb:{Vb}")

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

    encoder_l = Encoder(4, 1)
    encoder_r = Encoder(7, 5)
    encoder_b = Encoder(10, 8)

    # while True:
    #     print(f"左轮速度:{encoder_l.get_speed()}")
    #     print(f"右轮速度:{encoder_r.get_speed()}")
    #     print(f"后轮速度:{encoder_b.get_speed()}\n")
    #     time.sleep(0.1)


    # robot.motor_l_test(100)
    # robot.motor_r_test(100)
    # robot.motor_b_test(100)
    
    # robot.move(0,0,1)

    # while True:
    #     robot.go_forward(50)
    #     robot.go_backward(50)
    #     robot.go_left(50)
    #     robot.go_right(50)
    #     robot.turn_left(50)
    #     robot.turn_right(50)
    #     robot.stop()
