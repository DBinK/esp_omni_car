"""
    Omni Bot 全向轮 运动控制模块
    by: DBin_K
"""
import math
from machine import SoftI2C, Pin, PWM

class Motor:
    def __init__(self, speed_pin, dir_pin):
        # 初始化电机控制对象
        self.motor_speed = PWM(Pin(speed_pin), freq=50)  # 速度控制引脚
        self.motor_dir = Pin(dir_pin, Pin.OUT)           # 方向控制引脚

    def set_speed(self, pwm_val):
        # 设置电机的速度和方向
        if pwm_val > 0:
            self.motor_dir.value(0)  # 正转
            self.motor_speed.duty(abs(pwm_val))  # 设置速度
        elif pwm_val < 0:
            self.motor_dir.value(1)  # 反转
            self.motor_speed.duty(abs(pwm_val))  # 设置速度
        else: 
            self.motor_speed.duty(0)  # 停止电机
            self.motor_dir.value(0)  # 确保方向值为0

class RobotController:
    def __init__(self):
        self.PWM_LIMIT = 1023  # PWM限幅
        self.wheel_distance    # 轮子距离机器人中心的距离, 作为旋转速度系数
        self.motor_l = Motor(speed_pin=15, dir_pin=14)  # 左电机
        self.motor_r = Motor(speed_pin=16, dir_pin=13)  # 右电机
        self.motor_b = Motor(speed_pin=17, dir_pin=12)  # 后电机

    def limit_value(self, value, min_value=-3000, max_value=3000):
        """限制输入的值在给定的范围内。"""
        return min(max(value, min_value), max_value)

    def move(self, v_y, v_x, v_w):
        # 限制速度在PWM范围内
        v_y = min(max(v_y, -self.PWM_LIMIT), self.PWM_LIMIT)
        v_x = min(max(v_x, -self.PWM_LIMIT), self.PWM_LIMIT)
        v_w = min(max(v_w, -self.PWM_LIMIT), self.PWM_LIMIT)

        # 计算每个轮子的速度
        R = self.wheel_distance  # 轮子距离机器人中心的距离

        Va = -v_y + R * v_w
        Vb =  v_x * math.cos(math.radians(30)) + v_y * math.sin(math.radians(30)) + R * v_w
        Vc = -v_x * math.cos(math.radians(30)) + v_y * math.sin(math.radians(30)) + R * v_w

        # 设置电机速度
        self.motor_l.set_speed(Va)
        self.motor_r.set_speed(Vb)
        self.motor_b.set_speed(Vc)

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

if __name__ == "__main__":
    # 测试所有功能
    robot = RobotController()
    while True:
        robot.go_forward(50)
        robot.go_backward(50)
        robot.go_left(50)
        robot.go_right(50)
        robot.turn_left(50)
        robot.turn_right(50)
        robot.stop()