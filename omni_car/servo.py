from machine import PWM, Pin  # type: ignore
import time


class Servo:
    def __init__(
        self,
        pin,                    # PWM 引脚号
        freq      = 50,         # PWM 频率
        min_us    = 500,        # 舵机最小脉宽
        max_us    = 2500,       # 舵机最大脉宽         
        max_angle = 180,        # 舵机可达最大角度
        min_accu  = 0.3,        # 舵机最小精度

        target_angle    = 90,   # 初始化目标角度
        limit_min_angle = 0,    # 最小角度限制
        limit_max_angle = 180   # 最大角度限制
    ):
        self.pin = pin
        self.pwm = PWM(Pin(pin), freq=freq, duty=0)

        self.freq      = freq      # 频率
        self.min_us    = min_us    # 最小脉宽
        self.max_us    = max_us    # 最大脉宽
        self.max_angle = max_angle # 最大角度
        self.min_accu  = min_accu  # 最小精度

        self.limit_max_angle = limit_max_angle  # 最大角度限制
        self.limit_min_angle = limit_min_angle  # 最小角度限制

        self.target_angle = target_angle  # 初始化目标角度
        self.set_angle(target_angle)

    def set_limit(self, limit_min_angle, limit_max_angle):  # 设置角度限制
        self.limit_max_angle = limit_max_angle
        self.limit_min_angle = limit_min_angle

    def set_angle(self, target_angle):  # 绝对角度运动

        # print(f"set_angle(): 传入 {self.pin} 号舵机的目标角度: {target_angle}")

        target_angle = min(max(target_angle, self.limit_min_angle), self.limit_max_angle) # 限制角度

        print(f"set_angle(): 实际 {self.pin} 号舵机可达角度: {target_angle}\n")

        self.target_angle = target_angle

        us = self.min_us + (self.max_us - self.min_us) * (target_angle / self.max_angle)
        ns = int(us * 1000)

        self.pwm.duty_ns(ns)

    def set_angle_relative(self, relative_angle):  # 相对角度运动
        print(f"set_angle_relative(): 传入 {self.pin} 号舵机的相对角度: {relative_angle}")
        self.target_angle += relative_angle
        self.set_angle(self.target_angle)

    def set_step(self, step=1):  # 以最小精度步进N步, 默认步进1步
        print(f"set_step(): 传入 {self.pin} 号舵机步进: {step} 步")
        self.set_angle_relative(self.min_accu * step)
        
    def reset(self):  # 复位
        print("reset(): 复位舵机")
        self.set_angle( self.target_angle)


if __name__ == "__main__":
    servo_y = Servo(5)
    servo_y.set_angle(90)
    time.sleep(1)