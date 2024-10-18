from machine import SoftI2C, Pin, PWM

class RobotController:
    def __init__(self):
        self.PWM_LIMIT = 1023
        self.init_motors()

    def init_motors(self):
        # 初始化速度控制引脚对象
        self.motor_speed_1 = PWM(Pin(15), freq=50)
        self.motor_speed_2 = PWM(Pin(16), freq=50)
        self.motor_speed_3 = PWM(Pin(17), freq=50)

        # 初始化方向控制引脚对象
        self.motor_dir_1 = Pin(14, Pin.OUT)
        self.motor_dir_2 = Pin(13, Pin.OUT)
        self.motor_dir_3 = Pin(12, Pin.OUT)

    def set_motor(self, motor_dir, motor_speed, pwm_val):
        if pwm_val > 0:
            motor_dir.value(0)
            motor_speed.duty(abs(pwm_val))
        elif pwm_val < 0:
            motor_dir.value(1)
            motor_speed.duty(abs(pwm_val))
        else:
            motor_speed.duty(0)
            motor_dir.value(0)

    def set_motor_1(self, pwm_val):
        self.set_motor(self.motor_dir_1, self.motor_speed_1, pwm_val)

    def set_motor_2(self, pwm_val):
        self.set_motor(self.motor_dir_2, self.motor_speed_2, pwm_val)

    def set_motor_3(self, pwm_val):
        self.set_motor(self.motor_dir_3, self.motor_speed_3, pwm_val)

    def move(self, v_y, v_x, v_w):
        v_y = min(max(v_y, -self.PWM_LIMIT), self.PWM_LIMIT)
        v_x = min(max(v_x, -self.PWM_LIMIT), self.PWM_LIMIT)
        v_w = min(max(v_w, -self.PWM_LIMIT), self.PWM_LIMIT)

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
    
