"""
    Omni Bot 全向轮 控制程序
    by: DBin_K
"""

import struct
import time
import json
import asyncio
import espnow
import network
from machine import UART, Pin

from motion import RobotController

time.sleep(1)  # 防止点停止按钮后马上再启动导致 Thonny 连接不上

# 初始化 Omni Bot
robot = RobotController()

# 初始化 LED
led = Pin(15, Pin.OUT, value=1)

# 创建串口对象
uart = UART(1, 115200, rx=34, tx=36)  # 设置串口号1和波特率
uart.init(bits=8, parity=None, stop=1)
uart.write("Hello Omni Bot!")  # 发送一条数据

# 初始化 WiFi 和 espnow
sta = network.WLAN(network.STA_IF)
sta.active(True)
sta.disconnect()  # 因为 ESP8266 会自动连接到最后一个接入点

now = espnow.ESPNow()
now.active(True)       # 连接dk广播地址
now.add_peer(b'\xff\xff\xff\xff\xff\xff')

sw = True
def stop_btn_callback(pin):
    global sw
    time.sleep(0.1)
    if pin.value() == 0:
        sw = not sw
        led.value(not led.value())
        print("停止定时器")  # 不然Thonny无法停止程序

stop_btn = Pin(0, Pin.IN, Pin.PULL_UP)
stop_btn.irq(stop_btn_callback, Pin.IRQ_FALLING)

def limit_value(value, min_value=-3000, max_value=3000):
    """限制输入的值在给定的范围内。"""
    return min(max(value, min_value), max_value)

def time_diff(last_time=[None]):
    """计算两次调用之间的时间差，单位为微秒。"""
    current_time = time.ticks_us()  # 获取当前时间（单位：微秒）

    if last_time[0] is None:  # 如果是第一次调用，更新last_time
        last_time[0] = current_time
        return 0.000_001  # 防止除零错误
    
    else:  # 计算时间差
        diff = time.ticks_diff(current_time, last_time[0])  # 计算时间差
        last_time[0] = current_time  # 更新上次调用时间
        return diff  # 返回时间差us


async def read_espnow():
    """读取espnow数据并进行解包处理"""
    while True:
        # print("正在读取espnow数据...")
        host, msg = now.recv()  # 读取所有可用的数据
        process_espnow_data(msg)  # 处理接收到的数据
        
        await asyncio.sleep(0.001)  # 等待一段时间再检查

def process_espnow_data(msg):
    if msg:
        try:
            data = json.loads(msg)  # 将接收到的消息从 JSON 字符串转换为字典
            print(data)

            lx = data.get("lx", 0)
            ly = data.get("ly", 0)
            rx = data.get("rx", 0)
            ry = data.get("ry", 0)
            
            lx -= 90
            ly -= 70
            rx -= 0
            ry -= 0
            
            print(f"接收到 espnow 数据: lx={lx}, ly={ly}, rx={rx}, ry={ry}")
            
            DEAD_AREA = 120  # 摇杆死区
            MAP_COEFF = 58   # 摇杆映射系数 (根据实际需求调整)

            # 检查lx, ly, rx, ry中是否至少有一个绝对值超过设定值
            stick_work = any(abs(value) > DEAD_AREA for value in [lx, ly, rx, ry])
            
            if stick_work:
                led.value(not led.value())  # 闪烁led

                v_y = limit_value(ly) / 3000 * MAP_COEFF if abs(ly) > DEAD_AREA else 0
                v_x = limit_value(lx) / 3000 * MAP_COEFF if abs(lx) > DEAD_AREA else 0
                v_w = limit_value(-rx) / 3000 * MAP_COEFF if abs(rx) > DEAD_AREA else 0

                robot.move(v_y, v_x, v_w)  # 调用移动函数
                
            else:
                robot.move(0, 0, 0)
                led.value(0)
            
        except ValueError as e:
            print(f'解析消息失败: {e}')
    else:
        print('No message received')
        

async def read_uart():
    while True:
        print("正在读取串口数据...")
        if uart.any():  # 检查是否有可读数据
            data = uart.read(uart.any())  # 读取所有可用的数据
            process_uart_data(data)  # 处理接收到的数据
        await asyncio.sleep(0.001)  # 等待一段时间再检查

def process_uart_data(data):
    # 检查数据长度
    # 解包数据
    try:
        pass

    except Exception as e:
        print(f"解包数据时出错: {e}")

async def main():
    await asyncio.gather(
        # read_uart(),   # 启动读取 UART 的任务
        read_espnow(), # 启动读取 espnow 的任务
    )

# 运行主协程
asyncio.run(main())

# 数据格式
data_now = {
    "lx": 0, "ly": 0, "ls_sw": True,
    "rx": 0, "ry": 0, "rs_sw": False,
}