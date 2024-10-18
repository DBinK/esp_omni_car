# This file is executed on every boot (including wake-boot from deepsleep)
#import esp
#esp.osdebug(None)
#import webrepl
#webrepl.start()

import time
from machine import Pin

time.sleep(1)  # 防止点停止按钮后马上再启动导致 Thonny 连接不上

# 释放所有GPIO, 断电重上电不再失控
def release_all_GPIO():
    for i in range(0, 49):
        try:
            GND = Pin(i, Pin.OUT, value=0)
            print(f"releasing gpio {i}")
        except:
            print(f"skip gpio {i}")
            continue

release_all_GPIO()