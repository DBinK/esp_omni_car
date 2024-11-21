# esp-omni-car 

使用 ESP32S2 制作的 打令小宝机器人 全向轮底盘 控制程序 

配套的遥控程序, 也是用 ESP32S2 做的, 包含:

`boot.py` 一些单片机初始化的代码

`main.py` 主程序, 使用 ADC 和 IO中断 获取摇杆信号, 打包成 JSON 后通过 ESP-NOW 发送

使用 MicroPython 编写
