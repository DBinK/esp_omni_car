import network
import espnow
import time

# 必须激活一个WLAN接口以进行发送()/接收()
sta = network.WLAN(network.STA_IF)  # 或者 network.AP_IF
sta.active(True)
sta.disconnect()      # 对于ESP8266

# 初始化ESP-NOW
e = espnow.ESPNow()
e.active(True)
peer = b'\xff\xff\xff\xff\xff\xff'  # 对端WiFi接口的MAC地址
e.add_peer(peer)      # 发送数据前必须添加对端

# 发送初始消息
e.send(peer, "Starting...")

# 循环发送数据
for i in range(100):
    str1 = str(i) * 20
    
    print(str1)
    e.send(peer, str1, True)
    
    time.sleep(1)

# 发送结束信号
e.send(peer, b'end')

# 断开WLAN连接
sta.disconnect()      # 对于ESP8266