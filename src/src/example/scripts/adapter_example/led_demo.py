#!/usr/bin/python3
# coding=utf8
import time
from sdk import led

print('led blink at 5hz')

try:
    while True:
        led.on()  # led亮
        time.sleep(0.1)  # 延时
        led.off()  # led灭
        time.sleep(0.1)
except KeyboardInterrupt:
    led.off()  # 关闭程序时将led熄灭

