#!/usr/bin/env python3
# encoding: utf-8
# @data:2022/08/28
# @author:aiden
"""
每30分钟检测一次日志文件，如果大于900m，则清空(the log file will be detected every 30 minutes. If they are greater than 900m, they will be cleared.)
过大的日志文件会导致cpu占用过高，运行异常(too large log file will occupy too much cpu, which will results in abnormal operation)
关闭此功能，重启生效：sudo systemctl disable clear_log.service(close this function, and take effect after restart)
"""
import os
import sys
import time
import signal

running = True
def handler(signum, frame):
    global running

    running = False
    print('exit')
    sys.exit(0)

signal.signal(signal.SIGINT, handler)

ros_log_path = os.path.join(os.environ['HOME'], '.ros/log')

log_size = 0
while running:
    try:
        for root, dirs, files in os.walk(ros_log_path):
            for f in files:
                log_size += os.path.getsize(os.path.join(root, f))
        log_size /= float(1024*1024)
        print('当前ros日志大小:{}m 大于900m将触发删除'.format(log_size))
        if log_size > 900:
            os.system('sudo rm -rf {}/*'.format(ros_log_path))
        time.sleep(30*60)
    except BaseException as e:
        print(e)
