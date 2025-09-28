#!/usr/bin/python3
# coding=utf8
from sdk import button

while True:
    try:
        print('\rkey1: {}   key2: {}'.format(button.get_button_status('key1'), button.get_button_status('key2')), end='', flush=True)  # 打印key状态
    except KeyboardInterrupt:
        break
