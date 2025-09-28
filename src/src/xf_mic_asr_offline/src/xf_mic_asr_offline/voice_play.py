#!/usr/bin/env python3
# encoding: utf-8
# @Author: Aiden
# @Date: 2022/11/21
import os

wav_path = os.path.join(os.path.split(os.path.realpath(__file__))[0], 'feedback_voice')

def get_path(f, language='Chinese'):
    if language == 'Chinese':
        return os.path.join(wav_path, f + '.wav')
    else:    
        return os.path.join(wav_path, 'english', f + '.wav')

def play(voice, volume=100, language='Chinese'):
    try:
        os.system('amixer -q -D pulse set Master {}%'.format(volume))
        os.system('play -q ' + get_path(voice, language))
    except BaseException as e:
        print('error', e)

if __name__ == '__main__':
    play('ok')
    play('running', language="English")

