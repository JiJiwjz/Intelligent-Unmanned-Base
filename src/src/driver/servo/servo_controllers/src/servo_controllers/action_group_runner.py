#!/usr/bin/env python3
# encoding: utf-8
import os
import rospy
import ujson
import actionlib
import threading
from servo_msgs.msg import ActionGroupRunnerAction
from servo_msgs.msg import ActionGroupRunnerResult
from servo_msgs.msg import ActionGroupRunnerFeedback

class ActionGroupRunner:
    _feedback = ActionGroupRunnerFeedback()
    _result = ActionGroupRunnerResult()

    def __init__(self,
                 controller_namespace="ActionGroupRunner",
                 set_multi_pos=(lambda x: None),
                 path="/home/ubuntu/ActionGroups"):
        self.controller_namespace = controller_namespace
        self.set_multi_pos = set_multi_pos
        self.lock = threading.Lock()
        self.path = path
        self.action_server = actionlib.SimpleActionServer(self.controller_namespace,
                                                          ActionGroupRunnerAction,
                                                          execute_cb=self.process_action_group_run, auto_start=False)

    """
        检查文件是否为合法的动作组文件
    """
    def get_actions_from_file(self, file_name):
        path = os.path.join(self.path, file_name)
        ret = None
        try:
            with open(path, 'r') as f:
                content = ujson.loads(f.read())
                actions = content['Actions']
                acts = []
                for action in actions:
                    joints = action['Joints']
                    ids = tuple(joints.keys())
                    duration = action['Duration']
                    acts.append((duration, tuple(map(lambda id_: (int(id_), joints[id_], duration), ids))))
                ret = acts
        except Exception as e:
            rospy.logerr("")
            print(e)
        return ret

    def start(self):
        self.action_server.start()

    def runner(self, act_s, repeat=1):
        print("running")
        for i in range(repeat):
            step = 0
            for act in act_s:
                if self.action_server.is_preempt_requested():
                    return 2
                dur_, pos_ = act
                try:
                    self.set_multi_pos(pos_)
                except Exception as e:
                    print("set mult pos failed" + str(e))
                    return 3
                self._feedback.step = step
                self.action_server.publish_feedback(self._feedback)
                rospy.sleep(dur_ / 1000.0)
                step += 1
        return 1

    def process_action_group_run(self, goal):
        name = goal.name
        repeat = goal.repeat
        self._feedback.name = name
        self._result.name = name

        '''所有动作组文件均以".json"结束，对不以".json"结束的动作组尝试运行命令，命令若有参数需要传入的使用'|'符号将命令与参数分隔'''
        print(name)
        print(repeat)
        success = 0
        with self.lock:
            act_s = self.get_actions_from_file(name)
            if act_s is not None:
                success = self.runner(act_s, repeat)
            if success == 1:
                self._result.result = "success"
                self.action_server.set_succeeded(self._result)
            elif success == 2:
                self._result.result = "cancel"
                self.action_server.set_preempted(self._result)
            else:
                self._result.result = "aborted"
                self.action_server.set_aborted(self._result)
