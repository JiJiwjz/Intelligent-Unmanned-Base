#!/usr/bin/env python3
# encoding: utf-8
import os
import time
import rospy
from ros_robot_controller.srv import GetBusServoState
from ros_robot_controller.msg import BusServoState, GetBusServoCmd, SetBusServoState

exception = None

class servo_state:
    def __init__(self):
        self.start_timestamp = time.time()
        self.end_timestamp = time.time()
        self.speed = 200
        self.goal = 500
        self.estimated_pos = 500

class ServoIO:
    def __init__(self):
        self.ser = None
        self.timeout = 10
        self.servos = {1: servo_state(), 2: servo_state(), 3: servo_state(), 4: servo_state(), 5: servo_state(),
                6: servo_state(), 7: servo_state(), 8: servo_state(), 9: servo_state(), 10: servo_state()}
        self.servo_state_pub = rospy.Publisher('ros_robot_controller/bus_servo/set_state', SetBusServoState, queue_size=10)

    def ping(self, servo_id):
        msg = GetBusServoCmd()
        msg.id = servo_id
        msg.get_id = 1
        for i in range(0, 20):
            res = rospy.ServiceProxy('/ros_robot_controller/bus_servo/get_state', GetBusServoState)(msg)[1]
            if res[0].present_id == servo_id:
                return True
        return False

    def get_position(self, servo_id, fake_read=False):
        if fake_read:
            return self.servos[servo_id].goal
        else:
            count = 0
            msg = GetBusServoCmd()
            msg.id = servo_id
            msg.get_position = 1
            while True:
                count += 1
                res = rospy.ServiceProxy('/ros_robot_controller/bus_servo/get_state', GetBusServoState)(msg)[1]
                if res[0].position != []:
                    return res[0].position
                if count > self.timeout:
                    return None

    def get_voltage(self, servo_id):
        count = 0
        msg = GetBusServoCmd()
        msg.id = servo_id
        msg.get_voltage = 1
        while True:
            count += 1
            res = rospy.ServiceProxy('/ros_robot_controller/bus_servo/get_state', GetBusServoState)(msg)[1]
            if res[0].voltage != []:
                return res[0].voltage
            if count > self.timeout:
                return None

    def get_feedback(self, servo_id, fake_read=False):
        position = self.get_position(servo_id, fake_read)
        if position != []:
            time.sleep(0.01)
            goal = self.servos[servo_id].goal
            error = position - goal
            voltage = 9000
            timestamp = time.time()

            return {'timestamp': timestamp,
                    'id': servo_id,
                    'goal': goal,
                    'position': position,
                    'error': error,
                    'voltage': voltage,
                    }
        else:
            return None

    def set_timeout(self, t=10):
        self.timeout = t

    def set_servo_id(self, oldid, newid):
        '''
        配置舵机id号, 出厂默认为1
        :param oldid: 原来的id， 出厂默认为1
        :param newid: 新的id
        '''
        servo_state = BusServoState()
        servo_state.present_id = [1, oldid]
        servo_state.target_id = [1, newid]
        msg = SetBusServoState()
        msg.state = [servo_state]
        self.servo_state_pub.publish(msg)

    def get_servo_id(self, servo_id=None):
        '''
        读取串口舵机id
        :param id: 默认为空
        :return: 返回舵机id
        '''
        count = 0
        msg = GetBusServoCmd()
        if servo_id is None:
            msg.id = 254
        else:
            msg.id = servo_id
        msg.get_id = 1
        while True:
            count += 1
            res = rospy.ServiceProxy('/ros_robot_controller/bus_servo/get_state', GetBusServoState)(msg)[1]
            if res[0].present_id != []:
                return res[0].present_id
            if count > self.timeout:
                return None

    def set_position(self, servo_id, position, duration=None):
        """
        驱动串口舵机转到指定位置
        :param id: 要驱动的舵机id
        :pulse: 位置
        :use_time: 转动需要的时间
        """
        # print("id:{}, pos:{}, duration:{}".format(servo_id, position, duration))
        servo = self.servos[servo_id]

        current_timestamp = time.time()
        if duration is None:
            duration = 0.02
        servo.goal = int(position)
        duration = 0 if duration < 0 else 30 if duration > 30 else duration
        position = 0 if position < 0 else 1000 if position > 1000 else position
        duration = int(duration)
        position = int(position)
        
        servo_state = BusServoState()
        servo_state.present_id = [1, servo_id]
        servo_state.position = [1, position]
        data = SetBusServoState()
        data.state = [servo_state]
        data.duration = duration
        self.servo_state_pub.publish(data)

    def stop(self, servo_id):
        '''
        停止舵机运行
        :param id:
        :return:
        '''
        self.write(servo_id, SERVO_MOVE_STOP, ())

    def set_servo_offset(self, servo_id, offset=0):
        '''
        调整偏差
        :param id: 舵机id
        :param d:  偏差
        '''
        servo_state = BusServoState()
        servo_state.present_id = [1, servo_id]
        servo_state.offset = [1, offset]
        msg = SetBusServoState()
        msg.state = [servo_state]
        self.servo_state_pub.publish(msg)

    def save_servo_offset(self, servo_id):
        '''
        配置偏差，掉电保护
        :param id: 舵机id
        '''
        servo_state = BusServoState()
        servo_state.present_id = [1, servo_id]
        servo_state.save_offset = [1, 1]
        msg = SetBusServoState()
        msg.state = [servo_state]
        self.servo_state_pub.publish(msg)
        
    def get_servo_offset(self, servo_id):
        '''
        读取偏差值
        :param id: 舵机号
        :return:
        '''
        count = 0
        msg = GetBusServoCmd()
        msg.id = servo_id
        msg.get_offset = 1
        while True:
            count += 1
            res = rospy.ServiceProxy('/ros_robot_controller/bus_servo/get_state', GetBusServoState)(msg)[1]
            if res[0].offset != []:
                return res[0].offset
            if count > self.timeout:
                return None

    def set_servo_range(self, servo_id, low, high):
        '''
        设置舵机转动范围
        :param id:
        :param low:
        :param high:
        :return:
        '''
        servo_state = BusServoState()
        servo_state.present_id = [1, servo_id]
        servo_state.position_limit = [1, low, high]
        msg = SetBusServoState()
        msg.state = [servo_state]
        self.servo_state_pub.publish(msg)

    def get_servo_range(self, servo_id):
        '''
        读取舵机转动范围
        :param id:
        :return: 返回元祖 0： 低位  1： 高位
        '''
        count = 0
        msg = GetBusServoCmd()
        msg.id = servo_id
        msg.get_position_limit = 1
        while True:
            count += 1
            res = rospy.ServiceProxy('/ros_robot_controller/bus_servo/get_state', GetBusServoState)(msg)[1]
            if res[0].position_limit != []:
                return res[0].position_limit
            if count > self.timeout:
                return None

    def set_servo_vin_range(self, servo_id, low, high):
        '''
        设置舵机电压范围
        :param id:
        :param low:
        :param high:
        :return:
        '''
        servo_state = BusServoState()
        servo_state.present_id = [1, servo_id]
        servo_state.voltage_limit = [1, low, high]
        msg = SetBusServoState()
        msg.state = [servo_state]
        self.servo_state_pub.publish(msg)

    def get_servo_vin_range(self, servo_id):
        '''
        读取舵机转动范围
        :param id:
        :return: 返回元祖 0： 低位  1： 高位
        '''
        count = 0
        msg = GetBusServoCmd()
        msg.id = servo_id
        msg.get_voltage_limit = 1
        while True:
            count += 1
            res = rospy.ServiceProxy('/ros_robot_controller/bus_servo/get_state', GetBusServoState)(msg)[1]
            if res[0].voltage_limit != []:
                return res[0].voltage_limit
            if count > self.timeout:
                return None

    def set_servo_temp_range(self, servo_id, m_temp):
        '''
        设置舵机最高温度报警
        :param id:
        :param m_temp:
        :return:
        '''
        servo_state = BusServoState()
        servo_state.present_id = [1, servo_id]
        servo_state.max_temperature_limit = [1, m_temp]
        msg = SetBusServoState()
        msg.state = [servo_state]
        self.servo_state_pub.publish(msg)

    def get_servo_temp_range(self, servo_id):
        '''
        读取舵机温度报警范围
        :param id:
        :return:
        '''
        count = 0
        msg = GetBusServoCmd()
        msg.id = servo_id
        msg.get_max_temperature_limit = 1
        while True:
            count += 1
            res = rospy.ServiceProxy('/ros_robot_controller/bus_servo/get_state', GetBusServoState)(msg)[1]
            if res[0].max_temperature_limit != []:
                return res[0].max_temperature_limit
            if count > self.timeout:
                return None

    def get_servo_temp(self, servo_id):
        '''
        读取舵机温度
        :param id:
        :return:
        '''
        count = 0
        msg = GetBusServoCmd()
        msg.id = servo_id
        msg.get_temperature = 1
        while True:
            count += 1
            res = rospy.ServiceProxy('/ros_robot_controller/bus_servo/get_state', GetBusServoState)(msg)[1]
            if res[0].temperature != []:
                return res[0].temperature
            if count > self.timeout:
                return None

    def get_servo_vin(self, servo_id):
        '''
        读取舵机电压
        :param id:
        :return:
        '''
        count = 0
        msg = GetBusServoCmd()
        msg.id = servo_id
        msg.get_voltage = 1
        while True:
            count += 1
            res = rospy.ServiceProxy('/ros_robot_controller/bus_servo/get_state', GetBusServoState)(msg)[1]
            if res[0].voltage != []:
                return res[0].voltage
            if count > self.timeout:
                return None

    def reset_offset(self, servo_id):
        # 舵机清零偏差和P值中位（500）
        self.set_servo_offset(servo_id, 0)    # 清零偏差
        time.sleep(0.1)
        self.set_position(servo_id, 500, 100)

    def enable_servo_torque(self, servo_id, torque=False):
        '''
        设置舵机转动力
        :param id:
        :return:
        '''
        servo_state = BusServoState()
        servo_state.present_id = [1, servo_id]
        if torque:
            servo_state.enable_torque = [1, 0]
        else:
            servo_state.enable_torque = [1, 1]
        msg = SetBusServoState()
        msg.state = [servo_state]
        self.servo_state_pub.publish(msg)

    def get_servo_torque(self, servo_id):
        count = 0
        msg = GetBusServoCmd()
        msg.id = servo_id
        msg.get_torque = 1
        while True:
            count += 1
            res = rospy.ServiceProxy('/ros_robot_controller/bus_servo/get_state', GetBusServoState)(msg)[1]
            if res[0].enable_torque != []:
                return res[0].enable_torque
            if count > self.timeout:
                return None

    def exception_on_error(self, error_code, servo_id, command_failed):
        global exception
        exception = None
        ex_message = '[servo #%d on %s@%sbps]: %s failed' % (servo_id, self.ser.port, self.ser.baudrate, command_failed)

        if not isinstance(error_code, int):
            msg = 'Communcation Error ' + ex_message
            exception = NonfatalErrorCodeError(msg, 0)
            return

class FatalErrorCodeError(Exception):
    def __init__(self, message, ec_const):
        Exception.__init__(self)
        self.message = message
        self.error_code = ec_const

    def __str__(self):
        return self.message

class NonfatalErrorCodeError(Exception):
    def __init__(self, message, ec_const):
        Exception.__init__(self)
        self.message = message
        self.error_code = ec_const

    def __str__(self):
        return self.message

class ErrorCodeError(Exception):
    def __init__(self, message, ec_const):
        Exception.__init__(self)
        self.message = message
        self.error_code = ec_const

    def __str__(self):
        return self.message

class DroppedPacketError(Exception):
    def __init__(self, message):
        Exception.__init__(self)
        self.message = message

    def __str__(self):
        return self.message

class UnsupportedFeatureError(Exception):
    def __init__(self, model_id, feature_id):
        Exception.__init__(self)
        if model_id in SERVO_PARAMS:
            model = SERVO_PARAMS[model_id]['name']
        else:
            model = 'Unknown'
        self.message = "Feature %d not supported by model %d (%s)" % (feature_id, model_id, model)

    def __str__(self):
        return self.message
