#!/usr/bin/env python3
# encoding: utf-8
# @data:2022/11/18
# @author:aiden
# 导航搬运
import math
import rospy
import numpy as np
import sdk.common as common
from interfaces.srv import SetPose2D
from std_srvs.srv import Trigger, TriggerResponse
from move_base_msgs.msg import MoveBaseActionResult
from geometry_msgs.msg import PointStamped, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray

markerArray = MarkerArray()

def start_pick_srv_callback(self, msg):
    global markerArray
    
    rospy.loginfo("start navigaiton pick")

    markerArray = MarkerArray()
    marker_Array = MarkerArray()
    marker = Marker()
    marker.header.frame_id = map_frame
    marker.action = Marker.DELETEALL
    marker_Array.markers.append(marker)

    mark_pub.publish(marker_Array)

    markerArray = MarkerArray()
    pose = PoseStamped()
    pose.header.frame_id = map_frame
    pose.header.stamp = rospy.Time.now()
    data = msg.data
    q = common.rpy2qua(math.radians(data.roll), math.radians(data.pitch), math.radians(data.yaw))
    pose.pose.position.x = data.x
    pose.pose.position.y = data.y
    pose.pose.orientation = q

    # 用数字标记来显示点(mark the point with number to display)
    marker = Marker()
    marker.header.frame_id = map_frame

    marker.type = marker.MESH_RESOURCE
    marker.mesh_resource = "package://rviz_plugin/media/flag.dae"
    marker.action = marker.ADD
    # 大小(size)
    marker.scale.x = 0.08
    marker.scale.y = 0.08
    marker.scale.z = 0.2
    # 颜色(color)
    color = list(np.random.choice(range(256), size=3))
    marker.color.a = 1
    marker.color.r = color[0] / 255.0
    marker.color.g = color[1] / 255.0
    marker.color.b = color[2] / 255.0
    # marker.lifetime = rospy.Duration(10)  # 显示时间，没有设置默认一直保留(display time. If not set, it will be kept by default)
    # 位置姿态
    marker.pose.position.x = pose.pose.position.x
    marker.pose.position.y = pose.pose.position.y
    marker.pose.orientation = pose.pose.orientation
    markerArray.markers.append(marker)

    mark_pub.publish(markerArray)
    nav_pub.publish(pose)

    return [True, 'navigation pick']

def start_place_srv_callback(self, msg):
    rospy.loginfo("start navigaiton place")

    pose = PoseStamped()
    pose.header.frame_id = map_frame
    pose.header.stamp = rospy.Time.now()
    data = msg.data
    q = common.rpy2qua(math.radians(data.roll), math.radians(data.pitch), math.radians(data.yaw))
    pose.pose.position.x = data.x
    pose.pose.position.y = data.y
    pose.pose.orientation = q

    # 用数字标记来显示点(mark the point with number to display)
    marker = Marker()
    marker.header.frame_id = map_frame

    marker.type = marker.MESH_RESOURCE
    marker.mesh_resource = "package://rviz_plugin/media/flag.dae"
    marker.action = marker.ADD
    # 大小(size)
    marker.scale.x = 0.08
    marker.scale.y = 0.08
    marker.scale.z = 0.2
    # 颜色(color)
    color = list(np.random.choice(range(256), size=3))
    marker.color.a = 1
    marker.color.r = color[0] / 255.0
    marker.color.g = color[1] / 255.0
    marker.color.b = color[2] / 255.0
    # marker.lifetime = rospy.Duration(10)  # 显示时间，没有设置默认一直保留(display time. If not set, it will be kept by default)
    # 位置姿态
    marker.pose.position.x = pose.pose.position.x
    marker.pose.position.y = pose.pose.position.y
    marker.pose.orientation = pose.pose.orientation
    markerArray.markers.append(marker)

    mark_pub.publish(markerArray)
    nav_pub.publish(pose)

    return [True, 'navigation place']

pick = True
place = False
def status_callback(msg):
    if msg.status.status == 3:
        if pick:
            res = rospy.ServiceProxy('/automatic_pick/pick', Trigger)()
            if res.success:
                print('start pick success')
            else:
                print('start pick failed')
        else:
            res = rospy.ServiceProxy('/automatic_pick/place', Trigger)()
            if res.success:
                print('start place success')
            else:
                print('start place failed')

def goal_callback(msg):
    global pick, place

    if rospy.get_param('/automatic_pick/status') == 'start' or rospy.get_param('/automatic_pick/status') == 'place_finish':  # 处于可以pick的状态
        pick = True
        place = False
        print('nav pick')
    elif rospy.get_param('/automatic_pick/status') == 'pick_finish':  # 处于可以place的状态
        pick = False
        place = True
        print('nav place')
    elif rospy.get_param('/automatic_pick/status') == 'start_pick':  # 在pick过程再次导航，会触发取消当前pick直到导航完成
        res = rospy.ServiceProxy('/automatic_pick/cancel', Trigger)()
        if res.success:
            print('cancel pick')
        else:
            print('cancel pick failed')
    elif rospy.get_param('/automatic_pick/status') == 'start_place':  # 在place过程再次导航，会触发取消当前place直到导航完成
        res = rospy.ServiceProxy('/automatic_pick/cancel', Trigger)()
        if res.success:
            print('cancel place')
        else:
            print('cancel place failed')

    goal_pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('navigation_transport')

    map_frame = rospy.get_param('~map_frame', 'map')
    nav_goal = rospy.get_param('~nav_goal', '/nav_goal')
    move_base_result = rospy.get_param('~move_base_result', '/move_base/result')

    goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
    nav_pub = rospy.Publisher(nav_goal, PoseStamped, queue_size=1)
    rospy.Service('~pick', SetPose2D, start_pick_srv_callback)
    rospy.Service('~place', SetPose2D, start_place_srv_callback)
    rospy.Subscriber(nav_goal, PoseStamped, goal_callback)
    rospy.Subscriber(move_base_result, MoveBaseActionResult, status_callback)
    mark_pub = rospy.Publisher('path_point', MarkerArray, queue_size=100)
    rospy.sleep(0.2)
    rospy.set_param('~init_finish', True)
    try:
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))
