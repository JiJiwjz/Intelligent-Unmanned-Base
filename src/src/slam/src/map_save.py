#!/usr/bin/env python3
# encoding: utf-8
import os
import rospy
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty, EmptyResponse

class MapSaveNode():
    def __init__(self, name):
        rospy.init_node(name, anonymous=False)
       
        self.map_frame = rospy.get_param('~map_frame', 'map')
        goal_topic = rospy.get_param('~goal', 'move_base_simple/goal')
        self.goal_pub = rospy.Publisher(goal_topic, PoseStamped, queue_size=1)

        rospy.Service('/map_save', Empty, self.map_save_srv_callback)

        try:
            rospy.spin()
        except Exception as e:
            rospy.logerr(str(e))

    def map_save_srv_callback(self, msg):
        origin = PoseStamped()
        origin.header.seq = 0
        origin.header.frame_id = self.map_frame
        origin.pose.position.x = 0
        origin.pose.position.y = 0
        origin.pose.position.z = 0
        origin.pose.orientation.x = 0
        origin.pose.orientation.y = 0
        origin.pose.orientation.z = 0
        origin.pose.orientation.w = 1
         
        os.system('rosrun map_server map_saver -f $HOME/ros_ws/src/slam/maps/explore')
        rospy.loginfo('************map save as slam/maps/explore**************')
        self.goal_pub.publish(origin)

        return EmptyResponse()

if __name__ == '__main__':
    MapSaveNode('map_save')
