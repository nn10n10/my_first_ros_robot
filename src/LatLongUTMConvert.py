#!/usr/bin/env python
import sys
from actionlib.action_client import GoalManager
from rospy.client import init_node
from sensor_msgs.msg import NavSatFix
from tf.transformations import quaternion_from_euler
import numpy as np
import rospy
import actionlib
import tf
import actionlib
import tf2_ros
from geometry_msgs.msg import Pose, Quaternion
from geodesy import utm
from math import *
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


def next_pos(lat,lon):

    rospy.loginfo("Moving to (%f,%f)" %(lat,lon))

    point = utm.fromLatLong(lat,lon)
    # 取得当前位置
    # pos = tf2_ros.Buffer.lookup_transform(self,"map","base_link",rospy.Time(0),rospy.Duration(2.0))
    pos_curr = utm.fromLatLong(36.00341,139.39296)
    # 计算现在位置与目标点的角度
    angle_to_goal = atan2(point.northing - pos_curr.northing , point.easting - pos_curr.easting)
    odom_quat = quaternion_from_euler(0, 0,angle_to_goal)

    goal=MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp    = rospy.get_rostime()
    goal.target_pose.pose.position.x = point.easting
    goal.target_pose.pose.position.y = point.northing
    goal.target_pose.pose.orientation.x = odom_quat[0]
    goal.target_pose.pose.orientation.y = odom_quat[1]
    goal.target_pose.pose.orientation.z = odom_quat[2]
    goal.target_pose.pose.orientation.w = odom_quat[3]
    return goal
    

def on_shutdown(self):
    rospy.loginfo("Canceling all goals")
    client.cancel_all_goals()

if __name__ == '__main__':
    #节点初始化
    rospy.init_node('move_base_gps_node')
    #创建MoveBaseAction client
    action_server_name = 'move_base'
    client = actionlib.SimpleActionClient(action_server_name, MoveBaseAction)
    #等待MoveBaseAction server启动  
    rospy.loginfo('Waiting for action Server '+action_server_name)
    client.wait_for_server()
    rospy.loginfo('Action Server Found...'+action_server_name)
    lat=36.00341
    lon=139.39113
    client.send_goal(next_pos(lat,lon))
    client.wait_for_result()
    






