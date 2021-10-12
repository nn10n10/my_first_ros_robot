import geodesy
from sensor_msgs.msg import NavSatFix
import rospy
from geometry_msgs.msg import Pose, Quaternion
from geodesy import utm, wu_point
import time
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult, MoveBaseFeedback

class gps_point:
    lat = 0.0
    lon = 0.0
    theta= 0.0
toxy=wu_point.WuPoint.toPointXY

def feedback_callback(feedback):
    rospy.loginfo(str(feedback))

goal=MoveBaseGoal()
goal.target_pose.header.frame_id = "map"
goal.target_pose.header.stamp    = rospy.get_rostime()
# goal.target_pose.pose = 

LLtoUTM = utm.fromLatLong
tess=LLtoUTM(36.00341,139.39113)
rospy.loginfo(str(tess.northing))
rospy.loginfo(str(tess.easting))

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
    #发送goal
    client.send_goal(goal, feedback_cb=feedback_callback)







