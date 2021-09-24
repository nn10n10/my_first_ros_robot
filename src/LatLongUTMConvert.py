from sensor_msgs.msg import NavSatFix
import rospy
from geometry_msgs.msg import Pose, Quaternion
from geodesy import utm, wu_point
import time
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult, MoveBaseFeedback

toxy=wu_point.WuPoint.toPointXY


def feedback_callback(feedback):
    rospy.loginfo(str(feedback))
# initializes the action client node
rospy.init_node('move_base_gps_node')
action_server_name = 'move_base'
client = actionlib.SimpleActionClient(action_server_name, MoveBaseAction)

# waits until the action server is up and running
rospy.loginfo('Waiting for action Server '+action_server_name)
client.wait_for_server()
rospy.loginfo('Action Server Found...'+action_server_name)

goal=MoveBaseGoal()
goal.target_pose.header.frame_id = "map"
goal.target_pose.header.stamp    = rospy.get_rostime()
# goal.target_pose.pose = 
client.send_goal(goal, feedback_cb=feedback_callback)
LLtoUTM = utm.fromLatLong
tess=LLtoUTM(36.00341,139.39113)
xy=toxy(tess)
rospy.loginfo(str(tess.northing))
rospy.loginfo(str(tess.easting))
rospy.loginfo(str(xy.x))
rospy.loginfo(str(xy.y))







