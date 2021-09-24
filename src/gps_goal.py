#! /usr/bin/env python

import time
from genpy import message
import geometry_msgs.msg
import rospy
from sensor_msgs.msg import NavSatFix
from swri_transform_util.origin_manager import OriginManager

GpsMsgs=NavSatFix()

def callback(data):
    rospy.loginfo("Longitude: %f, Latitude %f" % (data.longitude, data.latitude))
    GpsMsgs.longitude=data.longtitude
    GpsMsgs.latitude=data.latitude


def GPSlistener():
    rospy.init_node('GPSlistener', anonymous=True)
    sub_gps=rospy.Subscriber("/gps/fix", NavSatFix, callback)
    rospy.spin()
def getOriginLL(lat,lang):
    OriginManager.set_origin("/map",lat,lang)
if __name__ == '__main__':
    GPSlistener()
    getOriginLL(GpsMsgs.latitude,GpsMsgs.longitude)
    

