#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
import json

def callback(msg):
    data = {
        "lat": msg.latitude,
        "lon": msg.longitude,
        "alt": msg.altitude
    }
    pub.publish(json.dumps(data))

rospy.init_node('gps_web_stream')
pub = rospy.Publisher('/web_gps_stream', String, queue_size=1)
rospy.Subscriber('/uav1/mavros/global_position/global', NavSatFix, callback)
rospy.spin()
