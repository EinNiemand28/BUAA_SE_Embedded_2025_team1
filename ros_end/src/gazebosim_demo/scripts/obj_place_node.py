#!/usr/bin/python3

# coding=utf-8
import rospy
from wpb_home_behaviors.msg import Coord
from std_msgs.msg import String
from geometry_msgs.msg import Pose

def cbGrabResult(msg):
    rospy.logwarn("抓取执行结果 = %s",msg.data)

if __name__ == "__main__":
    rospy.init_node("place_node")
    place_msg = Pose()
    place_msg.position.x = 0.0
    place_msg.position.y = 2.0
    place_msg.position.z = 0.91
    place_pub = rospy.Publisher("/wpb_home/place_action", Pose, queue_size=10)
    result_sub = rospy.Subscriber("/wpb_home/place_result", String, cbGrabResult, queue_size=10)
    place_pub.publish(place_msg)
    rospy.spin()