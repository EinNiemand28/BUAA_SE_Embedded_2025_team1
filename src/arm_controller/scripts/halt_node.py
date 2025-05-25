#!/usr/bin/python3
# coding=utf-8

import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose

if __name__ == "__main__":
    rospy.init_node("halt_node")
    # 发布抓取行为激活话题
    grab_pub = rospy.Publisher("/wpb_home/grab_action", Pose, queue_size=10)
    # 发布机械臂控制话题
    mani_pub = rospy.Publisher("/wpb_home/mani_ctrl",JointState,queue_size=30)

    # 用于迫使抓取立刻停止
    halt_grab_msg = Pose()
    grab_pub.publish(halt_grab_msg)

    # 构造机械臂控制消息并进行赋值
    msg = JointState()
    msg.name = ['lift', 'gripper']
    msg.position = [ 0 , 0 ]
    msg.velocity = [ 0 , 0 ]
    # 延时三秒，让后台的话题发布操作能够完成
    rospy.sleep(3.0)

    rospy.loginfo("[mani_ctrl] Halt")
    mani_pub.publish(msg)