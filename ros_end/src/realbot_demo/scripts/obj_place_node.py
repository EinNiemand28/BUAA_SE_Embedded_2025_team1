#!/usr/bin/python3
# coding=utf-8

import rospy
from sensor_msgs.msg import JointState

state = 0

if __name__ == "__main__":
    rospy.init_node("place_node")
    # 发布机械臂控制话题
    mani_pub = rospy.Publisher("/wpb_home/mani_ctrl",JointState,queue_size=30)
    # 构造机械臂控制消息并进行赋值
    msg = JointState()
    msg.name = ['lift', 'gripper']
    msg.position = [ 0 , 0 ]
    msg.velocity = [ 0 , 0 ]
    # 延时三秒，让后台的话题发布操作能够完成
    rospy.sleep(3.0)

    

    rospy.loginfo("[mani_ctrl] UP -> DOWN")
    msg.position[0] = 0.5   # 升降高度(单位:米)
    msg.velocity[0] = 0.5   #升降速度(单位:米/秒)
    msg.position[1] = 0.1   #手爪指间距(单位:米)
    msg.velocity[1] = 5         #手爪开合角速度(单位:度/秒)
    mani_pub.publish(msg)
    rospy.sleep(3.0)

    rospy.loginfo("[mani_ctrl] DOWN -> ZERO")
    msg.position[0] = 0   # 升降高度(单位:米)
    msg.velocity[0] = 0.5   #升降速度(单位:米/秒)
    msg.position[1] = 0.1   #手爪指间距(单位:米)
    msg.velocity[1] = 5         #手爪开合角速度(单位:度/秒)
    mani_pub.publish(msg)