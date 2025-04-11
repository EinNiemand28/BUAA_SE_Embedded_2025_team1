#!/usr/bin/python3
# coding=utf-8

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

if __name__ == "__main__":
    rospy.init_node("simple_goal")
    # 生成一个导航请求客户端
    ac = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    # 等待服务器端启动
    ac.wait_for_server()

    # 构建目标航点消息
    goal = MoveBaseGoal()
    # 目标航点的参考坐标系
    # goal.target_pose.header.frame_id="base_footprint"
    goal.target_pose.header.frame_id="map"
    # 目标航点在参考坐标系里的三维数值
    goal.target_pose.pose.position.x = 0.0
    goal.target_pose.pose.position.y = 0.0
    goal.target_pose.pose.position.z = 0.0
    # 目标航点在参考坐标系里的朝向信息
    goal.target_pose.pose.orientation.x = 0.0
    goal.target_pose.pose.orientation.y = 0.0
    goal.target_pose.pose.orientation.z = 0.0
    goal.target_pose.pose.orientation.w = 1.0

    # 发送目标航点
    ac.send_goal(goal)
    rospy.loginfo("goal sent, start to move")    
    # 等待目标航点到达
    ac.wait_for_result()
    # 获取目标航点到达的结果
    result = ac.get_result()
    # 判断目标航点是否到达
    if result:
        rospy.loginfo("goal reached")
    else:
        rospy.loginfo("goal not reached")