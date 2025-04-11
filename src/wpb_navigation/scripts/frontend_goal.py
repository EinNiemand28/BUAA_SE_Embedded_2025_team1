#!/usr/bin/python3
# coding=utf-8

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from wpb_navigation.srv import frontend_goal, frontend_goalResponse

def handle_navigation_request(req):
    # 生成一个导航请求客户端
    ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    # 等待服务器端启动
    ac.wait_for_server()
    # 构建目标航点消息
    goal = MoveBaseGoal()
    # 目标航点的参考坐标系
    goal.target_pose.header.frame_id = "map"
    # 目标航点在参考坐标系里的三维数值
    goal.target_pose.pose.position.x = req.px
    goal.target_pose.pose.position.y = req.py
    goal.target_pose.pose.position.z = req.pz
    # 目标航点在参考坐标系里的朝向信息
    goal.target_pose.pose.orientation.x = req.ox
    goal.target_pose.pose.orientation.y = req.oy
    goal.target_pose.pose.orientation.z = req.oz
    goal.target_pose.pose.orientation.w = req.ow

    # 发送目标航点
    ac.send_goal(goal)
    rospy.loginfo("goal sent, start to move")

    # 等待目标航点到达
    ac.wait_for_result()
    # 获取目标航点到达的结果
    result = ac.get_result()
    # 判断目标航点是否到达
    if result:
        return frontend_goalResponse("goal reached")
    else:
        return frontend_goalResponse("goal not reached")

def navigation_server():
    rospy.init_node("navigation_server")
    # 启动服务
    a = rospy.Service('navigation_server', frontend_goal, handle_navigation_request)

    rospy.loginfo("Navigation server is ready")
    rospy.spin()

if __name__ == "__main__":
    navigation_server()