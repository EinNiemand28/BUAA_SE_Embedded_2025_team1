#!/usr/bin/python3

"""
记得把place的功能直接融合进来。机械臂急停倒是可以放到专门的另一个文件里，急停可以仿照place里进行参数设置。
"""

import rospy
from std_srvs.srv import Trigger, TriggerResponse
from arm_controller.srv import Goal, GoalResponse

class ArmControllerServer:
    def __init__(self):
        # 抓取服务
        self.grab_service = rospy.Service('/grab_service', Trigger, self.grab_target)

        # 放置服务
        self.place_service = rospy.Service('/place_service', Goal, self.place_to_goal)

        # 机械臂急停
        self.halt_service = rospy.Service('/halt_arm_service', Goal, self.halt_arm)

        # 存储抓取指令
        self.grab_command = "rosrun arm_controller obj_grab_node.py"

        # 存储放置指令
        self.place_command = "rosrun arm_controller obj_grab_node.py"

