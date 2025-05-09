#!/usr/bin/python3

"""
本脚本实现了一个ROS服务，用于传输数据给导航模块，并支持临时取消导航。

需要指出的是，这个脚本直接传输数据给导航模块，而并不关注导航的环境是否已经设定完毕。也就是说在启动这个脚本前，必须保证run_full.launch和simple_goal.py节点已经启动。
"""

import rospy
import actionlib
from std_srvs.srv import Trigger, TriggerResponse
from navigation.srv import Goal, GoalResponse
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import GoalStatus
from navigation.msg import *

class GoalServer:
    def __init__(self):
        # 启动导航服务
        self.goal_service = rospy.Service('/navigation_service', Goal, self.handle_goal)
        rospy.loginfo("导航服务已就绪")

        # 终止导航服务
        self.stop_service = rospy.Service('/halt_navigation', Trigger, self.stop_navigation)

        # action client对象
        self.client = actionlib.SimpleActionClient("/navigation/navigate", NavigateAction)

        # 存储此次导航目标
        self.goal = None

        # 此次导航的结果
        self.result = None

    def handle_goal(self, req):
        # 检查是否已经有目标
        if self.client.get_state() == GoalStatus.ACTIVE:
            return GoalResponse(message="有正在进行的导航，若要导航则请先结束当前导航")

        # 发送目标
        self.goal = NavigateGoal()
        self.goal.pos.position.x = req.px
        self.goal.pos.position.y = req.py
        self.goal.pos.orientation.z = req.oz
        self.goal.pos.orientation.w = 1.0
        self.client.wait_for_server()
        self.client.send_goal_and_wait(self.goal)

        # 确定导航结果
        return GoalResponse(message="导航结果：" + str(self.client.get_result().result))
        
    def stop_navigation(self, req):
        # 取消导航目标
        if self.client.get_state() == GoalStatus.ACTIVE:
            self.client.cancel_goal()
            rospy.loginfo("导航已取消")

            # 保证机器人原地静止
            movement_topic = rospy.Publisher('cmd_vel', Twist, queue_size=10)
            twist = Twist()
            twist.linear.x = 0
            twist.linear.y = 0
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = 0
            while self.client.get_state() == GoalStatus.ACTIVE:
                rospy.sleep(1)
            movement_topic.publish(twist)

            return TriggerResponse(success=True, message="导航已取消")
        else:
            return TriggerResponse(success=False, message="没有正在进行的导航任务")

if __name__ == "__main__":
    rospy.init_node('navigation_service')
    starter = GoalServer()
    rospy.spin()