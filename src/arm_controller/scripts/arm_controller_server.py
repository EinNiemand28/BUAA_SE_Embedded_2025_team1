#!/usr/bin/python3

"""
本脚本实现了三个ROS服务，用于控制机械臂的抓取、放置和收起功能。需要注意的是，在启动这些服务前必须首先launch run_full.launch文件。
"""

import rospy
import subprocess
from arm_controller.srv import Place, PlaceResponse
from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import String
from geometry_msgs.msg import Pose

class ArmControllerServer:
    def __init__(self):
        # 抓取服务
        self.grab_service = rospy.Service('/grab_service', Trigger, self.grab)

        # 放置服务
        self.place_service = rospy.Service('/place_service', Place, self.place)

        # 机械臂自然收起服务
        self.halt_service = rospy.Service('/arm_zero_service', Trigger, self.arm_zero)

        rospy.loginfo("机械臂控制服务已启动")

        # 存储抓取指令
        self.grab_command = "rosrun arm_controller obj_grab_node.py"

        # 存储终止抓取节点的指令
        self.kill_grab_node_command = "rosnode kill grab_node"

        # 发布放置行为激活话题
        self.place_pub = rospy.Publisher("/wpb_home/place_action", Pose, queue_size=10)

        # 确定放置是否已经结束
        self.is_place_over = True

        # 订阅放置执行结果的话题
        self.place_result_sub = rospy.Subscriber("/wpb_home/place_result", String, self.cbPlaceResult, queue_size=10)

        # 延时三秒，让后台的话题初始化操作能够完成
        rospy.sleep(3.0)

        # 存储机械臂自然收起指令
        self.zero_command = "rosrun arm_controller arm_zero_node.py"

        # 存储机械臂抓取进程
        self.grab_process = None

        # 机械臂收起进程
        self.zero_process = None

    def grab(self, req):
        """
        抓取服务的回调函数
        """
        # 如果之前有抓取、放置或机械臂收起进程在运行，则告知用户并返回
        if not self.is_place_over or (self.zero_process and self.zero_process.poll() is None) or (self.grab_process and self.grab_process.poll() is None):
            rospy.logwarn("有机械臂进程在运行，请稍后再试")
            return TriggerResponse(
                success=False,
                message=f"机械臂已被占用，请稍后再试"
            )
        
        rospy.loginfo("开始抓取物品")
        # 启动抓取进程
        self.grab_process = subprocess.Popen(self.grab_command, shell=True)
        return TriggerResponse(
            success=True,
            message=f"开始抓取物品"
        )
    
    def place(self, req):
        """
        放置服务的回调函数
        """
        # 如果之前有放置进程在运行，则告知用户并返回
        if not self.is_place_over:
            rospy.logwarn("放置操作仍在进行中，请稍后再试")
            return PlaceResponse(
                message=f"放置操作仍在进行中，请稍后再试"
            )
        
        # 要求之前有抓取进程在运行，以保证可以进行放置操作
        if self.grab_process and self.grab_process.poll() is None:
            subprocess.Popen(self.kill_grab_node_command, shell=True)
            self.grab_process = None
        else:
            rospy.logwarn("没有抓取物品，无法进行放置操作")
            return PlaceResponse(
                message=f"没有抓取物品，无法进行放置操作"
            )

        self.is_place_over = False  # 设置放置操作未完成
        rospy.loginfo("开始放置物品")
        # 创建放置位置的Pose消息
        place_pose_msg = Pose()
        place_pose_msg.position.x = req.x
        place_pose_msg.position.y = req.y
        place_pose_msg.position.z = req.z
        self.place_pub.publish(place_pose_msg)
        return PlaceResponse(
            message=f"开始放置物品"
        )
    
    # 抓取执行结果回调函数
    def cbPlaceResult(self, msg):
        rospy.logwarn("放置执行结果 = %s", msg.data)
        if str(msg.data) == "done":
            self.is_place_over = True

    def arm_zero(self, req):
        """
        机械臂收起的回调函数
        """
        # 如果之前有机械臂放置进程或机械臂收起进程在运行，则告知用户并返回
        if not self.is_place_over or (self.zero_process and self.zero_process.poll() is None):
            rospy.logwarn("机械臂放置或收起进程仍在运行，请稍后再试")
            return TriggerResponse(
                success=False,
                message="机械臂放置或收起进程仍在运行，请稍后再试"
            )
        
        # 如果之前有抓取进程在运行，则终止它
        if self.grab_process and self.grab_process.poll() is None:
            subprocess.Popen(self.kill_grab_node_command, shell=True)
            self.grab_process = None

        rospy.loginfo("机械臂收起")
        # 启动收起进程
        self.zero_process = subprocess.Popen(self.zero_command, shell=True)
        return TriggerResponse(
            success=True,
            message="正在机械臂收起"
        )
    
if __name__ == "__main__":
    rospy.init_node('arm_controller_server')
    arm_controller_server = ArmControllerServer()
    rospy.spin()
