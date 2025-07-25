#!/usr/bin/python3

"""
本脚本实现了三个ROS服务，用于控制机械臂的抓取、放置和收起功能。需要注意的是，在启动这些服务前必须首先launch run_full.launch文件。
"""

import rospy
import subprocess
from arm_controller.srv import Place, PlaceResponse
from std_srvs.srv import Trigger, TriggerResponse, Empty
from std_msgs.msg import String, Bool
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose

class ArmControllerServer:
    def __init__(self):
        # 抓取服务
        self.grab_service = rospy.Service('/grab_service', Trigger, self.grab)

        # 放置服务
        self.place_service = rospy.Service('/place_service', Place, self.place)

        # 机械臂自然收起服务
        self.halt_service = rospy.Service('/arm_zero_service', Trigger, self.arm_zero)

        # 发布“放置是否结束”的话题
        self.place_over_pub = rospy.Publisher("/place_over", Bool, queue_size=10)

        # 机械臂紧急停止服务
        self.emergency_stop_service = rospy.Service('/arm_emergency_stop', Empty, self.arm_emergency_stop)

        rospy.loginfo("机械臂控制服务已启动")

        # 存储抓取指令
        self.grab_command = "rosrun arm_controller obj_grab_node.py"

        # 存储终止抓取节点的指令
        self.kill_grab_node_command = "rosnode kill grab_node"

        # 存储放置行为激活的指令
        self.place_action_command = "roslaunch ros_end_core place_action.launch"

        # 终止放置动作指令
        self.stop_place_action_command = "LAUNCH_FILE=\"place_action.launch\"\n \
            ROSLAUNCH_PID=$(ps aux | grep roslaunch | grep \"$LAUNCH_FILE\" | grep -v grep | awk '{print $2}')\n\
            [ -z \"$ROSLAUNCH_PID\" ] || kill -INT $ROSLAUNCH_PID"

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

        # 发布放置是否结束的话题
        rate = rospy.Rate(1)  # 1 Hz
        while not rospy.is_shutdown():
            self.publish_place_over()
            rate.sleep()

    def grab(self, req):
        """
        抓取服务的回调函数
        """
        # 如果之前有抓取、放置或机械臂收起进程在运行，则告知用户并返回
        if not self.is_place_over or (self.zero_process and self.zero_process.poll() is None) or (not self.is_grab_over()):
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
        if not self.is_grab_over():
            subprocess.Popen(self.kill_grab_node_command, shell=True)
            self.grab_process = None
        else:
            rospy.logwarn("没有抓取物品，无法进行放置操作")
            return PlaceResponse(
                message=f"没有抓取物品，无法进行放置操作"
            )

        self.is_place_over = False  # 设置放置操作未完成

        # 启动放置行为激活进程
        subprocess.Popen(self.place_action_command, shell=True)
        rospy.sleep(3.0)  # 等待放置行为激活完成

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
            # 杀死放置行为激活进程
            subprocess.Popen(self.stop_place_action_command, shell=True)

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
        if not self.is_grab_over():
            subprocess.Popen(self.kill_grab_node_command, shell=True)
            self.grab_process = None

        rospy.loginfo("机械臂收起")
        # 启动收起进程
        self.zero_process = subprocess.Popen(self.zero_command, shell=True)
        return TriggerResponse(
            success=True,
            message="正在机械臂收起"
        )
    
    def arm_emergency_stop(self, req):
        """
        机械臂紧急停止的回调函数
        """
        rospy.logwarn("机械臂紧急停止")

        # 如果有抓取进程在运行，则终止它
        if not self.is_grab_over():
            subprocess.Popen(self.kill_grab_node_command, shell=True)
            self.grab_process = None

        # 终止抓取动作
        # rospy.wait_for_service("/wpb_home/grab_halt")
        # try:
        #     grab_halt_service = rospy.ServiceProxy('/wpb_home/grab_halt', Empty)
        #     grab_halt_service()
        # except rospy.ServiceException as e:
        #     rospy.logerr("抓取停止服务调用失败: %s", e)
        
        # 如果有放置进程在运行，则终止它
        if not self.is_place_over:
            subprocess.Popen(self.stop_place_action_command, shell=True)
            self.is_place_over = True

        # 终止放置动作
        # rospy.wait_for_service("/wpb_home/place_halt")
        # try:
        #     place_halt_service = rospy.ServiceProxy('/wpb_home/place_halt', Empty)
        #     place_halt_service()
        # except rospy.ServiceException as e:
        #     rospy.logerr("放置停止服务调用失败: %s", e)
        
        # 停止机械臂
        count = 0
        while not rospy.is_shutdown():
            self.halt_arm()
            count += 1
            rospy.sleep(0.1)
            if count > 20:
                break
        
        return TriggerResponse(
            success=True,
            message="机械臂已紧急停止"
        )
    
    def halt_arm(self):
        # 发布机械臂控制话题
        mani_pub = rospy.Publisher("/wpb_home/mani_ctrl", JointState, queue_size=30)
        # 构造机械臂控制消息并进行赋值
        msg = JointState()
        msg.name = ['lift', 'gripper']
        msg.position = [ 0 , 0 ]
        msg.velocity = [ 0 , 0 ]
        rospy.loginfo("[mani_ctrl] Halt")
        mani_pub.publish(msg)

    def is_grab_over(self):
        """
        检查抓取是否结束
        """
        if self.grab_process and self.grab_process.poll() is None:
            return False
        return True

    def publish_place_over(self):
        """
        发布放置是否结束的话题
        """
        place_over_msg = Bool()
        place_over_msg.data = self.is_place_over
        self.place_over_pub.publish(place_over_msg)
    
if __name__ == "__main__":
    rospy.init_node('arm_controller_server')
    arm_controller_server = ArmControllerServer()
    rospy.spin()
