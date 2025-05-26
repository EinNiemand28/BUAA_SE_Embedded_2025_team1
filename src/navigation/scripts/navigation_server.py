#!/usr/bin/python3

"""
本脚本实现了一个ROS服务，用于在前端发出导航指令时，这里启动run_full.launch、simple_goal.py节点和goal_server.py节点。
"""

import rospy
from navigation.srv import Start, StartResponse, Halt, HaltResponse
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist
import subprocess

class NavigationServer:
    def __init__(self):
        # 启动导航服务
        self.navigation_service = rospy.Service('/navigation_service', Start, self.start_navigation)
        rospy.loginfo("导航服务已就绪")

        # 终止导航服务
        self.stop_service = rospy.Service('/halt_navigation', Halt, self.halt_navigation)

        # 启动导航服务器指令
        self.server_command = None

        # 指定地图指令
        self.map_command = "rosrun map_server map_server /media/mitchell/Data/Projects/software_engineering/ros_end/src/mapping/maps/"

        # 启动目标服务器指令
        self.goal_server_command = "rosrun navigation goal_server.py"

        # 终止导航系统指令
        self.stop_command = "rosnode kill map_server\n \
            rosnode kill goal_server\n \
            LAUNCH_FILE=\"run_full.launch\"\n \
            ROSLAUNCH_PID=$(ps aux | grep roslaunch | grep \"$LAUNCH_FILE\" | grep -v grep | awk '{print $2}')\n\
            [ -z \"$ROSLAUNCH_PID\" ] || kill -INT $ROSLAUNCH_PID"

        # 存储导航进程
        self.navigation_process = None

        # 存储地图进程
        self.map_process = None

        # 存储目标服务器进程
        self.goal_server_process = None

        # 杀死进程
        self.kill_process = None

    def start_navigation(self, req):
        try:
            if self.is_navigation_active():
                return StartResponse(
                    success=False,
                    message=f"导航模块已启动"
                )
            
            # 准备导航模块启动指令
            if req.sim:
                self.server_command = "roslaunch ros_end_core_sim run_full.launch"
            else:
                self.server_command = "roslaunch task_manager run_full.launch"
                rospy.logerr_once("wrong!")

            # 如果需要指定地图，则添加地图指令
            if req.map:
                map_command = self.map_command + req.path + req.name + ".yaml" # 添加地图指令
            else:
                map_command = self.map_command + "map.yaml"
                rospy.logwarn("未指定地图，将使用默认地图")

            # 加载导航服务环境
            self.navigation_process = subprocess.Popen(self.server_command, shell=True)

            # 启动地图环境
            self.map_process = subprocess.Popen(map_command, shell=True)

            # 启动导航目的地服务器(goal server)节点
            self.goal_server_process = subprocess.Popen(self.goal_server_command, shell=True)

            # rospy.sleep(3)

            # 矫正机器人的位置
            subprocess.Popen("rosrun mapping position_correction.py __name:=my_position_correction_node", shell=True)
            rospy.sleep(3)
            subprocess.call("rosnode kill my_position_correction_node", shell=True)

            return StartResponse(
                success=True,
                message=f"导航模块已启动！"
            )

        except Exception as e:
            rospy.logerr(f"启动导航服务失败: {str(e)}")
            return StartResponse(
                success=False,
                message=f"启动导航服务失败: {str(e)}"
            )
        
    def is_navigation_active(self):
        return (self.navigation_process and self.navigation_process.poll() is None) or (self.goal_server_process and self.goal_server_process.poll() is None)
    
    def halt_navigation(self, req):
        try:
            if not self.is_navigation_active:
                return HaltResponse(
                    success=False,
                    message=f"导航模块尚未启动"
                )
            
            # 首先终止正在进行的导航任务
            try:
                rospy.wait_for_service('/halt_goal')
                halt_goal = rospy.ServiceProxy('/halt_goal', Trigger)
                halt_goal()
            except Exception as e:
                rospy.logwarn(f"导航任务未能正常结束，正在强制退出..." + str(e))
                self.keep_robot_static()
        
            # 终止导航模块
            self.kill_process = subprocess.Popen(self.stop_command, shell=True)
            rospy.loginfo(f"导航模块已终止: {self.stop_command}")

            return HaltResponse(
                success=True,
                message=f"导航模块已终止"
            )

        except Exception as e:
            rospy.logerr(f"导航模块终止失败: {str(e)}")
            return HaltResponse(
                success=False,
                message=f"导航模块终止失败: {str(e)}"
            )
        
    def keep_robot_static(self):
        movement_topic = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        movement_topic.publish(twist)

if __name__ == "__main__":
    rospy.init_node('navigation_server')
    starter = NavigationServer()
    rospy.spin()