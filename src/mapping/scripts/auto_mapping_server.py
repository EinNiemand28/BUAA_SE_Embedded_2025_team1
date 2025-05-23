#!/usr/bin/python3

"""
本脚本实现了一个ROS服务，用于完整提供自动建图的服务，包括临时终止自动建图。
"""

import rospy
import actionlib
from mapping.srv import Start, StartResponse, Halt, HaltResponse
from navigation.msg import *
from geometry_msgs.msg import Twist
import subprocess

class AutoMappingServer:
    def __init__(self):
        # 启动自动建图的服务
        self.start_service = rospy.Service('/auto_mapping_service', Start, self.start_mapping)
        rospy.loginfo("自动建图服务已就绪")

        # 终止自动建图的服务
        self.halt_service = rospy.Service('/halt_auto_mapping', Halt, self.halt_auto_mapping)
        
        # 存储启动建图服务器指令
        self.server_command = None
        
        # 存储探索指令
        self.explore_command = None

        # 存储保存地图指令
        self.save_map_command = None

        # 给每个保存的地图编号，保证地图的名称独一无二 TODO
        self.map_no = 0

        # 终止自动建图指令
        self.stop_command = "rosnode kill /explore\n \
            LAUNCH_FILE=\"run_mapping.launch\"\n \
            ROSLAUNCH_PID=$(ps aux | grep roslaunch | grep \"$LAUNCH_FILE\" | grep -v grep | awk '{print $2}')\n\
            [ -z \"$ROSLAUNCH_PID\" ] || kill -INT $ROSLAUNCH_PID"
        
        # 尝试在建图结束后回到起点
        ## action client对象
        self.action_client = actionlib.SimpleActionClient("/navigation/navigate", NavigateAction)

        self.return_to_init = NavigateGoal()
        self.return_to_init.pos.position.x = 0
        self.return_to_init.pos.position.y = 0
        self.return_to_init.pos.orientation.z = 0
        self.return_to_init.pos.orientation.w = 1.0

        # 存储建图服务器进程
        self.server_process = None

        # 存储探索进程
        self.explore_process = None

        # 存储地图保存进程
        self.save_map_process = None

        # 杀死进程
        self.kill_process = None

    def start_mapping(self, req):
        try:
            if self.is_mapping_now():
                return StartResponse(
                    success=False,
                    message=f"已在自动建图中"
                )
            
            # 准备启动建图服务器指令
            if req.sim:
                self.server_command = "roslaunch task_manager_sim run_mapping.launch"  # 启动仿真条件下的自动建图
            else:
                self.server_command = "roslaunch task_manager run_mapping.launch" # 启动真实环境中的自动建图 TODO
            
            # 准备探索指令
            self.explore_command = \
                        "rosparam set /move_base/global_costmap/width " + str(req.width) + "\n" + \
                        "rosparam set /move_base/global_costmap/height " + str(req.height) + "\n" + \
                        "rosrun explore_lite explore _robot_base_frame:=base_link \\\n\
                          _costmap_topic:=/map \\\n\
                          _planner_frequency:=1.0 \\\n\
                          _progress_timeout:=60.0 \\\n\
                          _potential_scale:=10.0 \\\n\
                          _min_frontier_size:=0.5 \\\n\
                          _visualize:=true"
            
            # 启动服务器节点
            self.server_process = subprocess.Popen(self.server_command, shell=True)
            rospy.loginfo(f"已启动建图服务器: {self.server_command}" + str(self.server_process.pid))

            # 启动探索进程
            self.explore_process = subprocess.Popen(self.explore_command, shell=True)
            rospy.loginfo(f"已启动自动建图: {self.explore_command}" + str(self.explore_process.pid))

            return StartResponse(
                success=True,
                message=f"已开始自动建图！"
            )
            
        except Exception as e:
            rospy.logerr(f"启动自动建图失败: {str(e)}")
            return StartResponse(
                success=False,
                message=f"启动自动建图失败: {str(e)}"
            )
        
    def is_mapping_now(self):
        return (self.server_process and self.server_process.poll() is None) or (self.explore_process and self.explore_process.poll() is None)

    def halt_auto_mapping(self, req):
        try:
            if not self.is_mapping_now():
                return HaltResponse(
                    success=False,
                    message=f"没有正在进行中的自动建图"
                )
            
            # 保存建得的地图
            self.map_no = self.map_no + 1
            self.save_map_command = "rosrun map_server map_saver -f " + str(self.map_no) # TODO
            self.save_map_process = subprocess.Popen(self.save_map_command, shell=True)
            
            # 终止自动建图
            while self.save_map_process.poll() is None:
                rospy.loginfo("正在保存地图，请稍等...")
                rospy.sleep(1)
            self.kill_process = subprocess.Popen(self.stop_command, shell=True)
            rospy.loginfo(f"建图服务已终止: {self.server_command}")

            # 首先让机器人原地静止
            movement_topic = rospy.Publisher('cmd_vel', Twist, queue_size=10)
            twist = Twist()
            twist.linear.x = 0
            twist.linear.y = 0
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = 0
            while self.is_mapping_now():
                rospy.sleep(1)
            movement_topic.publish(twist)

            # 让机器人回到原点
            self.action_client.wait_for_server()
            self.action_client.send_goal_and_wait(self.return_to_init)

            return HaltResponse(
                success=True,
                message=f"建图服务已终止",
                path=str(self.map_no)
            )
            
        except Exception as e:
            rospy.logerr(f"终止自动建图失败: {str(e)}")
            return HaltResponse(
                success=False,
                message=f"终止自动建图失败: {str(e)}"
            )

if __name__ == "__main__":
    rospy.init_node('auto_mapping_server')
    starter = AutoMappingServer()
    rospy.spin()