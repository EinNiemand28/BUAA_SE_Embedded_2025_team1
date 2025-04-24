#!/usr/bin/python3
# coding=utf-8

import time
import rospy  
import _thread
import actionlib  
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal  
from geometry_msgs.msg import Twist  
from std_msgs.msg import String
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan

from navigation.msg import *

# 设置速度阈值  
LINEAR_THRESHOLD = 0.00001  # 例如，0.1 m/s  
ANGULAR_THRESHOLD = 0.00001  

# 记录上次进度
last_percentage = 0

# 状态转移: 
#           planning -> normal
#           normal   -> barrier
#           barrier  -> planning
#           normal   -> reach
#           planning -> fail 
# 重要的状态变量：
#           plan_init   路径规划完成标志
#           radar_obstacle_detect 雷达检测动态避障
#           reach_flag  导航结束标志 
#           fail        路径规划失败标志      
#           last_per    上次进度

class SimpleGoalServer:
    def __init__(self):
        self.ac = actionlib.SimpleActionClient('move_base', MoveBaseAction) # simple_goal是move_base的client
        self.cur_goal = None 
        self.global_plan = None # 存路径

        # 状态
        self.reach_flag = True
        self.fail = False
        self.radar_obstacle_detect = None # 雷达检测动态避障
        self.cur_pos_id = 0 # 存当前进度
        self.plan_init = False # 全局路径是否规划完成

        # feedback和result
        self.feedback = NavigateFeedback()
        self.result = NavigateResult()

        # simple_goal是三个服务层节点的server
        self.server = actionlib.SimpleActionServer("/navigation/navigate",NavigateAction,self.simple_goal_cb,False)
        self.server.start()

        # 监听global_planner
        self.gp_sub = None
        rospy.loginfo("导航启动")
        self.gp_sub = rospy.Subscriber('/move_base/GlobalPlanner/plan',Path, self.updata_global_plan)

    def clear_state(self):
        self.cur_goal = None
        self.reach_flag = True
        self.fail = False
        self.global_plan = None 
        self.cur_pos_id = 0 
        self.plan_init = False

    def updata_global_plan(self, msg):
        p_list = msg.poses # geometry_msgs/PoseStamped[]
        self.global_plan = []
        self.cur_pos_id = 0
        for p in p_list: # geometry_msgs/PoseStamped
            pose = p.pose # geometry_msgs/Pose
            x = pose.position.x
            y = pose.position.y
            z = pose.orientation.z
            self.global_plan.append((x,y,z))
        if len(self.global_plan) == 0:
            return
        self.plan_init = True

    def move_base_done_cb(self,state,result):
        if state == actionlib.GoalStatus.SUCCEEDED:
            self.reach_flag = True
        else :
            self.reach_flag = True
            self.fail = True

    def move_base_active_cb(self):
        rospy.loginfo("move_base服务被激活....")


    def move_base_fb_cb(self,fb):
        if self.plan_init == False:
            self.feedback.cur_state = "planning"
            self.feedback.percentage = 0
            self.server.publish_feedback(self.feedback) 
            return
        self.feedback.cur_state = "normal"
        x = fb.base_position.pose.position.x
        y = fb.base_position.pose.position.y 
        z = fb.base_position.pose.orientation.z
        # 计算进度
        min_dis = None
        tar = None
        for i in range(self.cur_pos_id,len(self.global_plan)):
            dis = pow((x-self.global_plan[i][0]),2) + pow(y-self.global_plan[i][1],2) + 0.1*abs(z-self.global_plan[i][2])
            if tar == None or dis < min_dis:
                min_dis = dis
                tar = i
                
        
        self.cur_pos_id = tar
        self.feedback.percentage = int(100 * float(self.cur_pos_id + 1) / len(self.global_plan))
        self.server.publish_feedback(self.feedback) 
        
    # server的回调函数
    def simple_goal_cb(self,goal):
        # simple_goal作为move_base的client的回调函数
        
        # 初始化状态
        self.clear_state()
        # 把goal传递给move_base
        self.cur_goal = MoveBaseGoal()  
        self.cur_goal.target_pose.header.frame_id = "map"  
        self.cur_goal.target_pose.pose.position.x = goal.pos.position.x
        self.cur_goal.target_pose.pose.position.y = goal.pos.position.y
        self.cur_goal.target_pose.pose.position.z = 0 
        self.cur_goal.target_pose.pose.orientation.z = goal.pos.orientation.z
        self.cur_goal.target_pose.pose.orientation.w = goal.pos.orientation.w
        # 监听global_planner
        
        self.ac.send_goal(self.cur_goal,self.move_base_done_cb,self.move_base_active_cb,self.move_base_fb_cb)  
        self.reach_flag = False
        self.ac.wait_for_server()         
        rospy.loginfo("开始导航……")    
        # 检测前方障碍，实现动态避障
        self.radar_obstacle_detect = rospy.Subscriber("scan", LaserScan, self.obstacle_detect)

        # 开始监听cancel消息和move_base action的result
        r = rospy.Rate(1)  
        while not rospy.is_shutdown():
            if self.server.is_preempt_requested(): # cancel了
                self.ac.cancel_goal()
                self.result.result = "cancel"
                self.server.set_succeeded(self.result)
                break
            if self.fail == True: # move_base导航失败
                self.result.result = "fail"
                self.server.set_succeeded(self.result)
            if self.reach_flag == True: # 导航成功并到达
                self.result.result = "success"
                self.server.set_succeeded(self.result)
                break
            r.sleep()
        self.clear_state()
        rospy.loginfo("导航结束！")

    def renavigate(self):
        self.plan_init = False  
        self.global_plan = None
        self.cur_pos_id = 0
        rospy.logwarn("机器人遇到障碍，正在重新设置航点，新路径计算中...") 
        self.feedback.cur_state = "barrier"
        self.feedback.percentage = 0
        self.last_per = 0
        self.server.publish_feedback(self.feedback) 
        
        self.ac.cancel_goal()  # 取消当前目标
        self.ac.send_goal(self.cur_goal, self.move_base_done_cb, self.move_base_active_cb, self.move_base_fb_cb)

    # 监听进度，实现动态避障
    def obstacle_detect(self, msg):
        if (self.reach_flag == False and msg.ranges[180] < 0.5):
            self.renavigate()
            rospy.sleep(1)
    

rospy.init_node("simple_goal")  
server = SimpleGoalServer()
rospy.spin()






