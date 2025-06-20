#!/usr/bin/python3

import rospy
from std_msgs.msg import Bool
from fetch_server.srv import Fetch, FetchResponse
from navigation.srv import Start, StartResponse, Halt, HaltResponse
from navigation.srv import Goal, GoalResponse
from std_srvs.srv import Trigger, TriggerResponse
from arm_controller.srv import Place, PlaceResponse

class FetchServer:
    def __init__(self):
        # 启动抓取服务
        self.fetch_service = rospy.Service('/fetch_service', Fetch, self.handle_fetch)
        rospy.loginfo("抓取服务已就绪")

        # 订阅抓取完成的消息
        self.grab_over_sub = rospy.Subscriber("/grab_over", Bool, self.handle_grab_over, queue_size=10)

        # 订阅放置完成的消息
        self.place_over_sub = rospy.Subscriber("/place_over", Bool, self.handle_place_over, queue_size=10)

        # 初始化抓取和放置状态
        self.grab_over = False
        self.place_over = False

    def handle_grab_over(self, msg):
        # 处理抓取完成的消息
        self.grab_over = msg.data

    def handle_place_over(self, msg):
        # 处理放置完成的消息
        self.place_over = msg.data

    def handle_fetch(self, req):
        # 处理抓取请求
        rospy.loginfo(f"接收到抓取请求")

        # 初始化抓取和放置状态
        self.grab_over = False
        self.place_over = False
        
        # 若导航模块尚未启动，则首先启动导航模块
        try:
            rospy.wait_for_service('/navigation_service')
            start_navi = rospy.ServiceProxy('/navigation_service', Start)
            start_navi(sim = True, map = False, path = "", name = "")
        except Exception as e:
            rospy.loginfo(str(e))
            
        # 计算抓取时机器人应前往的位置
        grab_robot_destination = self.calculate_destination(req.gpx, req.gpy, req.goz)

        # 发布导航目标
        try:
            rospy.wait_for_service('/goal_service')
            navi_goal = rospy.ServiceProxy('/goal_service', Goal)
            navi_result = navi_goal(px=grab_robot_destination[0], py=grab_robot_destination[1], oz=grab_robot_destination[2])
            if not navi_result.success:
                return FetchResponse(success=False, message="导航失败")
        except Exception as e:
            return FetchResponse(success=False, message=f"导航服务调用失败: {str(e)}")
        
        # 执行抓取动作
        try:
            rospy.wait_for_service('/grab_service')
            grab_service = rospy.ServiceProxy('/grab_service', Trigger)
            grab_service()
        except Exception as e:
            return FetchResponse(success=False, message=f"抓取服务调用失败: {str(e)}")
        
        # 等待抓取完成
        rospy.loginfo("等待抓取完成...")
        rospy.sleep(3)
        while not self.grab_over:
            rospy.sleep(1)
        
        # 计算放置时机器人应前往的位置
        place_robot_destination = self.calculate_destination(req.ppx, req.ppy, req.poz)

        # 发布导航目标
        try:
            rospy.wait_for_service('/goal_service')
            navi_goal = rospy.ServiceProxy('/goal_service', Goal)
            navi_result = navi_goal(px=place_robot_destination[0], py=place_robot_destination[1], oz=place_robot_destination[2])
            if not navi_result.success:
                return FetchResponse(success=False, message="导航失败")
        except Exception as e:
            return FetchResponse(success=False, message=f"导航服务调用失败: {str(e)}")
        
        # 执行放置动作
        ## 设定放置位置
        place_position = self.calculate_place_position(req.ppx, req.ppy, req.ppz, req.poz, place_robot_destination)
        rospy.logerr(f"放置位置: x={place_position[0]}, y={place_position[1]}, z={place_position[1]}")

        ## 调用放置服务
        try:
            rospy.wait_for_service('/place_service')
            place_service = rospy.ServiceProxy('/place_service', Place)
            place_service(x = place_position[0], y = place_position[1], z = place_position[2])
        except Exception as e:
            return FetchResponse(success=False, message=f"放置服务调用失败: {str(e)}")
        
        # 等待放置完成
        rospy.loginfo("等待放置完成...")
        rospy.sleep(3)
        while not self.place_over:
            rospy.sleep(1)

        # 收起机械臂
        try:
            rospy.wait_for_service('/arm_zero_service')
            arm_zero = rospy.ServiceProxy('/arm_zero_service', Trigger)
            arm_zero()
        except Exception as e:
            return FetchResponse(success=False, message=f"机械臂收起服务调用失败: {str(e)}")
        
        return FetchResponse(success=True, message=f"成功抓取并放置物体")
    
    # 计算应该前往的位置
    def calculate_destination(self, px, py, oz):
        if abs(oz + 1.57) < 0.1:
            return (px, py - 1.23, 1.0)
        elif abs(oz) < 0.1:
            return (px + 1.23, py - 0.5, 4.0)
        elif abs(oz - 3.14) < 0.1:
            return (px - 1.23, py, 0.00)
        else:
            return (px, py + 1.23, -1.57)
    
    # 计算放置物品的位置
    def calculate_place_position(self, x, y, z, oz, place_robot_destination):
        if abs(oz + 1.57) < 0.1:
            place_position_x = y - place_robot_destination[1] - 0.1
            place_position_y = x - place_robot_destination[0] - 0.1
            place_position_z = z
        elif abs(oz) < 0.1:
            place_position_x =  - x + place_robot_destination[0]
            place_position_y = - y + place_robot_destination[1] + 0.4
            place_position_z = z
        elif abs(oz - 3.14) < 0.1:
            place_position_x = x - place_robot_destination[0]
            place_position_y = y - place_robot_destination[1]
            place_position_z = z
        return (place_position_x, place_position_y, place_position_z)
    
if __name__ == "__main__":
    rospy.init_node('fetch_server')
    fetch_server = FetchServer()
    rospy.spin()