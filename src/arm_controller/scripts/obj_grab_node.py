#!/usr/bin/python3

# coding=utf-8
import rospy
import threading
from wpb_home_behaviors.msg import Coord
from std_msgs.msg import String
from geometry_msgs.msg import Pose

# 标记变量，是否处于抓取过程当中
grabbing = False

# 标记变量，确定是否已经抓取成功
success = False

# 标记变量，确定是否仍在抓取
global_flag = 0
local_flag = 0

# 物品检测回调函数
def cbObject(msg):
    global grabbing
    if grabbing == False:
        num = len(msg.name)
        rospy.loginfo("检测物品个数 = %d", num)
        if num > 0:
            rospy.logwarn("抓取目标 %s! (%.2f , %.2f , %.2f)",msg.name[0],msg.x[0],msg.y[0],msg.z[0])
            grab_msg = Pose()
            grab_msg.position.x = msg.x[0]
            grab_msg.position.y = msg.y[0]
            grab_msg.position.z = msg.z[0]
            global grab_pub
            grab_pub.publish(grab_msg)
            grabbing = True
            # 已经获取物品坐标，停止检测
            behavior_msg = String()
            behavior_msg.data = "object_detect stop"
            behaviors_pub.publish(behavior_msg)

# 抓取执行结果回调函数
def cbGrabResult(msg):
    rospy.logwarn("抓取执行结果 = %s",msg.data)

    if str(msg.data) == "done":
        global success
        success = True
    else:
        global global_flag
        global_flag = global_flag + 1

class grabPauseErrorDetect(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.daemon = True  # 设置为守护线程
        self.start()  # 启动线程
    def run(self):
        global grabbing
        global global_flag
        global local_flag
        while True:
            rospy.sleep(15)
            if success:
                break
            elif global_flag == local_flag:
                rospy.logwarn("抓取错误，重新激活物品检测")
                grabbing = False

                # 发送消息，重新激活物品检测
                behavior_msg = String()
                behavior_msg.data = "object_detect start"
                behaviors_pub.publish(behavior_msg)
            else:
                local_flag = global_flag
                # 更新全局标记变量

# 主函数
if __name__ == "__main__":
    rospy.init_node("grab_node")
    # 发布物品检测激活话题
    behaviors_pub = rospy.Publisher("/wpb_home/behaviors", String, queue_size=10)
    # 发布抓取行为激活话题
    grab_pub = rospy.Publisher("/wpb_home/grab_action", Pose, queue_size=10)
    # 订阅物品检测结果的话题
    object_sub = rospy.Subscriber("/wpb_home/objects_3d", Coord, cbObject, queue_size=10)
    # 订阅抓取执行结果的话题
    result_sub = rospy.Subscriber("/wpb_home/grab_result", String, cbGrabResult, queue_size=10)
    # 延时三秒，让后台的话题初始化操作能够完成
    rospy.sleep(3.0)
    # 发送消息，激活物品检测行为
    msg = String()
    msg.data = "object_detect start"
    behaviors_pub.publish(msg)
    
    # 启动抓取错误检测线程
    grab_error_detect_thread = grabPauseErrorDetect()

    rospy.spin()