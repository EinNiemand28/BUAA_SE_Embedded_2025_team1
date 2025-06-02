#!/usr/bin/python3

# coding=utf-8
import rospy
import threading
from wpb_home_behaviors.msg import Coord
from std_msgs.msg import String
from geometry_msgs.msg import Pose

# 抓取执行结果回调函数
def cbPlaceResult(msg):
    rospy.logwarn("放置执行结果 = %s",msg.data)

# 主函数
if __name__ == "__main__":
    rospy.init_node("place_node")
    # 发布放置行为激活话题
    place_pub = rospy.Publisher("/wpb_home/place_action", Pose, queue_size=10)
    # 订阅放置执行结果的话题
    result_sub = rospy.Subscriber("/wpb_home/place_result", String, cbPlaceResult, queue_size=10)
    # 延时三秒，让后台的话题初始化操作能够完成
    rospy.sleep(3.0)
    # 发送消息，激活物品放置行为
    msg = Pose()
    msg.position.x = 0.5  # 设置放置位置的x坐标
    msg.position.y = 0.5  # 设置放置位置的y坐标
    place_pub.publish(msg)

    rospy.spin()