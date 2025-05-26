#!/usr/bin/python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

if __name__ == '__main__':
    rospy.init_node('position_correction_node')

    # 发布修正后的位姿
    corrected_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)

    rospy.loginfo("正在启动位置修正节点...")

    # 订阅里程计数据
    def odometry_callback(msg):
        corrected_pose = PoseWithCovarianceStamped()
        corrected_pose.header = msg.header
        corrected_pose.header.frame_id = 'map'
        corrected_pose.pose.pose = msg.pose.pose
        corrected_pose.pose.covariance = msg.pose.covariance

        # 发布修正后的位姿
        corrected_pose_pub.publish(corrected_pose)

    rospy.Subscriber('/odom', Odometry, odometry_callback)

    rospy.loginfo("Position correction node started.")
    rospy.spin()