#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Twist
    

def get_twist(ctrl_msg):
    twist = Twist()
    if ctrl_msg == 'w':
        twist.linear.x = 0.5
    elif ctrl_msg == 's':
        twist.linear.x = -0.5
    elif ctrl_msg == 'a':
        twist.linear.y = 0.5
    elif ctrl_msg == 'd':
        twist.linear.y = -0.5
    elif ctrl_msg == ' ':
        twist.linear.z = 0.5
    elif ctrl_msg == 'shift':
        twist.linear.z = -0.5
    return twist

if __name__ == '__main__':
    rospy.init_node('vel_ctrl_node')
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        ctrl_msg = input()
        twist = get_twist(ctrl_msg)
        rospy.loginfo('Linear: %f, Angular: %f' % (twist.linear.x, twist.angular.z))
        pub.publish(twist)
        rate.sleep()