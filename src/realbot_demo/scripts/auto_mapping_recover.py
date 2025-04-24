#!/usr/bin/python3
# coding=utf-8
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

class CollisionRecovery:
    """
    碰撞恢复类，实现在碰撞后自动后退和旋转，随后重新构建地图
    """
    def __init__(self):
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/collision_status', Bool, self.collision_cb)
        self.collision = False

    def collision_cb(self, msg):
        self.collision = msg.data
        if self.collision:
            self.recover()

    def recover(self):
        # 1. 后退
        back_cmd = Twist()
        back_cmd.linear.x = -0.2
        self.cmd_pub.publish(back_cmd)
        rospy.sleep(1.0)
        
        # 2. 小角度旋转
        rotate_cmd = Twist()
        rotate_cmd.angular.z = 0.5
        self.cmd_pub.publish(rotate_cmd)
        rospy.sleep(1.5)
        
        # 3. 停止并重置
        self.cmd_pub.publish(Twist())
        os.system("rosservice call /move_base/clear_costmaps")

if __name__ == '__main__':
    rospy.init_node('collision_recovery')
    cr = CollisionRecovery()
    rospy.spin()