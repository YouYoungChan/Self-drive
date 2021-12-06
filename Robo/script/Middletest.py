#!/home/pi/.pyenv/shims/python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class SelfDrive:
    def __init__(self, publisher):
        self.publisher = publisher
        self.count = 30

    def lds_callback(self, scan):
        # scan 분석 후 속도 결정
        # ...
        print("scan[0]:", scan.ranges[0])
        turtle_vel = Twist()
         # 전진 속도 및 회전 속도 지정
        if scan.ranges[0] > 0.15 or scan.ranges[0] == 0.0:
            turtle_vel.linear.x = 0.1
        else:
            turtle_vel.linear.x = 0.0
        turtle_vel.angular.z = 0.0
         # 속도 출력
        self.publisher.publish(turtle_vel)

def main():
    rospy.init_node('self_drive')
    publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    driver = SelfDrive(publisher)
    subscriber = rospy.Subscriber('scan', LaserScan,
                                  lambda scan: driver.lds_callback(scan))
    rospy.spin()

if __name__ == "__main__":
    main()


