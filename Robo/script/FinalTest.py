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

        for i in range(30):
            if scan.ranges[i] <= 0.25 or scan.ranges[-i] <= 0.25:
                turtle_vel.linear.x = 0.01
                turtle_vel.linear.z = 0.1
            else:
                turtle_vel.linear.x = 0.15
                turtle_vel.linear.z = 0.0
	
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


