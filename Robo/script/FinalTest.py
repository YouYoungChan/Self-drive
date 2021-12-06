#!/home/pi/.pyenv/versions/rospy3/bin/python3


import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np

def avg_range(ranges):
    range_arr = np.array(ranges)
    p_40 = range_arr[0:40]
    m_40 = range_arr[-40: ]
    p_80 = range_arr[40:80]
    m_80 = range_arr[-80:-40]
    p_40_real = p_arr[p_arr > 0]
    m_40_real = m_arr[m_arr > 0]
    p_80_real = p_80[p_80 > 0]
    m_80_real = m_80[m_80 > 0]
    p_40_mean = np.mean(p_arr_real)
    m_80_mean = np.mean(m_arr_real)
    p_80_mean = np.mean(p_80_real)
    m_80_mean = np.mean(m_80_real)
    print("p40 : ", p_40_mean, "\n M40 : ", m_40_mean , "\n p80 : ", p_80_mean, "\n M80 : ", m_80_mean )
    return p_40_mean,m_40_mean,p_80_mean,m_80_mean

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
        plus_avg, minus_avg, p_80, m_80 = avg_range(scan.ranges)

        if p_80 < 0.30 or m_80 < 0.30:
            if p_80 >= m_80:
                turtle_vel.angular.z = 2.0
            else:
                turtle_vel.angular.z = -2.0

        if plus_avg < 0.30 or minus_avg < 0.30:
            turtle_vel.linear.x = 0
            if plus_avg >= minus_avg:
                turtle_vel.angular.z = 2.0
            else:
                turtle_vel.angular.z = -2.0
        else:
            turtle_vel.linear.x = 0.15
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


