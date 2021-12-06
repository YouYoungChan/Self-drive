#!/home/pi/.pyenv/versions/rospy3/bin/python3


import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np

def avg_range(ranges):
    range_arr = np.array(ranges)
    p_10 = range_arr[0:10]
    m_10 = range_arr[-10: ]
    p_30 = range_arr[10:30]
    m_30 = range_arr[-30:-10]
    p_70 = range_arr[30:70]
    m_70 = range_arr[-70:-30]
    p_10_real = p_10[p_10 > 0]
    m_10_real = m_10[m_10 > 0]
    p_30_real = p_30[p_30 > 0]
    m_30_real = m_30[m_30 > 0]
    p_70_real = p_70[p_70 > 0]
    m_70_real = m_70[m_70 > 0]
    p_10_mean = np.mean(p_10_real)
    m_10_mean = np.mean(m_10_real)
    p_30_mean = np.mean(p_30_real)
    m_30_mean = np.mean(m_30_real)
    p_70_mean = np.mean(p_70_real)
    m_70_mean = np.mean(m_70_real)
#    print("P10 : ",p_10_mean, "\nM10 : ",m_10_real,"\np30 : ", p_30_mean, "\nM30 : ", m_30_mean , "\np70 : ", p_70_mean, "\nM70 : ", m_70_mean )
    return p_10_mean,m_10_mean,p_30_mean,m_30_mean,p_70_mean,m_70_mean

class SelfDrive:
    def __init__(self, publisher):
        self.publisher = publisher
        self.count = 30
  
    def lds_callback(self, scan):
        # scan 분석 후 속도 결정
        # ...
        turtle_vel = Twist()
         # 전진 속도 및 회전 속도 지정
        p_10_avg, m_10_avg, p30_avg, m30_avg, p70_avg, m70_avg = avg_range(scan.ranges)

        if p_10_avg < 0.25 or m_10_avg < 0.25:
            turtle_vel.linear.x = 0.03
            turtle_vel.angular.z = 2        

        elif p30_avg < 0.30 or m30_avg < 0.30:
            turtle_vel.linear.x = 0
            if p30_avg >= m30_avg:
                turtle_vel.angular.z = 2.0
            else:
                turtle_vel.angular.z = -2.0

        elif p70_avg < 0.16 or m70_avg < 0.16:
            turtle_vel.linear.x = 0.05
            if p70_avg >= m70_avg:
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


