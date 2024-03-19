#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
import math
from std_msgs.msg import Float64

class OusterDataProcessor:
    def __init__(self):
        self.prev_odom_data = None

    def odom_callback(self, data):
        
        if self.prev_odom_data is None:
            self.prev_odom_data = data
            return

        # 현재 위치
        current_x = data.pose.pose.position.x
        current_y = data.pose.pose.position.y
        
        current_time = data.header.stamp
        
        # 이전 위치
        prev_x = self.prev_odom_data.pose.pose.position.x
        prev_y = self.prev_odom_data.pose.pose.position.y
        
        prev_time = self.prev_odom_data.header.stamp
        
        # 시간 차
        time_diff = (current_time - prev_time).to_sec()
        
        x_diff = current_x - prev_x
        y_diff = current_y - prev_y
        
        # 속도 계산 (단위: m/s)
        if time_diff != 0:
            x_speed = x_diff / time_diff
            y_speed = y_diff / time_diff

            speed = math.sqrt(x_speed**2 + y_speed**2)  # Overall speed

        else:
            x_speed = 0.0
            y_speed = 0.0
            speed = 0.0

        self.prev_odom_data = data

        odom_speed_msg = Float64()
        odom_speed_msg.data = speed
        speed_publisher.publish(odom_speed_msg)
        
if __name__ == '__main__':
    rospy.init_node('Ouster_speed_calculator')
    speed_publisher = rospy.Publisher('/odom_speed', Float64, queue_size=10)

    processor = OusterDataProcessor()
    rospy.Subscriber('/odom', Odometry, processor.odom_callback)
    rospy.Subscriber('/Odometry', Odometry, processor.odom_callback)
    rospy.spin()