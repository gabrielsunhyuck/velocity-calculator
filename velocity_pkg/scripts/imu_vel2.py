#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
import math

lin_acc_x=[]
lin_acc_y=[]
time_diff = 0.0
loop = True

class ImuProcessor:
    def __init__(self, g=0.98):
        self.prev_time = None
        self.x_speed = 0.0
        self.y_speed = 0.0
        self.g = g  
        self.speed_publisher = rospy.Publisher('/imu_speed', Float64, queue_size=10)

    def imu_callback(self, data):
        #current_time = data.header.stamp
        lin_acc_x.append(data.linear_acceleration.x)
        lin_acc_y.append(data.linear_acceleration.y)
        
        global time_diff
        
        #if self.prev_time is not None:
        while loop:
        # 적분 보정
            self.x_speed = self.x_speed + sum(lin_acc_x) * self.g * time_diff
            self.y_speed = self.y_speed + sum(lin_acc_y) * self.g * time_diff
            time_diff+=1
            self.x_speed = self.x_speed / time_diff
            self.y_speed = self.y_speed / time_diff
            
            x_speed = math.log10(abs(self.x_speed))
            y_speed = math.log10(abs(self.y_speed))
            
            imu_speed = ((x_speed)**2 + (y_speed)**2)**0.5
            rospy.loginfo(f"imu_speed: {imu_speed} m/s")
            speed_msg = Float64()
            speed_msg.data = imu_speed
            self.speed_publisher.publish(speed_msg)

        # 이전 데이터 업데이트

if __name__ == '__main__':
    rospy.init_node('imu_speed_calculator')

    imu_processor = ImuProcessor()
    rospy.Subscriber('/nav/filtered_imu/data', Imu, imu_processor.imu_callback)

    rospy.spin()