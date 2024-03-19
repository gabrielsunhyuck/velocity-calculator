#!/usr/bin/env python3

import pyproj

import rospy
import utm
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry

class UtmConverter:
    def __init__(self):
        rospy.init_node('gps_converter', anonymous=False)
        
        self.utm_publisher = rospy.Publisher("/gps_odometry", Odometry, queue_size=10)
        
        self.prev_easting = None
        self.prev_northing = None
        self.prev_time = None
        
        rospy.Subscriber('/ublox_gps/fix', NavSatFix, self.navsatfix_callback)
        
    def navsatfix_callback(self, data):
        latitude = data.latitude
        longitude = data.longitude
        easting, northing, _, _ = utm.from_latlon(latitude, longitude)
        
        current_time = data.header.stamp
        
        if self.prev_easting is not None and self.prev_northing is not None and self.prev_time is not None:
            dt = (current_time - self.prev_time).to_sec()
            
            d_easting = easting - self.prev_easting
            d_northing = northing - self.prev_northing
            dist = (d_easting**2 + d_northing**2)**0.5
            
            speed = dist / dt  # Calculate speed as distance divided by time
            print(easting,northing,speed)
            odometry = Odometry()
            odometry.header.stamp = current_time
            odometry.child_frame_id = 'gps_baselink'
            
            odometry.pose.pose.position.x = easting
            odometry.pose.pose.position.y = northing
            odometry.pose.pose.position.z = 0.0
            
            odometry.twist.twist.linear.x = speed
            
            self.utm_publisher.publish(odometry)
            
        self.prev_easting = easting
        self.prev_northing = northing
        self.prev_time = current_time

if __name__ == '__main__':
    try:
        utm_converter = UtmConverter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
