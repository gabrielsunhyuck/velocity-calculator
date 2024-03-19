#!/usr/bin/env python3

import pyproj

import rospy
import utm
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import NavSatFix

def navsatfix_callback(data):
    latitude = data.latitude
    longitude = data.longitude
    easting, northing, zone_number, zone_letter = utm.from_latlon(latitude, longitude)

    
    #rospy.loginfo(f"Latitude: {latitude}, Longitude: {longitude}")
    rospy.loginfo(f"UTM_X : {easting}, UTM_Y : {northing}, Zone : {zone_number}{zone_letter}")

    utm_msg = Vector3()
    utm_msg.x = easting
    utm_msg.y = northing
    utm_msg.z = float(str(zone_number)+str(ord(zone_letter)))
    
    utm_publisher.publish(utm_msg)
    
    
def listener():
    rospy.init_node('utm_converter', anonymous=False)
    
    rospy.Subscriber('/ublox_gps/fix', NavSatFix, navsatfix_callback)
    rospy.spin()

if __name__ == '__main__':
    
    utm_publisher = rospy.Publisher("/utm", Vector3, queue_size=10)
    listener()