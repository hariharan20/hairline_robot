#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from geometry_msgs.msg import Point

if __name__=="__main__":
	rospy.init_node('aruco_detect')
	data = rospy.wait_for_message('kinect2/hd/image_color' , Image)
	bridge = CvBridge()
	img = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
	arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL)
	arucoParams = cv2.aruco.DetectorParameters_create()
	(corners, ids, rejected) = cv2.aruco.detectMarkers(img, arucoDict,parameters=arucoParams)
	data_p = Point()
	print(corners)
	data_p.x  = int(sum(corners[0][0][: , 0])/4)
	data_p.y = int(sum(corners[0][0][: , 1]) / 4)
	pub = rospy.Publisher('aruco_coordinates', Point, queue_size=10)
	rate = rospy.Rate(10)	
	while not rospy.is_shutdown():
		pub.publish(data_p)
		rate.sleep()
