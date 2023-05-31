#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point , PointStamped
import ros_numpy
import numpy as np

if __name__=="__main__":
	rospy.init_node('send_xyz')
	data_xy  = rospy.wait_for_message('aruco_single/pixel' , PointStamped)
	data = Point()
	data_depth = rospy.wait_for_message('kinect2/hd/points' , PointCloud2)
	data_np = ros_numpy.numpify(data_depth)
	data.x = data_np['x'][int(data_xy.point.y)][int(data_xy.point.x)]
	data.y = data_np['y'][int(data_xy.point.y)][int(data_xy.point.x)]
	print(data_xy.point.x)
	#data.x = data_xy.point.x
	#data.y = data_xy.point.y
	data.z = data_np['z'][int(data_xy.point.y)][int(data_xy.point.x)]
	print(data.z)	
	pub = rospy.Publisher('xyz_of_robot_from_cam' , Point , queue_size=10)	
	rate = rospy.Rate(10)
	print(data.z)
	while not rospy.is_shutdown():
		pub.publish(data)
		rate.sleep()


