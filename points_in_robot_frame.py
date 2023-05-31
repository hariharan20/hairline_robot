#!/usr/bin/env python

import rospy
import  tf
import numpy as np
from sensor_msgs.msg import PointCloud2
import ros_numpy 
import tf2_ros
from geometry_msgs.msg import PointStamped
rospy.init_node('points_in_robot_frame')
data = rospy.wait_for_message('hair_line_pointcloud' , PointCloud2)
data_np = ros_numpy.numpify(data)
l = tf.TransformListener()
l.waitForTransform('kinect2_link', 'joint1' , rospy.Time(0) , rospy.Duration(4.0))
points = np.zeros((len(data_np) , 3))
points = np.zeros(len(data_np) , dtype=[('x' , np.float32 ) , ('y' , np.float32) , ('z' , np.float32)])
print(len(data_np))
for i in range(len(data_np)):
	point = PointStamped()
	point.header.frame_id='kinect2_link'
	point.header.stamp = rospy.Time(0)
	point.point.x = data_np['x'][i]
	point.point.y = data_np['y'][i]
	point.point.z = data_np['z'][i]
	p_ = l.transformPoint('joint1' , point)
	points['x'][i] = p_.point.x
	points['y'][i] = p_.point.y
	points['z'][i] = p_.point.z
pc = ros_numpy.msgify(PointCloud2 , points)
pc.header.frame_id = 'joint1'
rate = rospy.Rate(10)
pub=  rospy.Publisher('points_wrt_joint1' , PointCloud2 , queue_size=10)
while not rospy.is_shutdown():
	pub.publish(pc)
	rate.sleep()
