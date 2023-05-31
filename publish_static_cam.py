#!/usr/bin/env python
import rospy
import tf2_ros
import tf2_msgs.msg
from tf.transformations import quaternion_from_euler as qe
from tf.transformations import euler_from_quaternion as eq
from geometry_msgs.msg import Point, TransformStamped , PoseStamped

if __name__=="__main__":
	rospy.init_node('cam_to_robot')
	pub = rospy.Publisher('/tf' , tf2_msgs.msg.TFMessage , queue_size = 1)
	data = rospy.wait_for_message('/xyz_of_robot_from_cam' , Point)
	data_o = rospy.wait_for_message('/aruco_single/pose', PoseStamped)
	print('Data received')
	while not rospy.is_shutdown():
		rospy.sleep(0.1)
		robot_transform = TransformStamped()
		robot_transform.header.frame_id = 'kinect2_link'
		robot_transform.header.stamp = rospy.Time.now()
		robot_transform.child_frame_id = 'joint1'
		robot_transform.transform.translation.x = data.x 
		robot_transform.transform.translation.y = data.y + 0.05
		robot_transform.transform.translation.z = data.z +0.05
		#robot_rotation = qe(3.14,1.2 , 1.57)
		roe = eq([data_o.pose.orientation.x , data_o.pose.orientation.y , data_o.pose.orientation.z , data_o.pose.orientation.w])
		roq = qe(roe[0] -1.57, roe[1] +1.57 , roe[2]+0.5 )
		#roq = qe(roe[2] , roe[0] , roe[1])
		robot_transform.transform.rotation.x = roq[0]
		robot_transform.transform.rotation.y = roq[1]
		robot_transform.transform.rotation.z = roq[2]
		robot_transform.transform.rotation.w = roq[3]
		robot_tfm = tf2_msgs.msg.TFMessage([robot_transform])
		pub.publish(robot_tfm)
	rospy.spin()
