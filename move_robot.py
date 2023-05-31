#!/usr/bin/env python

import rospy
import moveit_commander
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped, Pose
from sensor_msgs.msg import PointCloud2
from tf.transformations import quaternion_from_euler as qe
from tf.transformations import euler_from_quaternion as eq
import ros_numpy
import sys
import numpy

class Move:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('robot_movement')
        self.scene = moveit_commander.PlanningSceneInterface()
        rospy.sleep(1)
        self.arm = moveit_commander.MoveGroupCommander('arm_group')
        self.end_effector_link = self.arm.get_end_effector_link()
        self.reference_frame = 'joint1'
        self.arm.set_pose_reference_frame(self.reference_frame)
        self.arm.allow_replanning(True)
        #self.arm.set_goal_position_tolerance(0.01)
        #self.arm.set_goal_orientation_tolerance(0.05)
        self.arm.set_named_target('init_pose')
        self.arm.go()
    def move(self, point):
        target_pose = PoseStamped()
        target_pose.header.frame_id = self.reference_frame
        target_pose.header.stamp = rospy.Time.now()
	point = numpy.array(point)
	#point = point.astype(float32)
        target_pose.pose.position.x = float(point[0])
        target_pose.pose.position.y = float(point[1])
        target_pose.pose.position.z = float(point[2])
	#roe = qe(0.0 ,, 0.0)
        target_pose.pose.orientation.x = 0.0 #roe[0]
        target_pose.pose.orientation.y = 0.0 #roe[1]
        target_pose.pose.orientation.z = 0.0 #roe[2]
        target_pose.pose.orientation.w = 0.0 #roe[3]
        self.arm.set_start_state_to_current_state()
        self.arm.set_pose_target(target_pose , self.end_effector_link)
        traj = self.arm.plan()
        self.arm.execute(traj)
	self.arm.go()
        rospy.sleep(1)
    def run(self, data):
        for i in range(len(data)):
            self.move([data['x'][i] , data['y'][i] , data['z'][i]])
            rospy.sleep(2)
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__=="__main__":
    robo_move = Move()
    data_pc = rospy.wait_for_message('points_wrt_joint1' , PointCloud2 )
    data_np = ros_numpy.numpify(data_pc)
    robo_move.run(data_np)






