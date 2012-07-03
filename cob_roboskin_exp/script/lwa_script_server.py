#!/usr/bin/python

import time

import roslib
roslib.load_manifest('cob_roboskin_test')
import rospy


from simple_script_server import script

import tf
from geometry_msgs.msg import *
from kinematics_msgs.srv import *

#this should be in manipulation_msgs
#from cob_mmcontroller.msg import *

class GraspScript(script):
		
	def Initialize(self):
		
		# initialize components (not needed for simulation)
		self.listener = tf.TransformListener(True, rospy.Duration(10.0))
		

	def callIKSolver(self, current_pose, goal_pose):
		req = GetPositionIKRequest()
		req.ik_request.ik_link_name = "arm_7_link"
		req.ik_request.ik_seed_state.joint_state.position = current_pose
		req.ik_request.pose_stamped = goal_pose
		resp = self.iks(req)
		result = []
		for o in resp.solution.joint_state.position:
			result.append(o)
		return (result, resp.error_code)

	def Run(self): 
		self.iks = rospy.ServiceProxy('/arm_kinematics/get_ik', GetPositionIK)
		listener = tf.TransformListener(True, rospy.Duration(10.0))
		rospy.sleep(2)

		object_pose_bl = PoseStamped()
		object_pose_bl.header.stamp = rospy.Time.now()
		object_pose_bl.header.frame_id = "/arm_7_link"
		object_pose_bl.pose.position.x = 0
		object_pose_bl.pose.position.y = 0
		object_pose_bl.pose.position.z = 0
		rospy.sleep(2)

		
		if not self.sss.parse:
			object_pose_in = PoseStamped()
			object_pose_in = object_pose_bl
			object_pose_in.header.stamp = listener.getLatestCommonTime("/base_link",object_pose_in.header.frame_id)
			object_pose_bl = listener.transformPose("/base_link", object_pose_in)
			rospy.sleep(2)
			[new_x, new_y, new_z, new_w] = tf.transformations.quaternion_from_euler(-1.552, -0.042, 2.481) # rpy 
			#[new_x, new_y, new_z, new_w] = tf.transformations.quaternion_from_euler(0,0,0) # rpy 
			object_pose_bl.pose.orientation.x = new_x
			object_pose_bl.pose.orientation.y = new_y
			object_pose_bl.pose.orientation.z = new_z
			object_pose_bl.pose.orientation.w = new_w

			#arm_pre_grasp = rospy.get_param("/script_server/arm/pregrasp")
			arm_home = rospy.get_param("/script_server/arm/home")

			# calculate ik solutions for grasp configuration
			(grasp_conf, error_code) = self.callIKSolver(arm_home[0], object_pose_bl)
			if(error_code.val != error_code.SUCCESS):
				rospy.logerr("Ik grasp Failed")
				#return 'retry'
			handle_arm = self.sss.move("arm", [grasp_conf])
			handle_arm.wait()
	

			

if __name__ == "__main__":
	SCRIPT = GraspScript()
	SCRIPT.Start()
