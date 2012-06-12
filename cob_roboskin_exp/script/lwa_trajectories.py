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

class trajectories(script):
		
	def Initialize(self):
		
		# initialize components (not needed for simulation)
		self.listener = tf.TransformListener(True, rospy.Duration(10.0))
		

	def Run(self): 
		#self.iks = rospy.ServiceProxy('/arm_kinematics/get_ik', GetPositionIK)
		#listener = tf.TransformListener(True, rospy.Duration(10.0))
		rospy.sleep(2)

		
		if not self.sss.parse:
			for i in range(1):

				handle_arm = self.sss.move("arm", [[0.22, 1.55, 0, 0, 0.3, 0, 0]])
				handle_arm.wait()
				handle_arm = self.sss.move("arm", [[0.22, 1.68, 0, 0, 0.3, 0, 0]])
				handle_arm.wait()
				rospy.sleep(10)
				handle_arm = self.sss.move("arm", [[0.22, 1.55, 0, 0, 0.3, 0, 0]])
				handle_arm.wait()
				handle_arm = self.sss.move("arm", [[0.22, 1.55, 0, 0, 2, 0, 0]])
				handle_arm.wait()
				handle_arm = self.sss.move("arm", [[0.22, 1.72, 0, 0, 2, 0, 0]])
				handle_arm.wait()
				rospy.sleep(10)
				handle_arm = self.sss.move("arm", [[0.22, 1.55, 0, 0, 2, 0, 0]])
				handle_arm.wait()
				handle_arm = self.sss.move("arm", [[0.22, 1.55, 0, 0, 3.7, 0, 0]])
				handle_arm.wait()
				handle_arm = self.sss.move("arm", [[0.22, 1.68, 0, 0, 3.7, 0, 0]])
				handle_arm.wait()
				rospy.sleep(10)
				handle_arm = self.sss.move("arm", [[0.22, 1.55, 0, 0, 3.7, 0, 0]])
				handle_arm.wait()
				handle_arm = self.sss.move("arm", [[0.22, 1.55, 0, 0, 5.2, 0, 0]])
				handle_arm.wait()
				handle_arm = self.sss.move("arm", [[0.22, 1.72, 0, 0, 5.2, 0, 0]])
				handle_arm.wait()
				rospy.sleep(10)
				handle_arm = self.sss.move("arm", [[0.22, 1.55, 0, 0, 5.2, 0, 0]])
				handle_arm.wait()
				handle_arm = self.sss.move("arm", [[0.22, 1.55, 0, 0, 0.3, 0, 0]])
				handle_arm.wait()

				
	
if __name__ == "__main__":
	SCRIPT = trajectories()
	SCRIPT.Start()
