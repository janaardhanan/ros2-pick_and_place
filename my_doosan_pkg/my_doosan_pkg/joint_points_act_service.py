'''
Author: David Valencia
Date: 26 / 08 /2021

Describer:  An action Client to move the robot joint to a specific position

			This script send the position "angles" of each joint under 
			
			the ACTION - SERVICE /joint_trajectory_controller/joint_trajectory
			
			I need to run first the my_doosan_controller in order to load and start the controllers
			Update: I can also run my environment launch file 

			Executable name in the setup file: trajectory_points_act_server		
'''

import rclpy
from rclpy.duration import Duration
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


class TrajectoryActionClient(Node):

	def __init__(self):

		super().__init__('points_publisher_node_action_client')
		self.action_client = ActionClient (self, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')

	def send_goal(self):

		points = []
		time=0.0

		point_msg = JointTrajectoryPoint()
		point_msg.positions=[0.0, 0.0, 0.0, 0.0, 1.755, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 
		# point_msg.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
		time+=2
		point_msg.time_from_start = Duration(seconds=time, nanoseconds=0).to_msg()
		points.append(point_msg)

		point_msg = JointTrajectoryPoint()
		point_msg.positions=[0.0, 1.4, 0.0, 0.0, 1.755, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 
		# point_msg.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
		time+=2
		point_msg.time_from_start = Duration(seconds=time, nanoseconds=0).to_msg()
		points.append(point_msg)

		point_msg = JointTrajectoryPoint()
		point_msg.positions=[0.0, 1.4, 0.0, 0.0, 1.755, 0.0, 0.6, 0.6, 0.6, 0.6, -0.6, -0.6] 
		time+=2
		point_msg.time_from_start = Duration(seconds=time, nanoseconds=0).to_msg()
		points.append(point_msg)


		joint_names = ['joint1','joint2','joint3','joint4','joint5','joint6', 
				 'robotiq_85_left_knuckle_joint', 'robotiq_85_right_knuckle_joint', 
				 'robotiq_85_left_inner_knuckle_joint', 'robotiq_85_right_inner_knuckle_joint', 
				 'robotiq_85_left_finger_tip_joint', 'robotiq_85_right_finger_tip_joint']

		goal_msg = FollowJointTrajectory.Goal()
		goal_msg.goal_time_tolerance = Duration(seconds=1, nanoseconds=0).to_msg()
		goal_msg.trajectory.joint_names = joint_names
		goal_msg.trajectory.points = points

		self.action_client.wait_for_server()
		self.send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
		self.send_goal_future.add_done_callback(self.goal_response_callback)

	
	def goal_response_callback(self, future):
		
		goal_handle = future.result()

		if not goal_handle.accepted:
			self.get_logger().info('Goal rejected ')
			return

		self.get_logger().info('Goal accepted')

		self.get_result_future= goal_handle.get_result_async()
		self.get_result_future.add_done_callback(self.get_result_callback)

	def get_result_callback (self, future):
		
		result = future.result().result
		self.get_logger().info('Result: '+str(result))
		print("--------------------------------------------\n\nThe simulated robot is destroyed successfully. (THis shall be fixed!)\n\n--------------------------------------------")
		rclpy.shutdown()

		
	def feedback_callback(self, feedback_msg):
		feedback = feedback_msg.feedback


def main(args=None):

	rclpy.init()
	
	action_client = TrajectoryActionClient()
	future = action_client.send_goal()
	rclpy.spin(action_client)

if __name__ == '__main__':
	main()




