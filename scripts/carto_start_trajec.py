#!/usr/bin/env python3
# Simple service client node to restart the cartographer trajectory
# Used for Robot retry
# Coded by TinLethax (RB26)
import os

import rclpy
from rclpy.node import Node

import transforms3d
from cartographer_ros_msgs.srv import StartTrajectory, GetTrajectoryStates, FinishTrajectory

from ament_index_python.packages import get_package_share_directory

import sys, select, termios, tty

settings = termios.tcgetattr(sys.stdin)

class carto_start_trajec(Node):
	
	def __init__(self):
		super().__init__('carto_start_trajec')
		self.lastTrajecId = 0
		self.KeyFSM = "idle"
			
		self.get_logger().info('Starting Carto starter')
		self.timer = self.create_timer(0.1, self.timer_callback)

	def startTrajec(self):
		StartTrajCli = self.create_client(StartTrajectory, 'start_trajectory')
		while not StartTrajCli.wait_for_service(timeout_sec=1.0):
			self.get_logger().info('Waiting for start Trajectory service...')
		StartTrajReq = StartTrajectory.Request()
		q = transform3d.euler.euler2quat(0, 0, 0.0, 'sxyz')
		print('Sending Start trajectory request to cartographer')
		StartTrajReq.configuration_directory = os.path.join(get_package_share_directory('abu2025_ros2'), 'R1/params_r1')
		StartTrajReq.configuration_basename = 'R1_hokuyo_2d.lua'
		StartTrajReq.use_initial_pose = True
		StartTrajReq.initial_pose.position.x = 0.3
		StartTrajReq.initial_pose.position.y = -0.3
		StartTrajReq.initial_pose.position.z = 0.0
		StartTrajReq.initial_pose.orientation.x = q[1]
		StartTrajReq.initial_pose.orientation.y = q[2]
		StartTrajReq.initial_pose.orientation.z = q[3]
		StartTrajReq.initial_pose.orientation.w = q[0]
		StartTrajReq.relative_to_trajectory_id = self.lastTrajecId + 1 # Start on next trajectory ID
		future = StartTrajCli.call_async(StartTrajReq)
		futre.add_done_callback(self.startTrajecCallback)

	def startTrajecCallback(self, future):
		try:
			startTrajRes = future.result()
			self.get_logger().info(
				'Start trajec result: freed trajec ID : %d' %
				(startTrajRes.trajectory_id)
			)
		except Exception as e:
			self.get_logger().info(f'Service call failed {e}')

	def getTrajecId(self):
		self.get_logger().info('Request Current Trajectory ID')
		GetTrajCli = self.create_client(GetTrajectoryStates, 'get_trajectory_states')
		while not GetTrajCli.wait_for_service(timeout_sec=1.0):
			self.get_logger().info('Waiting for Trajectory ID service...')
		GetTrajReq = GetTrajectoryStates.Request()

		future = GetTrajCli.call_async(GetTrajReq)
		future.add_done_callback(self.getTrajecIdCallback)

	def getTrajecIdCallback(self, future):
		try:
			trajIdRes = future.result()
			self.lastTrajecId = trajIdRes.trajectory_states.trajectory_id
			self.get_logger().info('Latest trajectory ID : %d' % (self.lastTrajecId))
			self.KetFSM = "gottId"
		except Exception as e:
            		self.get_logger().info(f'Service call failed {e}')

	def finishTraj(self, tId):
		self.get_logger('Finishing current trajectory')
		FinTrajCli = self.create_client(FinishTrajectory, 'finish_trajectory')
		while not FinTrajCli.wait_for_service(timeout_sec=1.0):
			self.get_logger().info('Waiting for Finish Trajectory service...')
		FinTrajReq = FinishTrajectory.Request()
		FinTrajReq.trajectory_id = tId

		future = FinTrajCli.call_asyn(FinTrajReq)
		future.add_done_callback(self.finishTrajCallback)

	def finishTrajCallback(self, future):
		try:
			finTrajRes = future.result()
			self.get_logger().info('Finished current trajectory')
			self.KeyFSM = 'finished'
		except Exception as e:
			self.get_logger().info(f'Service call failed {e}')

	def timer_callback(self):
		match self.KeyFSM:
			case 'idle':
				key = getKey()
				if key == 'r': # if press 'r', restart cartographer
					self.KeyFSM = 'waitId'
					# Get current trajectory
					self.getTrajecId()
			case 'waitId':
				return

			case 'gettId':
				# Finish the current trajectory
				self.KeyFSM = 'waitFinish'
				self.finishTrajCallback(self.lastTrajecId)
		
			case 'waitFinish':
				return

			case 'finished':
				# Restart the cartographer with next trajectory Id
				self.KeyFSM = 'idle'
				self.startTrajec()

			case _:
				self.KeyFSM = 'idle'

def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

def main(args=None):
	rclpy.init(args=args)

	carto_client = carto_start_trajec()
	rclpy.spin(carto_client)
	rclpy.shutdown()

if __name__ == '__main__':
	main()
