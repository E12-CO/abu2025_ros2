import socket
import time
import errno

import transforms3d

import rclpy
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

import tf2_ros
import tf2_geometry_msgs

from geometry_msgs.msg import TransformStamped, Vector3

class socket_pose(Node):
	
	def __init__(self):
		super().__init__('robotPoseSocketNode')
		
		self.sock = socket.socket()

		try:
			self.sock.bind(("127.0.0.1", 5069))
		except socket.error:
			self.sock.close()
			self.sock.bind(("127.0.0.1", 5069))


		self.sock.listen(1)
		self.conn = None

		self.tf_buffer = tf2_ros.Buffer()
		self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

		self.r1Pose = TransformStamped()
		self.r2Pose = TransformStamped()

		# Initial pose on the map
		self.r1PoseStr = '0.40, -0.40, 0.0'
		self.r2PoseStr = '0.40, -1.40, 0.0'

		self.timer = self.create_timer(0.05, self.timer_callback)
		self.get_logger().info("Robot Club Engineering KMITL : starting robot Pose socket node...")
	
	def quat_to_yaw(self, x, y, z, w):
		(row, pitch, yaw) = transforms3d.euler.quat2euler([w, x, y, z], 'sxyz')
		if yaw < 0.0:
			yaw += 6.283185
		return -yaw

	def timer_callback(self):
		if self.conn is None:
			self.conn, _ = self.sock.accept()

		# Lookup for R1 transform from Map to Baselink
		try:
			self.r1Pose = self.tf_buffer.lookup_transform(
                		"map", "chassis_r1",  rclpy.time.Time()
            		)

			self.r1PoseStr = (
				str(round(self.r1Pose.transform.translation.x, 2)) + ', ' +
				str(round(self.r1Pose.transform.translation.y, 2)) + ', ' +
				str(round(self.quat_to_yaw(
					self.r1Pose.transform.rotation.x,
					self.r1Pose.transform.rotation.y,
					self.r1Pose.transform.rotation.z,
					self.r1Pose.transform.rotation.w
					), 2))
			)

		except (
            		tf2_ros.LookupException,
            		tf2_ros.ConnectivityException,
            		tf2_ros.ExtrapolationException,
        	):
			self.get_logger().debug("Can't get R1 transform!")
		
		# Loopup for R2 transform from Map to Baselink
		try:
			self.r2Pose = self.tf_buffer.lookup_transform(
                		"map", "chassis_r2", rclpy.time.Time()
            		)

			self.r2PoseStr = (
				str(round(self.r2Pose.transform.translation.x, 2)) + ', ' +
				str(round(self.r2Pose.transform.translation.y, 2)) + ', ' +
				str(round(self.quat_to_yaw(
					self.r2Pose.transform.rotation.x,
					self.r2Pose.transform.rotation.y,
					self.r2Pose.transform.rotation.z,
					self.r2Pose.transform.rotation.w
					), 2))
			)

		except (
            		tf2_ros.LookupException,
            		tf2_ros.ConnectivityException,
            		tf2_ros.ExtrapolationException,
        	):
			self.get_logger().debug("Can't get R2 transform!")

		sumStr = self.r1PoseStr + '%' + self.r2PoseStr

		try:
			self.conn.send(sumStr.encode())
		except socket.error:
			self.get_logger().error("Socket error!")
			self.conn.close()
			self.conn = None
			self.sock.listen(1) # Listen for next connection


def main(args=None):
	rclpy.init(args=args)
	
	socketPoseNode = socket_pose()
	rclpy.spin(socketPoseNode)
	socketPoseNode.conn.close()
	socketPoseNode.sock.close()
	socketPoseNode.destroy_node()
	rclpy.shutdown()

if __name__ == "__main__":
	main()
