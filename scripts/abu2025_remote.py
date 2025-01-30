from pynodered import node_red

# ROS2 stuffs
import rclpy
from rclpy.node import Node
from std_msgs.msg import String as StringMsg

teamModeStr = ''
playModeStr = ''

poseX = 0
poseY = 0


@node_red(category="pyfuncs")
def team_mode(node, msg):

	teamModeStr = str(msg['payload'])
	msg['payload'] = teamModeStr
	rclpy.init(args=None)
	abu_teammode_node = abu_teammode()
	abu_teammode_node.get_logger().info('Robot Club KMITL : Select team: ' % teamModeStr)
	
	abu_teammode_node.publish_teammode(teamModeStr)
	rclpy.spin_once(abu_teammode_node, timeout_sec=0.1)
	abu_teammode_node.destroy_node()
	rclpy.shutdown()

	return msg

@node_red(category="pyfuncs")
def play_mode(node, msg):

	teamModeStr = str(msg['payload'])
	msg['payload'] = teamModeStr
	rclpy.init(args=None)
	abu_teammode_node = abu_teammode()
	abu_teammode_node.get_logger().info('Robot Club KMITL : Selected play mode: ' % teamModeStr)
	
	abu_teammode_node.publish_playmode(playModeStr)
	rclpy.spin_once(abu_teammode_node, timeout_sec=0.1)
	abu_teammode_node.destroy_node()
	rclpy.shutdown()

	return msg


@node_red(category="pyfuncs")
def emergency_btn(node, msg):
	
	rclpy.init(args=None)
	abu_teammode_node = abu_teammode()
	abu_teammode_node.get_logger().info('Robot Club KMITL : Sending Emergency stop...')
	
	abu_teammode_node.publish_emergency()
	rclpy.spin_once(abu_teammode_node, timeout_sec=0.1)
	abu_teammode_node.destroy_node()
	rclpy.shutdown()

	return msg

class abu_teammode(Node):

	def __init__(self):
		super().__init__('AbuTeamModeNode')
		self.pub_teammode = self.create_publisher(
			StringMsg,
			'teammode',
			10)

		self.pub_playmode = self.create_publisher(
			StringMsg,
			'playmode',
			10)

		self.pub_emer = self.create_publisher(
			StringMsg,
			'irob_cmd',
			10
			)

	def publish_teammode(self, team_str):
		msg = StringMsg()
		msg.data = team_str
		self.pub_teammode.publish(msg)

	def publish_playmode(self, play_str):
		msg = StringMsg()
		msg.data = play_str
		self.pub_playmode.publish(msg)
	
	def publish_emergency(self):
		msg = StringMsg()
		msg.data = 'stop'
		self.pub_emer.publish(msg)
