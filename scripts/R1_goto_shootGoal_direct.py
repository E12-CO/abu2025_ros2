# R1 Goto shoot point directly (no obstacle avoidance)
# Hoop X,Y position is [13.4, -3.0]

import math
import sys

import transforms3d
import rclpy
import tf2_ros
import tf2_geometry_msgs

from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped

from visualization_msgs.msg import Marker

import os, sys, select, termios, tty

settings = termios.tcgetattr(sys.stdin)

class R1GotoShooter(Node):
    def __init__(self):
        super().__init__('r1_goto_shoot')

        # create TF2 transform listener, We're looking for current R1 position in the map
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.r1Pose = TransformStamped()

        # Hoop position marker
        self.hoopMarkPub_ = self.create_publisher(Marker, 'r1_hoopmark', 10)

        # Shoot position Publisher
        self.goalPub_ = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.goalMarkPub_ = self.create_publisher(Marker, 'r1_shootmark', 10)
        

        # Hoop pose
        self.hoopPose = PoseStamped()
        self.hoopPose.pose.position.x = 13.0
        self.hoopPose.pose.position.y = -3.0
        # Shoot radius, 2 meters
        self.shootRadius = 2.0 

        # Goal pose 
        self.goalPose = PoseStamped()
        
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.get_logger().info("Robot Club Engineering KMITL : r1 goto shooter starting...")

    def quat_to_yaw(self, x, y, z, w):
        (row, pitch, yaw) = transforms3d.euler.quat2euler([w, x, y, z], 'sxyz')
        return yaw

    def draw_hoopMarker(self):
        hoopMark = Marker()
        hoopMark.header.frame_id = 'map'
        hoopMark.header.stamp = self.get_clock().now().to_msg()
        hoopMark.id = 0
        hoopMark.type = hoopMark.SPHERE
        hoopMark.action = hoopMark.ADD
        hoopMark.scale.x = 0.5
        hoopMark.scale.y = 0.5
        hoopMark.scale.z = 0.5
        hoopMark.color.a = 1.0
        hoopMark.color.r = 1.0
        hoopMark.pose.position.x = 13.0
        hoopMark.pose.position.y = -3.0
        hoopMark.pose.position.z = 0.5
        self.hoopMarkPub_.publish(hoopMark)

    def draw_shootMarker(self, x, y):
        shootMark = Marker()
        shootMark.header.frame_id = 'map'
        shootMark.header.stamp = self.get_clock().now().to_msg()
        shootMark.id = 0
        shootMark.type = shootMark.SPHERE
        shootMark.action = shootMark.ADD
        shootMark.scale.x = 0.3
        shootMark.scale.y = 0.3
        shootMark.scale.z = 0.3
        shootMark.color.a = 1.0
        shootMark.color.g = 1.0
        shootMark.pose.position.x = x
        shootMark.pose.position.y = y
        shootMark.pose.position.z = 0.5
        self.goalMarkPub_.publish(shootMark)

    def calculate_goal(self):
        self.get_logger().info('Calculating pose...')
        # Calculation steps
        # 1. Get current R1 position with lookuptransform
        # 2. Calculate the heading angle from the R1 to the hoop position
        heading = math.atan2(
                (self.hoopPose.pose.position.y - self.r1Pose.transform.translation.y),
                (self.hoopPose.pose.position.x - self.r1Pose.transform.translation.x)
            )
        # 3. Calculate the X,Y goal point relative to the hoop position. Position in map frame
        self.goalPose.pose.position.x = (self.shootRadius * math.cos(heading + 3.141593)) + self.hoopPose.pose.position.x
        self.goalPose.pose.position.y = (self.shootRadius * math.sin(heading + 3.141593)) + self.hoopPose.pose.position.y
        self.draw_shootMarker(self.goalPose.pose.position.x, self.goalPose.pose.position.y)
        # 4. The heading will be used to control robot orientation to face the hoop
        q = transforms3d.euler.euler2quat(0, 0, heading, 'sxyz')
        self.goalPose.pose.orientation.w = q[0]
        self.goalPose.pose.orientation.x = q[1]
        self.goalPose.pose.orientation.y = q[2]
        self.goalPose.pose.orientation.z = q[3]

        # 5. Publish goal_pose
        self.goalPub_.publish(self.goalPose)

    def timer_callback(self):
        self.draw_hoopMarker()
        key = getKey()

        # Listen for R1 transform map --> r1_base_link
        try:
            self.r1Pose = self.tf_buffer.lookup_transform(
                 "map",
                 "base_link_r1",
                 rclpy.time.Time()
                )

       	except(
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            self.get_logger().warning('Transform Lookup R1 failed!, Check Cartographer node!')

        if key == 'r':
            self.calculate_goal()

        return

def getKey():
    tty.setcbreak(sys.stdin.fileno())
    key = ''
    if select.select([sys.stdin], [], [], 0.05) == ([sys.stdin], [], []):
        key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main(args=None):
    rclpy.init(args=args)

    node = R1GotoShooter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

