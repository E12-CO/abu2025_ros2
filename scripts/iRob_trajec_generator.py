# Trajectory smoother with save to CSV capabilities
# Thanks god ChatGPT can help me write python

import csv
import math
import sys

import transforms3d
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose
from nav_msgs.msg import Path

import numpy as np
from ccma import CCMA

import os, sys, select, termios, tty

settings = termios.tcgetattr(sys.stdin)

class PathInterpolator(Node):
    def __init__(self, filename):
        super().__init__('path_interpolator')
        self.linearPub_ = self.create_publisher(Path, 'interpolated_path', 10)
        self.smoothPub_ = self.create_publisher(Path, 'smooth_path', 10)
        self.subscription_ = self.create_subscription(
            PoseStamped,
            'goal_pose',
            self.pose_callback,
            10
        )
        self.linearPath_msg = Path()
        self.linearPath_msg.header.frame_id = 'map'
        self.ccma = CCMA(w_ma=300, w_cc=3)

        self.pathCSVName = filename
        with open(self.pathCSVName, mode='w', newline='\n') as file:
            writer = csv.writer(file)
            writer.writerow(['X', 'Y', 'Yaw'])

        self.last_point = None
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.get_logger().info("Robot Club Engineering KMITL : starting Path Interpolator...")

    def quat_to_yaw(self, x, y, z, w):
        (row, pitch, yaw) = transforms3d.euler.quat2euler([w, x, y, z], 'sxyz')
        return yaw

    def interpolate_points(self, start, end, num_points):
        x_values = np.linspace(start[0], end[0], num_points + 2)
        y_values = np.linspace(start[1], end[1], num_points + 2)
        yaw_values = np.linspace(start[2], end[2], num_points + 2)
        return [(x, y, yaw) for x, y, yaw in zip(x_values, y_values, yaw_values)]


    def pose_callback(self, msg):
        """
        Callback function for processing incoming PoseStamped messages.
        """
        self.get_logger().info('Received Pose')

        current_point = (
            msg.pose.position.x, 
            msg.pose.position.y,
            self.quat_to_yaw(
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w
            )
            )

        with open(self.pathCSVName, mode='a', newline='\n') as file:
            writer = csv.writer(file)
            writer.writerow(
                [current_point[0],
                 current_point[1], 
                 current_point[2]]
            )
 

        # If we have a last point, interpolate between them
        if self.last_point is not None:
            interpolated_points = self.interpolate_points(self.last_point, current_point, num_points=50)

            for lPose in interpolated_points:
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.pose.position.x = lPose[0]
                pose.pose.position.y = lPose[1]
                pose.pose.position.z = 0.0
                q = transforms3d.euler.euler2quat(0, 0, lPose[2], 'sxyz')
                pose.pose.orientation.x = q[1]
                pose.pose.orientation.y = q[2]
                pose.pose.orientation.z = q[3]
                pose.pose.orientation.w = q[0]
                self.linearPath_msg.poses.append(pose)

            print("Interpolated path length:", len(self.linearPath_msg.poses))

            # Publish the linear interpolated path
            self.linearPath_msg.header.stamp = self.get_clock().now().to_msg()
            self.linearPub_.publish(self.linearPath_msg)

                
        # Update last point
        self.last_point = current_point

    def timer_callback(self):
        # Keypress detection
        Key = getKey()
        
        if Key == 'r':
            # Clear current path
            self.get_logger().info("Clear current Path...")
            self.linearPath_msg.poses.clear()
            self.last_point = None
        elif Key == 's':
            if len(self.linearPath_msg.poses) < 2:
                return

	    # Smoothing the path
            self.get_logger().info('Generating smooth Path...')
            x_list = []
            y_list = []
            for xy_point in self.linearPath_msg.poses:
                x_list.append(xy_point.pose.position.x)
                y_list.append(xy_point.pose.position.y)

            smooth_path = self.ccma.filter(np.column_stack([x_list,y_list]), cc_mode=False)
            
            smoothPath_msg = Path()
            smoothPath_msg.header.frame_id = 'map'
            
            # Insert initial point
            smoothPose = PoseStamped()
            smoothPose.header.frame_id = 'map'
            smoothPose.header.stamp = self.get_clock().now().to_msg()
            smoothPose.pose.position.x = self.linearPath_msg.poses[0].pose.position.x
            smoothPose.pose.position.y = self.linearPath_msg.poses[0].pose.position.y
            smoothPath_msg.poses.append(smoothPose)

            for smooth_pose in smooth_path:
                smoothPose = PoseStamped()
                smoothPose.header.frame_id = 'map'
                smoothPose.header.stamp = self.get_clock().now().to_msg()
                smoothPose.pose.position.x = smooth_pose[0]
                smoothPose.pose.position.y = smooth_pose[1]
                smoothPath_msg.poses.append(smoothPose)

            # Insert last point
            smoothPose.header.stamp = self.get_clock().now().to_msg()
            smoothPose.pose.position.x = self.linearPath_msg.poses[-1].pose.position.x
            smoothPose.pose.position.y = self.linearPath_msg.poses[-1].pose.position.y
            smoothPath_msg.poses.append(smoothPose)

            # Copy YAW
            smoothPath_msg.poses[0].pose.orientation = self.linearPath_msg.poses[0].pose.orientation

            for yaw_pose_idx in range(0, len(self.linearPath_msg.poses)):
                smoothPath_msg.poses[yaw_pose_idx+1].pose.orientation = self.linearPath_msg.poses[yaw_pose_idx].pose.orientation

            smoothPath_msg.poses[-1].pose.orientation = self.linearPath_msg.poses[-1].pose.orientation

            print("Smooth path length:", len(smoothPath_msg.poses))

            # Update the Path header timestamp
            smoothPath_msg.header.stamp = self.get_clock().now().to_msg()
            # Publish the updated Path
            self.smoothPub_.publish(smoothPath_msg)
            # Clear the original Path
            self.linearPath_msg.poses.clear()
            self.last_point = None

def getKey():
    tty.setcbreak(sys.stdin.fileno())
    key = ''
    if select.select([sys.stdin], [], [], 0.05) == ([sys.stdin], [], []):
        key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) > 1:
        filename = sys.argv[1]
    else:
        print('Error! Please provice the save file name')
        exit()

    node = PathInterpolator(filename)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

