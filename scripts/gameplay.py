#!/usr/bin/env python3
# ABU 2025 semi-auto gameplay system for R1 and R2 robot
# Coded by TinLethax (RB26)

import math
import sys

import transforms3d
import rclpy
import tf2_ros
import tf2_geometry_msgs

from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, TransformStamped
from visualization_msgs.msg import Marker

import os, sys, select, termios, tty

settings = termios.tcgetattr(sys.stdin)

class abugameplay(Node):
    def __init__(self):
        super().__init__('abu2025_gameplay')

        # FSM
        self.mainFSM = 'idle'
        self.ballPosession = False

        # create TF2 transform listener, We're looking for current R1 and R2 position in the map
        # self Transform Listener
        self.tf_buffer_self = tf2_ros.Buffer()
        self.tf_listener_self = tf2_ros.TransformListener(self.tf_buffer_self, self)
        self.selfPose = TransformStamped()
        
        # buddy Transform Listener
        self.tf_buffer_buddy = tf2_ros.Buffer()
        self.tf_listener_buddy = tf2_ros.TransformListener(self.tf_buffer_buddy, self)
        self.buddyPose = TransformStamped()

        # ROS2 parameters
        self.declare_parameter('self_robot_frame', 'base_link_r1')
        self.declare_parameter('buddy_robot_frame', 'base_link_r2')
        

        self.self_robot_frame = str(self.get_parameter("self_robot_frame").value)
        self.buddy_robot_frame = str(self.get_parameter("buddy_robot_frame").value)
   
        # Self to buddy message publisher
        self.selfToBudPub_ = self.create_publisher(String, 'to_buddy', 10)
        # Buddy to self message subscriber
        self.budToSelfPub_ = self.create_subscription(String, 'from_buddy', self.msg_BuddyToSelf, 10)

        # Hoop position marker
        self.hoopMarkPub_ = self.create_publisher(Marker, 'hoopmark', 10)

        # Shoot position Publisher
        self.goalPub_ = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.goalMarkPub_ = self.create_publisher(Marker, 'shootmark', 10)
        
        # iRob maneuv3r stuffs
        self.irobCmdPub_ = self.create_publisher(String, 'irob_cmd', 10)
        self.irobStatSub_ = self.create_subscription(String, 'rob_stat', self.irobStatCallback, 10)
        
        # Hoop pose
        self.hoopPose = PoseStamped()
        self.hoopPose.pose.position.x = 13.0
        self.hoopPose.pose.position.y = -3.0
        # Shoot radius, 2 meters
        self.shootRadius = 2.0 

        # Goal pose 
        self.goalPose = PoseStamped()
        
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.get_logger().info("Robot Club Engineering KMITL : ABU2025 gameplay system...")

    # Convert quaternion to euler yaw
    def quat_to_yaw(self, x, y, z, w):
        (row, pitch, yaw) = transforms3d.euler.quat2euler([w, x, y, z], 'sxyz')
        return yaw

    # Listen for self transform map --> rX_base_link
    def tf_selfListener(self):
        
        try:
            self.selfPose = self.tf_buffer_self.lookup_transform(
                 "map",
                 self.self_robot_frame,
                 rclpy.time.Time()
                )
            
       	except(
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            self.get_logger().warning('Self Transform Lookup failed!, Check Cartographer node!')
            return 1
        
        return 0
    
    # Listen for buddy transform map --> rX_base_link
    def tf_buddyListener(self):    
        
        try:
            self.buddyPose = self.tf_buffer_buddy.lookup_transform(
                 "map",
                 self.buddy_robot_frame,
                 rclpy.time.Time()
                )
            
       	except(
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            self.get_logger().warning('Buddy Transform Lookup failed!, Check Cartographer node!')
            return 1
        
        return 0
    
    # Draw Hoop marker on rViz
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
        hoopMark.color.r = 0.5
        hoopMark.pose.position.x = 13.0
        hoopMark.pose.position.y = -3.0
        hoopMark.pose.position.z = 0.5
        self.hoopMarkPub_.publish(hoopMark)

    # Draw shoot goal point on rViz
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

    # Send message to buddy
    def msg_selfToBuddy(self, str):
        txMsg = String()
        txMsg.data = str
        
        self.selfToBudPub_.publish(txMsg)
        
    # receive message from buddy
    def msg_BuddyToSelf(self, msg):
        return

    # send iRob maneuv3r command
    def irobSendCmd(self, cmdStr):
        cmdMsg = String()
        cmdMsg.data = cmdStr
        
        self.irobCmdPub_.publish(cmdMsg)
        
    # Receive goal status from iRob maneuv3r    
    def irobStatCallback(self, msg):
        return

    # Calculate shoot goal pose
    def calculate_shootGoal(self):
        self.get_logger().info('Calculating shoot pose...')
        # Calculation steps
        # 1. Get current position with lookuptransform
        # 2. Calculate the heading angle from the self to the hoop position
        heading = math.atan2(
                (self.hoopPose.pose.position.y - self.selfPose.transform.translation.y),
                (self.hoopPose.pose.position.x - self.selfPose.transform.translation.x)
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
        
    # Calculate pass goal pose    
    def calculate_passGoal(self):
        self.get_logger().info('Calculating pass pose...')
        # Calculation steps
        # 1. Get R1 and R2 position with lookuptransform
        # 2. Calculate the heading from robot with ball posession to the buddy robot
        heading = math.atan2(
                (self.buddyPose.transform.translation.y - self.selfPose.transform.translation.y),
                (self.buddyPose.transform.translation.x - self.selfPose.transform.translation.x)
            )
        # 3. Calculate the X,Y goal point relative to the buddy robot position. Position in map frame
        self.goalPose.pose.position.x = (self.shootRadius * math.cos(heading + 3.141593)) + self.hoopPose.pose.position.x
        self.goalPose.pose.position.y = (self.shootRadius * math.sin(heading + 3.141593)) + self.hoopPose.pose.position.y
        self.draw_shootMarker(self.goalPose.pose.position.x, self.goalPose.pose.position.y)
        # 4. The heading will be used to control robot orientation to face the buddy
        q = transforms3d.euler.euler2quat(0, 0, heading, 'sxyz')
        self.goalPose.pose.orientation.w = q[0]
        self.goalPose.pose.orientation.x = q[1]
        self.goalPose.pose.orientation.y = q[2]
        self.goalPose.pose.orientation.z = q[3]

        # 5. Publish goal_pose
        self.goalPub_.publish(self.goalPose)
        
    # Calculate receive pass orientation
    def calculate_passOrient(self):
        self.get_logger().info('Calculating pass receive pose...')
        # Calculation steps
        # 1. Get R1 and R2 position with lookuptransform
        # 2. Calculate the heading from self to buddy robot with the ball posession
        heading = math.atan2(
                (self.buddyPose.transform.translation.y - self.selfPose.transform.translation.y),
                (self.buddyPose.transform.translation.x - self.selfPose.transform.translation.x)
            )
        # 3. X,Y goal will be the current self position
        self.goalPose.pose.position.x = self.selfPose.transform.translation.x
        self.goalPose.pose.position.y = self.selfPose.transform.translation.y
        self.draw_shootMarker(self.goalPose.pose.position.x, self.goalPose.pose.position.y)
        # 4. The heading will be pre-process. As the ball receiver was on the back of the robot
        q = transforms3d.euler.euler2quat(0, 0, heading + 3.141593, 'sxyz')
        self.goalPose.pose.orientation.w = q[0]
        self.goalPose.pose.orientation.x = q[1]
        self.goalPose.pose.orientation.y = q[2]
        self.goalPose.pose.orientation.z = q[3]
        
        # 5. Publish goal_pose
        self.goalPub_.publish(self.goalPose)

    def abu_gameplayFSM(self):
        match self.mainFSM:
            case 'idle':
                if self.ballPosession is True:
                    self.mainFSM = 'shootBall'
                else:
                    self.mainFSM = 'waitBall'
                
            case 'shootBall':
                return
                
            case _:
                self.mainFSM = 'idle'

    def timer_callback(self):
        self.draw_hoopMarker()
        key = getKey()

        if self.tf_selfListener() or self.tf_buddyListener():
            self.get_logger().warning('Lookup transform error, will skip this cycle...')
            return

        if key == 'r':
            self.calculate_passGoal()

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

    node = abugameplay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()