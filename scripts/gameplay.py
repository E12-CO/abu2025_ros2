#!/usr/bin/env python3
# ABU 2025 semi-auto gameplay system for R1 and R2 robot
# Coded by TinLethax (RB26)

# on R1 run
# ros2 run abu2025_ros2 gameplay.py --ros-args -r __ns:=/r1  -r to_buddy:=/r2/from_buddy

# on R2 run
# ros2 run abu2025_ros2 gameplay.py --ros-args -r __ns:=/r2 -p "self_robot_frame:=base_link_r2" -p "buddy_robot_frame:=base_link_r1" -r to_buddy:=/r1/from_buddy

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
from irob_msgs.msg import IrobCmdMsg

import os, sys, select, termios, tty

settings = termios.tcgetattr(sys.stdin)

class abugameplay(Node):
    def __init__(self):
        super().__init__('abu2025_gameplay')

        # FSM
        self.mainFSM = 'idle'
        
        # Key press (temporary)
        self.key = '' 
        
        # Keep track of ball position of self and buddy robot
        self.selfBallPosession = False
        self.buddyBallPossesion = False
        
        # Flag to goto ball receive pose
        self.receiveBallCmdFlag = False

        # Get namespace for unique robot customization
        self.selfNamespace = self.get_namespace()

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
        
        # Posession marker
        self.possMarkPub_ = self.create_publisher(Marker, 'possesion', 10)
        
        # iRob maneuv3r stuffs
        self.irobCmdPub_ = self.create_publisher(IrobCmdMsg, 'irob_cmd', 10)
        self.irobStatSub_ = self.create_subscription(IrobCmdMsg, 'irob_stat', self.irobStatCallback, 10)
        
        # Hoop pose
        self.hoopPose = PoseStamped()
        self.hoopPose.pose.position.x = 13.0
        self.hoopPose.pose.position.y = -3.0
        # Shoot radius, 2 meters
        self.shootRadius = 2.0 

        # Goal pose 
        self.goalPose = PoseStamped()
        
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.get_logger().info(f"Robot Club Engineering KMITL : {self.selfNamespace} ABU2025 gameplay system...")

    # Convert quaternion to euler yaw
    def quat_to_yaw(self, x, y, z, w):
        (row, pitch, yaw) = transforms3d.euler.quat2euler([w, x, y, z], 'sxyz')
        return yaw

    # Listen for self transform map --> rX_base_link
    def tf_selfListener(self):
        if not self.tf_buffer_self.can_transform("map", self.self_robot_frame, rclpy.time.Time()):
            self.get_logger().warning('Can\'t get self robot transform right now...')
            return 1
            
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
        if not self.tf_buffer_buddy.can_transform("map", self.buddy_robot_frame, rclpy.time.Time()):
            self.get_logger().warning('Can\'t get buddy robot transform right now...')
            return 1
            
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
    
    # Draw posession marker on rViz
    def draw_possesionMarker(self):
        possesMark = Marker()
        possesMark.header.frame_id = 'map';
        possesMark.header.stamp = self.get_clock().now().to_msg()
        possesMark.id = 0
        possesMark.type = possesMark.SPHERE
        if self.selfBallPosession is True:
            possesMark.action = possesMark.ADD
        else:
            possesMark.action = possesMark.DELETE
            
        possesMark.scale.x = 0.3
        possesMark.scale.y = 0.3
        possesMark.scale.z = 0.3
        possesMark.color.a = 1.0
        possesMark.color.g = 0.4
        possesMark.color.r = 1.0
        possesMark.pose.position.x = self.selfPose.transform.translation.x
        possesMark.pose.position.y = self.selfPose.transform.translation.y
        possesMark.pose.position.z = 0.5
        self.possMarkPub_.publish(possesMark)
        
    # Messages for Ball passing and posession transfer
    
    # 'BallPass' 
    # Send from robot with ball posession to the ball receiver robot.
    # This will be send when play press pass ball button
    
    # 'BallReceived'
    # Send from robot that received ball pass AND checked that the ball is in possesion.
    
    # 'PossesState'
    # Inquiry buddy robot for ball posession status.
    # Return with 
    # 'YesBall' or 'NoBall'
    
    # 
    
    # Send message to buddy
    def msg_selfToBuddy(self, str):
        txMsg = String()
        txMsg.data = str
        
        self.selfToBudPub_.publish(txMsg)
        
    # receive message from buddy
    def msg_BuddyToSelf(self, msg):
        self.get_logger().debug('Received data from buddy : ' + msg.data)
        
        match msg.data:
            case 'BallPass':
                self.get_logger().info('Moving to ball pass receive position')
                if self.receiveBallCmdFlag is False:
                    self.receiveBallCmdFlag = True
                
            case 'BallReceived':
                self.get_logger().info('Buddy received ball pass')
                self.buddyBallPossesion = True
            
            case 'PossesState':
                self.get_logger().info('Buddy asked for ball posession status, replying...')
                if self.selfBallPosession is True:
                    self.msg_selfToBuddy('YesBall')
                else:
                    self.msg_selfToBuddy('NoBall')
            
            case 'YesBall':
                self.get_logger().info('Buddy has ball!')
                self.buddyBallPossesion = True 
            
            case 'NoBall':
                self.get_logger().info('Buddy has no ball!')
                self.buddyBallPossesion = False
                
            case _:
                return
                
        return

    def msg_InquireBuddyBallStatus(self):
        self.msg_selfToBuddy('PossesState')

    # send iRob maneuv3r command
    def irobSendCmd(self, cmdStr):
        cmdMsg = IrobCmdMsg()
        cmdMsg.irobcmd = cmdStr
        
        self.irobCmdPub_.publish(cmdMsg)
        
    # Receive goal status from iRob maneuv3r    
    def irobStatCallback(self, msg):
        self.get_logger().info(f'Got iRob message! : {msg.irobcmd}')
    
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
        # 3. Calculate Buddy's orientation
        buddy_orientation = self.quat_to_yaw(
            self.buddyPose.transform.rotation.x,
            self.buddyPose.transform.rotation.y,
            self.buddyPose.transform.rotation.z,
            self.buddyPose.transform.rotation.w)
            
        # 4. Calculate the X,Y goal point relative to the buddy robot position. Position in map frame
        self.goalPose.pose.position.x = (self.shootRadius * math.cos(buddy_orientation + 3.141593)) + self.buddyPose.transform.translation.x
        self.goalPose.pose.position.y = (self.shootRadius * math.sin(buddy_orientation + 3.141593)) + self.buddyPose.transform.translation.y
        self.draw_shootMarker(self.goalPose.pose.position.x, self.goalPose.pose.position.y)
        
        # 5. Buddy orientation will be used to control robot orientation to face the buddy
        q = transforms3d.euler.euler2quat(0, 0, buddy_orientation, 'sxyz')
        self.goalPose.pose.orientation.w = q[0]
        self.goalPose.pose.orientation.x = q[1]
        self.goalPose.pose.orientation.y = q[2]
        self.goalPose.pose.orientation.z = q[3]

        # 6. Publish goal_pose
        self.goalPub_.publish(self.goalPose)
        
    # Calculate receive pass orientation, rotates ball receiver to buddy robot.
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

    def calculate_poseHome(self):
        # 1. pick home pose 
        if self.selfNamespace == '/r1':
            home_x = 0.0
            home_y = 0.0
        elif self.selfNamespace == '/r2':
            home_x = 0.0
            home_y = -1.0
        else:
            return
        
         # 3. X,Y goal will be the home position
        self.goalPose.pose.position.x = home_x
        self.goalPose.pose.position.y = home_y
        self.draw_shootMarker(self.goalPose.pose.position.x, self.goalPose.pose.position.y)
        # 4. The orientation will be 0 degree
        q = transforms3d.euler.euler2quat(0, 0, 0, 'sxyz')
        self.goalPose.pose.orientation.w = q[0]
        self.goalPose.pose.orientation.x = q[1]
        self.goalPose.pose.orientation.y = q[2]
        self.goalPose.pose.orientation.z = q[3]
        
        self.goalPub_.publish(self.goalPose)
        
            
    def abu_gameplayFSM(self):
        match self.mainFSM:
            case 'idle': # Start case
                if self.selfBallPosession is True:
                    self.mainFSM = 'shootBall'
                else:
                    self.mainFSM = 'waitBall'
                
            case 'shootBall': # Shoot ball mode
                if self.key == 'r':
                    self.calculate_shootGoal()
                    
                elif self.key == 'p':
                    self.calculate_passGoal()
                    self.msg_selfToBuddy('BallPass')
                
                elif self.key == 's': # simulate shooting out the ball
                    self.selfBallPosession = False
                
                if self.selfBallPosession is False:
                    self.mainFSM = 'waitBall'
                
                return
                
            case 'waitBall': # Wait ball mode
                # If received the ball pass flag, rotates the ball receiver toward the buddy robot.
                if self.receiveBallCmdFlag is True:
                    self.receiveBallFlag = False
                    # self.calculate_passOrient()
                
                if self.key == 'b':
                    if self.buddyBallPossesion is False:
                        self.selfBallPosession = True
                
                # Check self ball possesion set by detection sensor
                if self.selfBallPosession is True:
                    self.msg_selfToBuddy('BallReceived')
                    self.mainFSM = 'shootBall'
                    
                return
                
            case _:
                self.mainFSM = 'idle'

    def timer_callback(self):
        self.draw_hoopMarker()
        self.key = getKey()

        if self.key == 'c':
            rclpy.shutdown()

        if self.tf_selfListener() or self.tf_buddyListener():
            self.get_logger().warning('Lookup transform error, will skip this cycle...')
            return

        if self.key == 'h':
            self.calculate_poseHome()
        elif self.key == 'g':
            self.msg_selfToBuddy('PossesState')

        self.abu_gameplayFSM()
        self.draw_possesionMarker()
        
        return

def getKey():
    tty.setraw(sys.stdin.fileno())
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