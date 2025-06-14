#!/usr/bin/env python3
# ABU 2025 semi-auto gameplay system for R1 and R2 robot
# Coded by TinLethax (RB26)

# on R1 run
# ros2 run abu2025_ros2 gameplay.py --ros-args -r __ns:=/r1  -r to_buddy:=/r2/from_buddy

# on R2 run
# ros2 run abu2025_ros2 gameplay.py --ros-args -r __ns:=/r2 -p "self_robot_frame:=base_link_r2" -p "buddy_robot_frame:=base_link_r1" -r to_buddy:=/r1/from_buddy

import math
import sys
import subprocess
import os, signal, sys, select, termios, tty

import transforms3d
import rclpy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
import tf2_ros
import tf2_geometry_msgs
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from sensor_msgs.msg import Joy
from visualization_msgs.msg import Marker
from irob_msgs.msg import IrobCmdMsg

#settings = termios.tcgetattr(sys.stdin)

class abugameplay(Node):
    def __init__(self):
        super().__init__('abu2025_gameplay')

        # FSM
        self.mainFSM = 'idle'
        # Physical Emergency button pressed flag
        self.emerFlag = False
        
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
        self.tf_buffer_self = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=1.0))
        self.tf_listener_self = tf2_ros.TransformListener(self.tf_buffer_self, self)
        self.selfPose = TransformStamped()
        self.selfOrientation = 0.0
        self.localizationLostFlag = False
        
        # buddy Transform Listener
        self.tf_buffer_buddy = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=1.5))
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
        self.irobHaltStatus = False
        
        # Node-red signal subscribers
        # note that the topic name is absolute path with /, indication that it's overriding the namespace.
        
        # Emergency soft-stop topic 
        self.emerSub_ = self.create_subscription(String, '/soft_emergency', self.softEmerCallback, QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT))
        
        # Start and Stop topic
        self.ssSub_ = self.create_subscription(String, '/start_stop', self.ssCallback, QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT))
        self.cartoStarted = False
        
        # (Joy) Manual control
        # Cmd vel publisher (to the twist_mux)
        self.manualVelPub_ = self.create_publisher(Twist, 'cmd_vel_manual', 10)
        # Sub to abu_joyInterface node
        self.joySub_ = self.create_subscription(Joy, 'joy', self.joyCallback, 10)
        
        self.fieldOriented = True # Default enable
        self.toggleLockShootGoal = False
        self.autoShootGoal  = False
        
        self.cmd_twist = Twist()
        
        self.Vxy_max = 1.1 # Max linear velocity of 1.1m/s
        self.Vz_max  = 1.57 # Max angular Z velocity of 1.57 rad/s (90 deg/s)
        
        self.vel_magnitude  = 0.0
        self.vel_heading    = 0.0
        self.vel_az         = 0.0
        
        self.s2h_prevError  = 0.0
        
        # rViz markers
        # Hoop pose
        self.hoopPose = PoseStamped()
        self.hoopPose.pose.position.x = 13.0
        self.hoopPose.pose.position.y = -3.0
        # Shoot radius, 4.2 meters
        self.shootRadius = 4.2 

        # Goal pose 
        self.goalPose = PoseStamped()
        
        self.timer = self.create_timer(0.02, self.timer_callback) # 50Hz wall timer
        # self.serialTimer = self.create_timer(0.05, self.serial_callback)
        self.get_logger().info(f"Robot Club Engineering KMITL : {self.selfNamespace} ABU2025 gameplay system...")

    # Convert quaternion to euler yaw
    def quat_to_yaw(self, x, y, z, w):
        (row, pitch, yaw) = transforms3d.euler.quat2euler([w, x, y, z], 'sxyz')
        return yaw

    # Listen for self transform map --> rX_base_link
    def tf_selfListener(self):
        if not self.tf_buffer_self.can_transform("map", self.self_robot_frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.05)):
            #self.get_logger().warning('Can\'t get self robot transform right now...')
            return 1
            
        try:
            self.selfPose = self.tf_buffer_self.lookup_transform(
                 "map",
                 self.self_robot_frame,
                 rclpy.time.Time(),
                 timeout=rclpy.duration.Duration(seconds=0.05)
                )
            
            self.selfOrientation = self.quat_to_yaw(
                                self.selfPose.transform.rotation.x,
                                self.selfPose.transform.rotation.y,
                                self.selfPose.transform.rotation.z,
                                self.selfPose.transform.rotation.w
                                )
            
       	except(
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            #self.get_logger().warning('Self Transform Lookup failed!, Check Cartographer node!')
            return 1
        
        return 0
    
    # Listen for buddy transform map --> rX_base_link
    def tf_buddyListener(self):    
        if not self.tf_buffer_buddy.can_transform("map", self.buddy_robot_frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.5)):
            #self.get_logger().warning('Can\'t get buddy robot transform right now...')
            return 1
            
        try:
            self.buddyPose = self.tf_buffer_buddy.lookup_transform(
                 "map",
                 self.buddy_robot_frame,
                 rclpy.time.Time(),
                 timeout=rclpy.duration.Duration(seconds=0.5)
                )
            
       	except(
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            #self.get_logger().warning('Buddy Transform Lookup failed!, Check Cartographer node!')
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
        possesMark.header.frame_id = 'map'
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
        possesMark.pose.position.x = self.selfPose.transform.translation.x + 0.5
        possesMark.pose.position.y = self.selfPose.transform.translation.y - 0.5
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
        
        if msg.data == 'BallPass':
            self.get_logger().info('Moving to ball pass receive position')
            if self.receiveBallCmdFlag is False:
                self.receiveBallCmdFlag = True
                
        elif msg.data == 'BallReceived':
            self.get_logger().info('Buddy received ball pass')
            self.buddyBallPossesion = True
            
        elif msg.data == 'PossesState':
            self.get_logger().info('Buddy asked for ball posession status, replying...')
            if self.selfBallPosession is True:
                self.msg_selfToBuddy('YesBall')
            else:
                self.msg_selfToBuddy('NoBall')
            
        elif msg.data == 'YesBall':
            self.get_logger().info('Buddy has ball!')
            self.buddyBallPossesion = True 
            
        elif msg.data == 'NoBall':
            self.get_logger().info('Buddy has no ball!')
            self.buddyBallPossesion = False
            
        elif msg.data == 'Emer':
            self.get_logger().warning('Received emergency stop signal from buddy!')
            # Set the gameplay FSM to idle
            self.mainFSM = 'idle'
            # Stop iRob maneuv3r
            self.irobSendCmd('stop')
            
        else:
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
    
        if msg.irobcmd == 'starting':
            self.irobHaltStatus = False
    
        elif msg.irobcmd == 'done':
            self.irobHaltStatus = True
        
        elif msg.irobcmd == 'failed':
            self.irobHaltStatus = True
        
        elif msg.irobcmd == 'canceled':
            self.irobHaltStatus = True
        
        return
        
    # Soft emergency response
    def softEmerCallback(self, msg):
        # Set the gameplay FSM to idle
        self.mainFSM = 'idle'
        # Stop iRob maneuv3r
        self.irobSendCmd('stop')
        
        self.get_logger().warning('Got the soft EMERGENCY signal!') 
        
    # Start and Stop button callback
    def ssCallback(self, msg):
        if msg.data == 'start':
            if self.cartoStarted == True:
                return
            self.cartoStarted = True
            
            self.get_logger().info('Starting Cartographer ROS')
            # Start cartographer 
            self.carto_startNode()
            
        elif msg.data == 'stop':
            if self.cartoStarted == False:
                return
            self.cartoStarted = False
            
            self.get_logger().info('Stopping Cartographer ROS')
            # Kill cartographer
            self.carto_killNode()
            
        else:
            return
    
    # Joystick callback
    def joyCallback(self, msg):
    
        # Check M1. Field oriented vs Robot oriented control
        if bool(msg.buttons[0]) is True:
            if (self.localizationLostFlag is False) and (self.fieldOriented is False):
                self.get_logger().info('Field oriented ON!')
                self.fieldOriented = True
                
        else:
            if self.fieldOriented is True:
                self.get_logger().info('Field oriented OFF!')
                self.fieldOriented = False
            
        # Check A1. Active Hoop locking
        if (bool(msg.buttons[6])) is True:
            if (self.localizationLostFlag is False) and (self.toggleLockShootGoal is False) and (self.fieldOriented is True):
                self.toggleLockShootGoal = True
                # Set the gameplay FSM to idle
                self.mainFSM = 'idle'
                # Stop iRob maneuv3r
                self.irobSendCmd('stop')
            
        else:    
            if self.toggleLockShootGoal is True:
                self.toggleLockShootGoal = False
             
        # Check A2. Automatic park at nearest shoot goal
        if (bool(msg.buttons[7])) is True:
            if (self.localizationLostFlag is False) and (self.irobHaltStatus is False):
                self.calculate_shootGoal()
                
            else:
                self.irobHaltStatus = False # Clear halt flag
                # Set the gameplay FSM to idle
                self.mainFSM = 'idle'
                # Stop iRob maneuv3r
                self.irobSendCmd('stop')
            
        self.vel_magnitude = math.sqrt(msg.axes[0]**2 + msg.axes[1]**2) * self.Vxy_max  # Velocity magnitude
        self.vel_heading = math.atan2(msg.axes[1], msg.axes[0])                         # Heading angle 
        self.vel_az = msg.axes[3]                                                       # Angular (spin) velocity
    
    def joyRunner(self):
        v_vector = (self.vel_heading - self.selfOrientation) if (self.fieldOriented is True) and (self.localizationLostFlag is False) else self.vel_heading
        
        self.cmd_twist.linear.x = self.vel_magnitude * math.cos(v_vector)
        self.cmd_twist.linear.y = self.vel_magnitude * math.sin(v_vector)  
    
        if (self.toggleLockShootGoal is True) and (self.localizationLostFlag is False):
            # In the case of shoot goal lock mode, always send cmd_vel
            self.cmd_twist.angular.z = self.calculate_shootOrientation()
            self.manualVelPub_.publish(self.cmd_twist)
            
        else:
            # In the case of ordinary control mode, send cmd only when sticks were moved.
            if (abs(self.vel_magnitude) > 0.05) or (abs(self.vel_az) > 0.1):
            
                # Halt the auto park when user take over control.
                if self.irobHaltStatus is False:
                    self.irobHaltStatus = True
                    # Set the gameplay FSM to idle
                    self.mainFSM = 'idle'
                    # Stop iRob maneuv3r
                    self.irobSendCmd('stop')
                    
                self.cmd_twist.angular.z = self.Vz_max * self.vel_az
                self.manualVelPub_.publish(self.cmd_twist)
                
            else:
                return
    
    # PD controller for shoot 
    def calculate_shootOrientation(self):
        # 1. Get current position with lookuptransform
        # 2. Calculate the heading angle from the self to the hoop position, this will be used to command the robot orientation
        self_to_hoop_angle = math.atan2(
                (self.hoopPose.pose.position.y - self.selfPose.transform.translation.y),
                (self.hoopPose.pose.position.x - self.selfPose.transform.translation.x)
            )
         
        s2h_error = self_to_hoop_angle - self.selfOrientation
    
        s2h_velAz = (s2h_error * 2.1) + ((s2h_error - self.s2h_prevError) * 0.001)
        
        self.s2h_prevError = s2h_error
        
        return s2h_velAz
    
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
        
        # 3.5 limit the position not to collide with the gamefield
        if self.goalPose.pose.position.y > 0.0:
            self.goalPose.pose.position.y = 0.0
        elif self.goalPose.pose.position.y < -6.0:
            self.goalPose.pose.position.y = -6.0
        
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
        
        # 4.5 limit the position not to collide with the gamefield
        if self.goalPose.pose.position.y > 0.0:
            self.goalPose.pose.position.y = 0.0
        elif self.goalPose.pose.position.y < -6.0:
            self.goalPose.pose.position.y = -6.0
        
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
       
        
        # 3.5 limit the position not to collide with the gamefield
        if self.goalPose.pose.position.y > 0.0:
            self.goalPose.pose.position.y = 0.0
        elif self.goalPose.pose.position.y < -6.0:
            self.goalPose.pose.position.y = -6.0
        
        self.draw_shootMarker(self.goalPose.pose.position.x, self.goalPose.pose.position.y)
        
        # 4. The heading will be pre-process. As the ball receiver was on the back of the robot
        q = transforms3d.euler.euler2quat(0, 0, heading + 3.141593, 'sxyz')
        self.goalPose.pose.orientation.w = q[0]
        self.goalPose.pose.orientation.x = q[1]
        self.goalPose.pose.orientation.y = q[2]
        self.goalPose.pose.orientation.z = q[3]
        
        # 5. Publish goal_pose
        self.goalPub_.publish(self.goalPose)

    # Calculate home goal pose.
    def calculate_homeGoal(self):
        # 1. pick home pose 
        if self.selfNamespace == '/r1':
            # Home pose of R1 robot
            home_x = 0.0
            home_y = 0.0
        elif self.selfNamespace == '/r2':
            # Home pose of R2 robot
            home_x = 0.0
            home_y = -2.0
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
           
    # Kill cartographer node
    def carto_killNode(self):
        subprocess.run(['pkill', '-f', 'cartographer_node']) # Kill cartographer node
        subprocess.run(['pkill', '-f', 'cartographer_occupancy_grid_node']) # Kill cartographer occupancy grid node
        
    # Launch cartographer node
    def carto_startNode(self):
        if self.selfNamespace == '/r1':
            subprocess.Popen(['ros2', 'launch', 'abu2025_ros2', 'r1_localize.launch.py'])
        elif self.selfNamespace == '/r2':
            subprocess.Popen(['ros2', 'launch', 'abu2025_ros2', 'r2_localize.launch.py'])
        else:
            return
           
    # Gameplay statemachine       
    def abu_gameplayFSM(self):
        if self.mainFSM == 'idle':
            if self.selfBallPosession is True:
                self.mainFSM = 'shootBall'
            else:
                self.mainFSM = 'waitBall'
        
        elif self.main == 'shootBall':
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
        
        elif self.main == 'waitBall':
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
        
        else:
            self.mainFSM = 'idle'

    def timer_callback(self):
        self.draw_hoopMarker()
        # self.key = getKey()

        # if self.key == 'c':
            # rclpy.shutdown()

        if self.tf_selfListener():
            self.localizationLostFlag = True
            #self.get_logger().warning('Self Lookup transform error, will skip this cycle...')
            #return
        else:
            if self.localizationLostFlag == True:
                self.localizationLostFlag = False
                
        #if self.tf_buddyListener():
            #self.get_logger().warning('Buddy Lookup transform error')
        # if self.key == 'h':
            # self.calculate_homeGoal()
        # elif self.key == 'g':
            # self.msg_selfToBuddy('PossesState')

        # if self.emerFlag is True:
            # return

        # self.abu_gameplayFSM()
        self.joyRunner()
        self.draw_possesionMarker()
        
        return
        

# def getKey():
    # tty.setraw(sys.stdin.fileno())
    # key = ''
    # if select.select([sys.stdin], [], [], 0.05) == ([sys.stdin], [], []):
        # key = sys.stdin.read(1)
    # termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    # return key

def main(args=None):
    rclpy.init(args=args)

    node = abugameplay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
