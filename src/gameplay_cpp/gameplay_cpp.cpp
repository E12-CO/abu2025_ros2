//ABU 2025 semi-auto gameplay system for R1 and R2 robot
//Coded by TinLethax (RB26)

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <string>
#include <iostream>
#include <stdexcept>

// ROS2 lib
#include <rclcpp/rclcpp.hpp>

// tf2 lib
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include "tf2/utils.h"

// Standard message
#include <std_msgs/msg/string.hpp>

// Geometry message
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>

// Sensor message
#include <sensor_msgs/msg/joy.hpp>

// Visualization message
#include <visualization_msgs/msg/marker.hpp>

// iRob command message
#include "irob_msgs/msg/irob_cmd_msg.hpp"

// Define Feedback Loop time 
#define LOOP_TIME_MIL   20 // 20 millisec -> 50Hz
#define LOOP_TIME_SEC	LOOP_TIME_MIL/1000 // Loop time in second

class gameplay_abu2025 : public rclcpp::Node{
	public:
	
	std::string selfNamespace;
	
	// Status flags
	bool selfBallPosession = false;
	
	bool cartoStarted = false;
	
	bool fieldOriented			= true;
	bool toggleLockShootGoal	= false;
	bool autoShootGoal			= false;
	
	
	// tf2 related
	bool localizationLostFlag = false;
	
	// Self transform listener
	std::shared_ptr<tf2_ros::TransformListener> tf_listener_self{nullptr};
	std::unique_ptr<tf2_ros::Buffer> 		tf_buffer_self;
	geometry_msgs::msg::TransformStamped 	selfPose;
	tf2::Quaternion self_quat_tf;
	double selfOrientation = 0.0;
	
	// Buddy transform listener
	std::shared_ptr<tf2_ros::TransformListener> tf_listener_buddy{nullptr};
	std::unique_ptr<tf2_ros::Buffer> 		tf_buffer_buddy;
	geometry_msgs::msg::TransformStamped	buddyPose;
	// tf2::Quaternion self_quat_tf;
	// double selfOrientation = 0.0;

	std::string self_robot_frame;
	std::string	buddy_robot_frame;
	
	// Inter-robot communication message
	// to buddy
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr	pubSelfToBud;
	std_msgs::msg::String	toBuddyMsg;
	// from buddy
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subBudToSelf;
	std_msgs::msg::String	fromBuddyMsg; 
	
	// Node-red signals
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr	subEmer;
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr	subSS;
	
	// Visualization markers
	rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr	pubHoopMark;
	rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr	pubShootMark;
	rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr	pubPossesMark;
	visualization_msgs::msg::Marker	hoopMark;
	visualization_msgs::msg::Marker	shootMark;
	visualization_msgs::msg::Marker	possesMark;
	
	// iRob related
	rclcpp::Publisher<irob_msgs::msg::IrobCmdMsg>::SharedPtr	pubiRobCmd;
	irob_msgs::msg::IrobCmdMsg									iRobCmdMsg;
	rclcpp::Subscription<irob_msgs::msg::IrobCmdMsg>::SharedPtr	subiRobStatus;
	bool irobHaltStatus = false;
	
	// Goal publisher
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr	pubGoal;
	geometry_msgs::msg::PoseStamped									goalPose;
	
	// Joy related
	// Joy subscriber
	rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr			subJoy;
	// Joy Twist message
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr					pubManualVel;
	geometry_msgs::msg::Twist										cmdTwist;
	
	// Parameters
	double shoot_radius;
	double vxy_max;
	double vz_max;
	
	double hoop_px, hoop_py;
	
	// Internal variables
	double vel_magnitude;
	double vel_heading;
	double vel_az;
	
	double v_vector;
	
	
	double rotate_kp;
	double rotate_kd;
	double self_to_hoop_angle;
	double s2h_error;
	double s2h_velAz;
	double s2h_prevError;
	
	
	// Wall timer for PID control loop 50Hz
	rclcpp::TimerBase::SharedPtr timer_;
	
	gameplay_abu2025() : Node("abu2025_gameplay"){
		selfNamespace = this->get_namespace();
		
		// Parameters
		
		declare_parameter("self_robot_frame", "base_link_r1");
		declare_parameter("buddy_robot_frame", "base_link_r2");
		
		declare_parameter("shoot_radius", 4.1);
		declare_parameter("v_xy_max", 1.1);
		declare_parameter("v_az_max", 1.57);
		
		declare_parameter("hoop_position_x", 13.0);
		declare_parameter("hoop_position_y", -3.0);
		
		declare_parameter("rotate_kp", 2.1);
		declare_parameter("rotate_kd", 0.001);
		
		get_parameter("self_robot_frame", self_robot_frame);
		get_parameter("buddy_robot_frame", buddy_robot_frame);
		
		get_parameter("shoot_radius", shoot_radius);
		get_parameter("v_xy_max", vxy_max);
		get_parameter("v_az_max", vz_max);
		
		get_parameter("hoop_position_x", hoop_px);
		get_parameter("hoop_position_y", hoop_py);
		
		get_parameter("rotate_kp", rotate_kp);
		get_parameter("rotate_kd", rotate_kd);
		
		// Self and buddy listener
		tf_buffer_self = 
			std::make_unique<tf2_ros::Buffer>(this->get_clock(), tf2::durationFromSec(1.0));
		tf_listener_self =
			std::make_shared<tf2_ros::TransformListener>(*tf_buffer_self);
		
		tf_buffer_buddy = 
			std::make_unique<tf2_ros::Buffer>(this->get_clock(), tf2::durationFromSec(1.5));
		tf_listener_buddy =
			std::make_shared<tf2_ros::TransformListener>(*tf_buffer_buddy);
		
		// Inter-robot communication
		pubSelfToBud = create_publisher<std_msgs::msg::String>("to_buddy", 10);
		
		subBudToSelf = 
			create_subscription<std_msgs::msg::String>(
			"from_buddy",
			10,
			std::bind(
				&gameplay_abu2025::msgFromBuddyCallback,
				this,
				std::placeholders::_1)
			);
			
		// Node-red signals
		rclcpp::QoS qos_profile(rclcpp::KeepLast(10));
		qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
		
		// Soft Emergency
		subEmer = 
			create_subscription<std_msgs::msg::String>(
			"/soft_emergency",
			qos_profile,
			std::bind(
				&gameplay_abu2025::softEmerCallback,
				this,
				std::placeholders::_1)
			);
			
		// Cartographer Start-Stop
		subSS	=
			create_subscription<std_msgs::msg::String>(
			"/start_stop",
			qos_profile,
			std::bind(
				&gameplay_abu2025::ssCallback,
				this,
				std::placeholders::_1)
			);
			
		// Visualization
		pubHoopMark = create_publisher<visualization_msgs::msg::Marker>("hoopmark", 10);
		pubShootMark = create_publisher<visualization_msgs::msg::Marker>("shootmark", 10);
		pubPossesMark = create_publisher<visualization_msgs::msg::Marker>("possesion", 10);
		
		// iRob
		pubiRobCmd = create_publisher<irob_msgs::msg::IrobCmdMsg>("irob_cmd", 10);
		subiRobStatus = 
			create_subscription<irob_msgs::msg::IrobCmdMsg>(
			"irob_stat",
			10,
			std::bind(
				&gameplay_abu2025::irobStatCallback,
				this,
				std::placeholders::_1)
			);
		pubGoal = create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 10);
		
		// Joy related
		pubManualVel = create_publisher<geometry_msgs::msg::Twist>("cmd_vel_manual", 10);
		
		subJoy = 
			create_subscription<sensor_msgs::msg::Joy>(
			"joy",
			10,
			std::bind(
				&gameplay_abu2025::joyCallback,
				this,
				std::placeholders::_1)
			);
		
		// Wall timer 
		timer_ = 
			this->create_wall_timer(
				std::chrono::milliseconds(LOOP_TIME_MIL),
				std::bind(
					&gameplay_abu2025::ABURunner, 
					this)
			);
		
		RCLCPP_INFO(
			this->get_logger(),
			"Robot Club Engineering KMITL : %s ABU 2025 CPP Gameplay System",
			selfNamespace.c_str()
		);
	}
	
	int tf_selfListener(){
		if(
			!tf_buffer_self->canTransform(
			"map",
			self_robot_frame,
			tf2::TimePointZero,
			tf2::durationFromSec(0.1)
			)
		){
			return 1;
		}
		
		try{
			selfPose = 
				tf_buffer_self->lookupTransform(
					"map", 
					self_robot_frame, 
					tf2::TimePointZero,
					tf2::durationFromSec(0.1)
				);
			tf2::fromMsg(selfPose.transform.rotation, self_quat_tf);
			selfOrientation = tf2::getYaw(self_quat_tf);
			
		} catch (const tf2::TransformException & ex){
			
			return 1;
		}
		
		return 0;
	}
	
	int tf_buddyListener(){
		if(
			!tf_buffer_self->canTransform(
			"map",
			buddy_robot_frame,
			tf2::TimePointZero,
			tf2::durationFromSec(0.5)
			)
		){
			return 1;
		}
		
		try{
			buddyPose = 
				tf_buffer_buddy->lookupTransform(
					"map", 
					buddy_robot_frame, 
					tf2::TimePointZero,
					tf2::durationFromSec(0.5)
				);
			// tf2::fromMsg(buddyPose.transform.rotation, buddy_quat_tf);
			// buddyOrientation = tf2::getYaw(buddy_quat_tf);
			
		} catch (const tf2::TransformException & ex){
			
			return 1;
		}
		
		return 0;
	}
	
	void draw_hoopMarker(){
		hoopMark.header.frame_id = "map";
		hoopMark.header.stamp = this->get_clock()->now();
		hoopMark.id = 0;
		hoopMark.type = hoopMark.SPHERE;
		hoopMark.action = hoopMark.ADD;
		hoopMark.scale.x = 0.5;
		hoopMark.scale.y = 0.5;
		hoopMark.color.a = 1.0;
        hoopMark.color.r = 1.0;
        hoopMark.color.r = 0.5;
        hoopMark.pose.position.x = hoop_px;
        hoopMark.pose.position.y = hoop_py;
        hoopMark.pose.position.z = 0.5;
        pubHoopMark->publish(hoopMark);
	}
	
	void draw_shootMarker(double x, double y){
		shootMark.header.frame_id = "map";
		shootMark.header.stamp = this->get_clock()->now();
		shootMark.id = 0;
        shootMark.type = shootMark.SPHERE;
        shootMark.action = shootMark.ADD;
        shootMark.scale.x = 0.3;
        shootMark.scale.y = 0.3;
        shootMark.scale.z = 0.3;
        shootMark.color.a = 1.0;
        shootMark.color.g = 1.0;
        shootMark.pose.position.x = x;
        shootMark.pose.position.y = y;
        shootMark.pose.position.z = 0.5;
        pubShootMark->publish(hoopMark);
	}
	
	void draw_possesionMarker(){
		possesMark.header.frame_id = "map";
        possesMark.header.stamp = this->get_clock()->now();
        possesMark.id = 0;
        possesMark.type = possesMark.SPHERE;
        if (selfBallPosession == true)
            possesMark.action = possesMark.ADD;
        else
            possesMark.action = possesMark.DELETE;
            
        possesMark.scale.x = 0.3;
        possesMark.scale.y = 0.3;
        possesMark.scale.z = 0.3;
        possesMark.color.a = 1.0;
        possesMark.color.g = 0.4;
        possesMark.color.r = 1.0;
        possesMark.pose.position.x = selfPose.transform.translation.x + 0.5;
        possesMark.pose.position.y = selfPose.transform.translation.y - 0.5;
        possesMark.pose.position.z = 0.5;
		pubPossesMark->publish(possesMark);
	}
	
	void msgToBuddy(std::string msgToBuddy){
		toBuddyMsg.data = msgToBuddy;
		pubSelfToBud->publish(toBuddyMsg);
	}
	
	void msgFromBuddyCallback(const std_msgs::msg::String::SharedPtr msg){
		
		
	}
	
	void irobSendCmd(std::string irobMsgStr){
		iRobCmdMsg.irobcmd = irobMsgStr;
		pubiRobCmd->publish(iRobCmdMsg);
	}
	
	void irobStatCallback(const irob_msgs::msg::IrobCmdMsg::SharedPtr msg){
		RCLCPP_INFO(
			this->get_logger(),
			"Got iRob status message! : %s",
			msg->irobcmd.c_str()
		);
		
		if(msg->irobcmd == "starting"){
			irobHaltStatus = false;
		}else if(msg->irobcmd == "done"){
			irobHaltStatus = true;
		}else if(msg->irobcmd == "failed"){
			irobHaltStatus = true;
		}else if(msg->irobcmd == "canceled"){
			irobHaltStatus = true;
		}	
		
	}
	
	void softEmerCallback(std_msgs::msg::String::SharedPtr msg){
		irobSendCmd("stop");
		RCLCPP_WARN(
			this->get_logger(),
			"Got the soft EMERGENCY signal!"
		);
	}
	
	void carto_startNode(){
		if(selfNamespace == "/r1")
			std::system("ros2 launch abu2025_ros2 r1_localize.launch.py &");
		else if(selfNamespace == "/r2")
			std::system("ros2 launch abu2025_ros2 r2_localize.launch.py &");
		else
			return;
	}
	
	void carto_killNode(){
		std::system("pkill -f cartographer_node & pkill -f cartographer_occupancy_grid_node");
	}
	
	void ssCallback(std_msgs::msg::String::SharedPtr msg){
		if(msg->data == "start"){
			if(cartoStarted == true)
				return;
			
			RCLCPP_INFO(
				this->get_logger(),
				"Starting Cartographer ROS"
			);
			
			cartoStarted = true;
			
			carto_startNode();
			
		}else if(msg->data == "stop"){
			if(cartoStarted == false)
				return;
			
			RCLCPP_INFO(
				this->get_logger(),
				"Stopping Cartographer ROS"
			);
			
			cartoStarted = false;
			
			carto_killNode();
		}else{
			return;
		}
	}
	
	void calculate_shootGoal(){
		RCLCPP_INFO(
			this->get_logger(),
			"Calculating shoot pose..."
		);
		
		self_to_hoop_angle = atan2(
			(hoop_px - selfPose.transform.translation.y),
			(hoop_py - selfPose.transform.translation.x)
		);
		
		goalPose.pose.position.x = 
			shoot_radius * cos(self_to_hoop_angle + 3.141593);
		goalPose.pose.position.y = 
			shoot_radius * sin(self_to_hoop_angle + 3.141593);
			
		if(goalPose.pose.position.y > 0.0)
			goalPose.pose.position.y = 0.0;
		if(goalPose.pose.position.y < -6.0)
			goalPose.pose.position.y = -6.0;
		
		draw_shootMarker(
			goalPose.pose.position.x,
			goalPose.pose.position.y
		);
		
		tf2::Quaternion xyz_angular;
		xyz_angular.setRPY(0, 0, self_to_hoop_angle);
		goalPose.pose.orientation.x = xyz_angular.getX();
		goalPose.pose.orientation.y = xyz_angular.getY();
		goalPose.pose.orientation.z = xyz_angular.getZ();
		goalPose.pose.orientation.w = xyz_angular.getW();
		
		goalPose.header.frame_id 	= "map";
		goalPose.header.stamp 		= this->get_clock()->now();
		
		pubGoal->publish(goalPose);
	}
	
	double calculate_shootOrientation(){
		self_to_hoop_angle = atan2(
			(hoop_px - selfPose.transform.translation.y),
			(hoop_py - selfPose.transform.translation.x)
		);
		
		s2h_error = self_to_hoop_angle - selfOrientation;
		
		s2h_velAz = 
			(s2h_error * rotate_kp) +
			((s2h_error - s2h_prevError) * rotate_kd);
		
		s2h_prevError = s2h_error;
		
		return s2h_velAz;
	}
	
	void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg){
		// Check M1. Field oriented vs Robot oriented control
		if(msg->buttons[0] > 0){
			if(
				(localizationLostFlag == false) &&
				(fieldOriented == false)
			){
				RCLCPP_INFO(
					this->get_logger(),
					"Field oriented ON!"
				);
				fieldOriented = true;
			}
		}else{
			if(fieldOriented == true){
				RCLCPP_INFO(
					this->get_logger(),
					"Field oriented OFF!"
				);
				fieldOriented = false;
			}
			
		}
		
		// Check A1. Active Hoop locking
		if(msg->buttons[6] > 0){
			if(
				(localizationLostFlag == false) &&
				(toggleLockShootGoal == false) &&
				(fieldOriented == true)
			){
				toggleLockShootGoal = true;
				irobSendCmd("stop");
			}

		}else{
			if(toggleLockShootGoal == true)
				toggleLockShootGoal = false;
			
		}
		
		// Check A2. Automatic park at nearest shoot goal
		if(msg->buttons[7] > 0){
			if(
				(localizationLostFlag == false) &&
				(irobHaltStatus == false)
			){
				calculate_shootGoal();
			}
			
		}else{
			irobHaltStatus = true;
			irobSendCmd("stop");
		}
		
		vel_magnitude = 
			sqrt(
				pow(msg->axes[0], 2) +
				pow(msg->axes[1], 2)
			) * vxy_max;
		vel_heading = atan2(msg->axes[1], msg->axes[0]);
		vel_az = msg->axes[3];
		
	}
	
	void joyRunner(){
		v_vector = fieldOriented ? (vel_heading - selfOrientation) : vel_heading;
		
		cmdTwist.linear.x 	= vel_magnitude * cos(v_vector);
		cmdTwist.linear.y	= vel_magnitude * sin(v_vector);
		
		if(toggleLockShootGoal){
			cmdTwist.angular.z = calculate_shootOrientation();
			pubManualVel->publish(cmdTwist);
		}else{
			if(
				(abs(vel_magnitude) > 0.05) ||
				(abs(vel_az) > 0.1)
			){
				if(irobHaltStatus == false){
					irobHaltStatus = true;
					irobSendCmd("stop");
				}
				
				cmdTwist.angular.z = vz_max * vel_az;
				pubManualVel->publish(cmdTwist);
			}
		}
		
	}
	
	void ABURunner(){
		
		if(tf_selfListener()){
			localizationLostFlag = true;
		}else{
			if(localizationLostFlag)
				localizationLostFlag = false;
		}
		
		joyRunner();
		//draw_possesionMarker();
	}
	
};

int main (int argc, char **argv){
	rclcpp::init(argc, argv);
	auto gameplay {std::make_shared<gameplay_abu2025>()};
	rclcpp::spin(gameplay);
	rclcpp::shutdown();	
}