// ABU Robocon 2025 ESP32 Joy-con interface
// By TinLethax at Robot Club KMITL (RB26)

#include <chrono>
#include <cmath>
#include <string>
#include <iostream>
#include <stdexcept>

#include <stdio.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h> 

struct termios tty;

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

// Define Feedback Loop time 
#define LOOP_TIME_MIL   20 // 20 millisec -> 50Hz
#define LOOP_TIME_SEC	LOOP_TIME_MIL/1000 // Loop time in second

// Joy RX com size
#define RX_COM_SIZE		8

typedef struct __attribute__((packed)) {
	uint8_t Header[2];

	union {
		uint8_t moveBtnByte;
		struct {
		  uint8_t move1 : 1;// Field oriented control
		  uint8_t move2 : 1;// Robot oriented control
		  uint8_t move3 : 1;
		  uint8_t move4 : 1;
		  uint8_t res1 : 2;
		  uint8_t set1 : 1;
		  uint8_t set2 : 1;
		} moveBtnBit;
	};

	union {
		uint8_t attackBtnByte;
		struct {
		  uint8_t attack1 : 1;// Left Attack -> go to shoot goal 
		  uint8_t attack2 : 1;// Left Attack -> cancel shhot goal
		  uint8_t attack3 : 1;// Right Attack
		  uint8_t attack4 : 1;// Right Attack
		  uint8_t res1 : 4;
		} attackBtnBit;
	};

	union{
		int8_t stickValue[4];  //joyL_X,joyL_Y ,joyR_X,joyR_Y
		struct{
			int8_t stickLX;// Vel X 
			int8_t stickLY;// Vel Y
			int8_t stickRX;
			int8_t stickRY;// Angular vel z
		} stickByte;
	};
  

} ControllerData_t;

class abu_joy_if : public rclcpp::Node{
	
	public:
	
	std::string serial_port_;
	int serial_port = 0;
	
	ControllerData_t recvControllerData_t;
	
	// cmd_vel twist publisher
	rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr 		pubJoy;
	sensor_msgs::msg::Joy 										joyMsg;	
	
	// Used in wall timer callback
	rclcpp::TimerBase::SharedPtr timer_;
	
	abu_joy_if() : Node("abu_joy"){
		RCLCPP_INFO(
			this->get_logger(),
			"Robot Club KMITL : Starting ABU Joy node..."
		);
		
		declare_parameter("serial_port", "/dev/ESPJOY");
		get_parameter("serial_port", serial_port_);
		
		char *serial_port_file = new char[serial_port_.length() + 1];
		strcpy(serial_port_file, serial_port_.c_str());
		serial_port = open(serial_port_file, O_RDWR);
		// Can't open serial port
		if(serial_port < -1){
			RCLCPP_ERROR(
				this->get_logger(), 
				"Error openning Serial %s", 
				serial_port_.c_str()
				);
			std::raise(SIGTERM);
			return;
		}
		
		if(tcgetattr(serial_port, &tty) != 0){
			RCLCPP_ERROR(
				this->get_logger(), 
				"Error %i from tcgetattr: %s\n", 
				errno, 
				strerror(errno)
				);
			std::raise(SIGTERM);
			return;			
		}
		
		
		tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
		tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
		tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size 
		tty.c_cflag |= CS8; // 8 bits per byte (most common)
		tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
		tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

		tty.c_lflag &= ~ICANON;
		tty.c_lflag &= ~ECHO; // Disable echo
		tty.c_lflag &= ~ECHOE; // Disable erasure
		tty.c_lflag &= ~ECHONL; // Disable new-line echo
		tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
		tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
		tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

		tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
		tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
		// tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
		// tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

		tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
		tty.c_cc[VMIN] = 0;

		// Set in/out baud rate to be 115200
		cfsetispeed(&tty, B115200);
		cfsetospeed(&tty, B115200);
		
		if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
			RCLCPP_ERROR(
				this->get_logger(),
				"Error %i from tcsetattr: %s\n", 
				errno, 
				strerror(errno));
			std::raise(SIGTERM);
			return;
		}
	
		pubJoy = create_publisher<sensor_msgs::msg::Joy>(
			"joy",
			10
		);
	
		joyMsg.header.frame_id = "map";
		
		joyMsg.axes.resize(4);
		joyMsg.buttons.resize(10);
	
		timer_ = 
			this->create_wall_timer(
				std::chrono::milliseconds(LOOP_TIME_MIL),
				std::bind(
					&abu_joy_if::abu_joy_interfaceRunner, 
					this)
			);
	
		RCLCPP_INFO(this->get_logger(), "ABU Joy node started!");
	}
	
	
	void abu_joy_serialFlush(){
		tcflush(
			serial_port,
			TCIOFLUSH
		);
		
		tcflush(
			serial_port,
			TCIOFLUSH
		);		
	}
	
	int rx_bytes = 0;
	
	void abu_joy_interfaceRunner(){
		ioctl(
			serial_port,
			FIONREAD,
			&rx_bytes
		);
		
		if(rx_bytes < RX_COM_SIZE)
			return;
		
		if(rx_bytes > RX_COM_SIZE)
			rx_bytes = RX_COM_SIZE;
		
		read(
			serial_port,
			(unsigned char *)&recvControllerData_t,
			rx_bytes
			);
		
		// Return if header is missing
		if(
			(recvControllerData_t.Header[0] != 'R') ||
			(recvControllerData_t.Header[1] != 'B') 
		)
			return;
			
		// Process Joystick data
		joyMsg.header.stamp = this->get_clock()->now();
		
		joyMsg.axes[0] = (float)recvControllerData_t.stickByte.stickLX / 128;
		joyMsg.axes[1] = (float)recvControllerData_t.stickByte.stickLY / 128;
		joyMsg.axes[2] = (float)recvControllerData_t.stickByte.stickRX / 128;
		joyMsg.axes[3] = (float)recvControllerData_t.stickByte.stickRY / 128;
		
		// Move buttons
		joyMsg.buttons[0] = recvControllerData_t.moveBtnBit.move1;
		joyMsg.buttons[1] = recvControllerData_t.moveBtnBit.move2;
		joyMsg.buttons[2] = recvControllerData_t.moveBtnBit.move3;
		joyMsg.buttons[3] = recvControllerData_t.moveBtnBit.move4;
		// Set buttons
		joyMsg.buttons[4] = recvControllerData_t.moveBtnBit.set1;
		joyMsg.buttons[5] = recvControllerData_t.moveBtnBit.set2;
		// Attack buttons
		joyMsg.buttons[6] = recvControllerData_t.attackBtnBit.attack1;
		joyMsg.buttons[7] = recvControllerData_t.attackBtnBit.attack2;
		joyMsg.buttons[8] = recvControllerData_t.attackBtnBit.attack3;
		joyMsg.buttons[9] = recvControllerData_t.attackBtnBit.attack4;
		
		pubJoy->publish(joyMsg);
	}
	
};

int main (int argc, char **argv){
	rclcpp::init(argc, argv);
	auto abuJoy {std::make_shared<abu_joy_if>()};
	rclcpp::spin(abuJoy);
	rclcpp::shutdown();	
}