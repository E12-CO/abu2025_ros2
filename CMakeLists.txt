cmake_minimum_required(VERSION 3.8)
project(abu2025_ros2)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -Wextra -Wpedantic -g")

# find dependencies
# Default ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(xacro REQUIRED)
# ROS2 dependencies for ROS2 nodes
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)

# Robot specific dependencies
# iRob packages
find_package(irob_msgs REQUIRED)
find_package(irob_interface REQUIRED)
find_package(irob_controller REQUIRED)
find_package(irob_maneuv3r REQUIRED)

# Sensors packages
find_package(ust_05ln_ros2 REQUIRED)

# SLAM packages
find_package(cartographer_ros REQUIRED)

# set dependencies for the abu joy node
set(abu_joy_dependencies
	rclcpp
	rclcpp_components
	sensor_msgs
)

# Install python scripts

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
 scripts/gameplay.py
 scripts/socketABU_Pose.py
 DESTINATION lib/${PROJECT_NAME}
)

# ABU Joy node
add_executable(abu_joyInterface
	src/abu_joyInterface/abu_joyInterface.cpp
)

target_link_libraries(
	abu_joyInterface
)

ament_target_dependencies(
	abu_joyInterface
	${abu_joy_dependencies}
)

install(
	TARGETS abu_joyInterface
	DESTINATION lib/${PROJECT_NAME}
)

# Exporting environment hook got the Gazebo sim
# Following hooks are used to ensure that the correct environment variables
# will be set by exectuting 'sourece install/setup.bash' after compilation.
# When using this template for your project, change the filenames of the
# files in the 'hooks' folder, to correspond to your project name.
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env_hooks/${PROJECT_NAME}.dsv.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env_hooks/${PROJECT_NAME}.sh.in")


install(
 DIRECTORY launch R1 R2 params_global models meshes maps worlds scripts rviz
 DESTINATION share/${PROJECT_NAME}
)


ament_package()
