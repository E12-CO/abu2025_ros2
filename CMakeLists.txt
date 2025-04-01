cmake_minimum_required(VERSION 3.8)
project(abu2025_ros2)


# find dependencies
# Default ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(xacro REQUIRED)

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

ament_python_install_package(${PROJECT_NAME})

install(
 DIRECTORY launch R1 R2 params_global models meshes maps worlds scripts
 DESTINATION share/${PROJECT_NAME}
)

# Following hooks are used to ensure that the correct environment variables
# will be set by exectuting 'sourece install/setup.bash' after compilation.
# When using this template for your project, change the filenames of the
# files in the 'hooks' folder, to correspond to your project name.
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env_hooks/${PROJECT_NAME}.dsv.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env_hooks/${PROJECT_NAME}.sh.in")

install(PROGRAMS
 scripts/gameplay.py
 scripts/socketABU_Pose.py
 DESTINATION lib/${PROJECT_NAME}
)

ament_package()
