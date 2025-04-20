# Launch file for ABU Robocon 2025 Robot
# Coded by TinLethax
import os

import launch
from launch.substitutions import LaunchConfiguration
import launch_ros.actions
import launch_ros.descriptions
import xacro

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

     # Specify the name of the package and path to xacro file within the package
    pkg_name = 'abu2025_ros2'
    file_subpath = 'R2/description_r2/robot_r2.urdf.xacro'
    
    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(pkg_name),file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()
    
    # Configure the node
    node_robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='r2',
        output='screen',
        parameters=[{'robot_description': robot_description_raw}] # add other parameters here if required
    )

    # iRob interface
    irob_interface_instant = launch_ros.actions.Node(
        package='irob_interface',
        executable='iRob_interface',
        namespace='r2',
        output='screen',
        parameters=[os.path.join(get_package_share_directory(pkg_name), 'R2/params_r2', 'R2_iRob_interface_esp32.yaml')]
    )
    
    # iRob controller
    irob_controller_instant = launch_ros.actions.Node(
        package='irob_controller',
        executable='iRob_controller',
        namespace='r2',
        output='screen',
        parameters=[os.path.join(get_package_share_directory(pkg_name), 'R2/params_r1', 'R2_iRob_controller.yaml')]
    )

    # Hokuyo UST laser scanners
    hokuyo_back_instant = launch_ros.actions.Node(
        package='ust_05ln_ros2',
        executable='urg_node',
        name='urg_node_back',
        namespace='r2',
        output='screen',
        parameters=[os.path.join(get_package_share_directory(pkg_name), 'R2/params_r2', 'R2_ust08_front.yaml')]
    )

    hokuyo_front_instant = launch_ros.actions.Node(
        package='ust_05ln_ros2',
        executable='urg_node',
        name='urg_node_front',
        namespace='r2',
        output='screen',
        parameters=[os.path.join(get_package_share_directory(pkg_name), 'R2/params_r2', 'R2_ust08_back.yaml')]
    )

    # Gameplay System
    gameplay_instant = launch_ros.actions.Node(
        package=pkg_name,
        executable='gameplay.py',
        namespace='r2',
        output='screen',
        remappings=[('to_buddy', '/r1/from_buddy')],
        parameters=[
            {'self_robot_frame','base_link_r2'},
            {'buddy_robot_frame','base_link_r1'}
            ]
    )

    return launch.LaunchDescription([
        node_robot_state_publisher,
        irob_interface_instant,
        irob_controller_instant,
        hokuyo_front_instant,
        hokuyo_back_instant,
        gameplay_instant
    ])
