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
    
    # Configure node launch information 
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    # Map resolution
    resolution = LaunchConfiguration('resolution', default='0.05')
    # Map publish period  
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')
    # Configuration file folder path
    configuration_directory = LaunchConfiguration('configuration_directory',default= os.path.join(get_package_share_directory(pkg_name), 'params_global') )
    # Configuration file
    configuration_basename_r1 = LaunchConfiguration('configuration_basename_r1', default='R1_localize_sim.lua')
    configuration_basename_r2 = LaunchConfiguration('configuration_basename_r2', default='R2_localize_sim.lua')

    # Cartographer SLAM on R1

    cartographer_node_r1 = launch_ros.actions.Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node_r1',
        namespace='r1',
        output='screen',
        remappings=[
            ('/imu', '/imu/data_r1')
            ],
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-configuration_directory', configuration_directory,
                   '-configuration_basename', configuration_basename_r1,
                   '-load_state_filename', os.path.join(get_package_share_directory(pkg_name), 'maps', 'abu2025_map.pbstream'),
                   '-load_frozen_state true']
    )

    cartographer_occupancy_grid_node_r1 = launch_ros.actions.Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node_r1',
        namespace='r1',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec]
    )

    # Cartographer SLAM on R2
    cartographer_node_r2 = launch_ros.actions.Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node_r2',
        namespace='r2',
        output='screen',
        remappings=[
            ('/imu', '/imu/data_r2')
            ],
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-configuration_directory', configuration_directory,
                   '-configuration_basename', configuration_basename_r2,
                   '-load_state_filename', os.path.join(get_package_share_directory(pkg_name), 'maps', 'abu2025_map.pbstream'),
                   '-load_frozen_state true']
    )

    cartographer_occupancy_grid_node_r2 = launch_ros.actions.Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node_r2',
        namespace='r2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec]
    )


    return launch.LaunchDescription([
        cartographer_node_r1,
        cartographer_occupancy_grid_node_r1,
        cartographer_node_r2,
        cartographer_occupancy_grid_node_r2,
    ])
