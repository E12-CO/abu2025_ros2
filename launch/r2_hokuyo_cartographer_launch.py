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
    file_subpath = 'description_r1/robot.urdf.xacro'
    
    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(pkg_name),file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()
    
    # Configure node launch information 
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    # Map resolution
    resolution = LaunchConfiguration('resolution', default='0.05')
    # Map publish period  
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')
    # Configuration file folder path
    configuration_directory = LaunchConfiguration('configuration_directory',default= os.path.join(get_package_share_directory(pkg_name), 'params') )
    # Configuration file
    configuration_basename = LaunchConfiguration('configuration_basename', default='hokuyo_2d.lua')

    # Configure the node
    node_robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw}] # add other parameters here if required
    )

    # iRob interface
    irob_interface_instant = launch_ros.actions.Node(
        package='irob_interface',
        executable='iRob_interface',
        output='screen',
        parameters=[os.path.join(get_package_share_directory(pkg_name), 'params', 'irob_interface_mpu90.yaml')]
    )
    
    # iRob controller
    irob_controller_instant = launch_ros.actions.Node(
        package='irob_controller',
        executable='irob_controller',
        output='screen',
        parameters=[os.path.join(get_package_share_directory(pkg_name), 'params', 'irob_controller_R1.yaml')]
    )

    # IMU Complementary filter
    imu_filter_instant = launch_ros.actions.Node(
        package='imu_complementary_filter',
        executable='complementary_filter_node',
        name='complementary_filter_gain_node',
        output='screen',
        parameters=[os.path.join(get_package_share_directory(pkg_name), 'params', 'filter_config.yaml')]
    )

    # Hokuyo UST-05LN laser scanner
    hokuyo_instant = launch_ros.actions.Node(
        package='ust_05ln_ros2',
        executable='urg_node',
        output='screen',
        parameters=[os.path.join(get_package_share_directory(pkg_name), 'params', 'ust.yaml')]
    )
    
    # Hokuyo angle filter
    hokuyo_filter_instant = launch_ros.actions.Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        parameters=[os.path.join(get_package_share_directory(pkg_name), 'params', 'hokuyo_angle_filter.yaml')]
    )
    
    # RF2O laser odometry
    rf2o_instant = launch_ros.actions.Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{
            'laser_scan_topic' : '/scan',
            'odom_topic' : '/odom_rf2o',
            'publish_tf' : False,
            'base_frame_id' : 'base_link',
            'odom_frame_id' : 'odom',
            'init_pose_from_topic' : '',
            'freq' : 10.0
        }]
    )

    # Extended Kalman Filter 
    ekf_fusion_instant = launch_ros.actions.Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(get_package_share_directory(pkg_name), 'params', 'ekf.yaml')],
    )

     # Delayed sensor 
    delayed_sensor_instant = launch.actions.TimerAction(
        period=2.0, 
        actions=[
#            hokuyo_filter_instant,
            imu_filter_instant,
#            rf2o_instant,
            ]
    )

    delayed_fusion_instant = launch.actions.TimerAction(
        period=3.0,
        actions=[ekf_fusion_instant]
    )

    cartographer_node = launch_ros.actions.Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        remappings=[
            ('/imu', '/imu/data') 
            ],
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-configuration_directory', configuration_directory,
                   '-configuration_basename', configuration_basename]
    )

    cartographer_occupancy_grid_node = launch_ros.actions.Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec]
    )

    # Cartographer SLAM
    delayed_slam_instant = launch.actions.TimerAction(period=3.0, actions=[cartographer_node, cartographer_occupancy_grid_node])


    return launch.LaunchDescription([
        node_robot_state_publisher,
        irob_interface_instant,
        irob_controller_instant,
        hokuyo_instant,
        delayed_sensor_instant,
#        delayed_fusion_instant,
        delayed_slam_instant
    ])
