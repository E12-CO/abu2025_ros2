import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node



def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='abu_holonomic' #<--- CHANGE ME
    
    world_path=os.path.join(get_package_share_directory(package_name), 'worlds/my_world.sdf'),

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_r1 = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'R1',
                                   '-x', '1.0', 
                                   '-y', '-1.0',
                                   '-z', '0.15',
                                   '-Y', '0.0'                                   
                                  ],
                        namespace='r1',
                        output='screen')

    spawn_r2 = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'R2',
                                   '-x', '1.0', 
                                   '-y', '-2.0',
                                   '-z', '0.15',
                                   '-Y', '0.0'                                   
                                  ],
                        namespace='r2',
                        output='screen')


    # Launch them all!
    return LaunchDescription([
        rsp,
        ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so',world_path], output='screen'),
        spawn_r1,
        spawn_r2
    ])
