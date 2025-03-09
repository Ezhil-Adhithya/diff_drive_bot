import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    position = [0.0, 0.0, 0.2]
    orientation = [0.0, 0.0, 0.0]
    robot_base_name = "my_bot"
    urdf_file = 'urdf/my_car.urdf'
    
    package_desc = 'gazebo_urdf_launch'
    print("URDF LOADING..")
    robot_desc_path = os.path.join(get_package_share_directory(package_desc), urdf_file)
    world = os.path.join(get_package_share_directory(package_desc), 'worlds', 'empty.world')
    gazebo_models_path = os.path.join(package_desc, 'models')
    install_dir = get_package_prefix(package_desc)

    
    # Gazebo launch
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', world,
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so'],
        output='screen', emulate_tty=True)

    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_node',
        emulate_tty=True,
        parameters=[{'use_sim_time': True, 'robot_description': Command(['xacro ', robot_desc_path])}],
        output="screen"
    )

    
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=['-entity',
                    'bot',
                   '-x', str(position[0]), '-y', str(position[1]
                                                     ), '-z', str(position[2]),
                   '-R', str(orientation[0]), '-P', str(orientation[1]
                                                        ), '-Y', str(orientation[2]),
                   '-topic', '/robot_description'
                   ]
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', " "]
    )


    return LaunchDescription(
        [
            gazebo,
            robot_state_publisher_node,
            spawn_robot,
            rviz_node,
            
        ]
    )
