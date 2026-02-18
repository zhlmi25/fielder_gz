from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
import os
import xacro

def generate_launch_description():
    pkg_share = get_package_share_directory('fielder_gz')

    default_world = os.path.join(
        pkg_share,
        'world',
        'empty.world'
        )    
    
    world = LaunchConfiguration('world')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='World to load'
        )

    bridge_params = os.path.join(pkg_share,'config','gz_bridge.yaml')
    
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ]
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
            launch_arguments={'gz_args': ['-r -v4 ', world], 'on_exit_shutdown': 'true'}.items()
    )

    display_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'display.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    spawn_robot= Node(package='ros_gz_sim',
                      executable='create',
                      arguments=['-name', 'fielder', 
                                 '-topic', 'robot_description'],
                      output='screen')
    

    return LaunchDescription([
        world_arg,
        gazebo_launch,
        display_launch,
        spawn_robot,
        ros_gz_bridge,
    ])
