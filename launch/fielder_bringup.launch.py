from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    # Get package share directory
    pkg_share = FindPackageShare('fielder_gz')
    
    # Include fielder_sim.launch.py
    fielder_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                pkg_share,
                'launch',
                'fielder_sim.launch.py'
            ])
        )
    )
    
    # Include localization.launch.py
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                pkg_share,
                'launch',
                'localization.launch.py'
            ])
        )
    )
    
    # Include navigation.launch.py
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                pkg_share,
                'launch',
                'navigation.launch.py'
            ])
        )
    )
    
    return LaunchDescription([
        fielder_sim_launch,
        localization_launch,
        navigation_launch
    ])