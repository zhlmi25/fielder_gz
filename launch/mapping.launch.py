import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import yaml

def generate_launch_description():
    # Get the package share directory and map path
    package_share = get_package_share_directory('fielder_gz')
    map_path = os.path.join(package_share, 'map')
    
    # Set environment variable for the map path
    os.environ['FIELDER_GZ_MAP_PATH'] = map_path
    
    # Read YAML config and expand environment variables
    config_file = os.path.join(package_share, 'config', 'mapper_params_online_async.yaml')
    
    with open(config_file, 'r') as f:
        config = yaml.safe_load(f)
    
    # Expand environment variables in any paths that may contain them
    if 'slam_toolbox' in config and 'ros__parameters' in config['slam_toolbox']:
        params = config['slam_toolbox']['ros__parameters']
        if 'map_file_name' in params and params['map_file_name']:
            params['map_file_name'] = os.path.expandvars(params['map_file_name'])
    
    # Write modified config to temporary file
    temp_config = '/tmp/mapper_params_online_async.yaml'
    with open(temp_config, 'w') as f:
        yaml.dump(config, f)

    launch_slam = IncludeLaunchDescription(
         PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'slam_params_file': temp_config,
            'use_sim_time': 'true'
        }.items()
    )


    # Run the nodes
    return LaunchDescription([
        launch_slam
    ])