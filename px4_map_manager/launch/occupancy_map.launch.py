import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('px4_map_manager')
    # Get the path to the parameter file
    param_file = os.path.join(pkg_share, 'cfg', 'occupancy_map_param.yaml')
    
    # Load parameters directly from the YAML file
    with open(param_file, 'r') as f:
        param_dict = yaml.safe_load(f)
    
    # Extract the parameters from the YAML structure
    if '/occupancy_map/occupancy_map_node' in param_dict:
        params = param_dict['/occupancy_map/occupancy_map_node']['ros__parameters']
    else:
        # Fallback in case the YAML structure is different
        params = next(iter(param_dict.values()))['ros__parameters']
    
    # Add use_sim_time parameter
    params['use_sim_time'] = True
    
    # Print the parameters for debugging
    # print("Parameters being loaded:", params)
    
    return LaunchDescription([
        Node(
            package='px4_map_manager',
            executable='occupancy_map_node',
            name='occupancy_map_node',
            namespace='occupancy_map',
            output='screen',
            parameters=[params],
        ),
        Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            parameters=[{'use_sim_time': True}],
            arguments=['-d', os.path.join(pkg_share, 'rviz', 'map.rviz')]
        ),
    ])
