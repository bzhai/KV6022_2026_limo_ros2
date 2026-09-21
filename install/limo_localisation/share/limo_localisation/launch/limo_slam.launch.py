import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter

def generate_launch_description():
    ld = LaunchDescription()

    pkg_name = 'limo_localisation'
    slam_params_file = os.path.join(get_package_share_directory(pkg_name),'params','slam.yaml')
   
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam',
        parameters=[slam_params_file],
    )

    
    # Add actions to LaunchDescription
    ld.add_action(SetParameter(name='use_sim_time', value=True))
    ld.add_action(slam_node)

    return ld