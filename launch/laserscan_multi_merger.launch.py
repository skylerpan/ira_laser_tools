import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='False')
    params_file  = LaunchConfiguration(
            'params',
            default=os.path.join(
                get_package_share_directory('ira_laser_tools'),
                'cfg',
                'params.yaml')
            )
    return LaunchDescription([
        Node(
            package='ira_laser_tools',
            node_executable='laserscan_multi_merger',
            node_name='laser_merger',
            parameters=[params_file , {'use_sim_time':use_sim_time}],
            output='screen',
        ),
    ])
