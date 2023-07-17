import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('mindvision_camera'), 'config', 'camera_params.yaml')
    
    # load params for composable node
    with open(params_file, 'r') as f:
        camera_params = yaml.safe_load(f)['/mv_camera']['ros__parameters']

    return LaunchDescription([
        Node(
            package='mindvision_camera',
            executable='mindvision_camera_node',
            name='camera_node',
            # output='screen',
            # emulate_tty=True,
            parameters=[camera_params, {'use_sensor_data_qos': False}],
        )
    ])
