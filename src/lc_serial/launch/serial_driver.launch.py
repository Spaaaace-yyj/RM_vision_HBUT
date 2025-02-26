import os


from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import yaml

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('lc_serial'), 'config', 'serial_driver.yaml')
    with open(config, 'r') as f:
        serial_params = yaml.safe_load(f)['/lc_serial_driver']['ros__parameters']

    rm_serial_driver_node = Node(
        package='lc_serial',
        executable='lc_serial_node',
        namespace='',
        output='screen',
        emulate_tty=True,
        parameters=[serial_params],
    )

    return LaunchDescription([rm_serial_driver_node])
