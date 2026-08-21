from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer, Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import Command, LaunchConfiguration
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory

import os
import yaml


def generate_launch_description():
    # 实车能量机关链路：相机 + buff 检测 + 云台控制 + 串口，一条命令跑通
    buff_mode = LaunchConfiguration('buff_mode')
    buff_color = LaunchConfiguration('buff_color')

    declare_buff_mode_cmd = DeclareLaunchArgument(
        'buff_mode', default_value='2', description='1=小符 2=大符')
    declare_buff_color_cmd = DeclareLaunchArgument(
        'buff_color', default_value='1', description='蓝队打红符')

    params_file = os.path.join(
        get_package_share_directory('bringup'), 'config', 'params.yaml')
    with open(params_file, 'r') as f:
        camera_params = yaml.safe_load(f)['/mv_camera']['ros__parameters']
    with open(params_file, 'r') as f:
        controller_params = yaml.safe_load(f)['/gimbal_controller']['ros__parameters']
    with open(params_file, 'r') as f:
        serial_params = yaml.safe_load(f)['/lc_serial_driver']['ros__parameters']

    robot_description = Command(['xacro ', os.path.join(
        get_package_share_directory('rm_description'), 'urdf', 'gimbal_b3.urdf.xacro')])

    # 相机和 buff 检测进同一个容器，图像进程内零拷贝
    camera_buff_container = ComposableNodeContainer(
        name='camera_buff_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='mindvision_camera',
                plugin='mindvision_camera::MVCameraNode',
                name='camera_node',
                parameters=[camera_params, {'use_sensor_data_qos': False}],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='buff_detector',
                plugin='buff_auto_aim::BuffDetectorNode',
                name='buff_detector_node',
                parameters=[{
                    'model_path': os.path.join(
                        get_package_share_directory('buff_detector'), 'model', 'buff.onnx'),
                    'detector_format': 'zju',
                    'buff_mode': buff_mode,
                    'buff_color': buff_color,
                    # 实车在 odom 系跟踪，需要 tf 树 odom←gimbal←camera
                    'target_frame': 'odom',
                    'debug_image': True,
                }],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen',
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description,
                     'publish_frequency': 1000.0}]
    )

    gimbal_controller_node = Node(
        package='gimbal_controller',
        executable='gimbal_controller',
        output='screen',
        emulate_tty=True,
        parameters=[controller_params],
    )

    serial_node = Node(
        package='lc_serial',
        executable='lc_serial_node',
        namespace='',
        output='screen',
        emulate_tty=True,
        parameters=[serial_params],
    )

    delay_serial_node = TimerAction(period=1.0, actions=[serial_node])
    delay_controller_node = TimerAction(period=1.0, actions=[gimbal_controller_node])

    return LaunchDescription([
        declare_buff_mode_cmd,
        declare_buff_color_cmd,
        camera_buff_container,
        robot_state_publisher,
        delay_serial_node,
        delay_controller_node,
    ])
