from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer, Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression
from launch.substitutions import Command, LaunchConfiguration
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory

import os
import yaml


def generate_launch_description():
    # 实车前哨站链路：相机 + 装甲板检测 + 前哨站目标 + 云台控制 + 串口
    outpost_x = LaunchConfiguration('outpost_x')
    outpost_y = LaunchConfiguration('outpost_y')

    declare_outpost_x_cmd = DeclareLaunchArgument(
        'outpost_x', default_value='0.0',
        description='前哨站在 odom 系的位置，位置门需要它，设 0 关闭位置门')
    serial_protocol = LaunchConfiguration('serial_protocol')
    declare_serial_protocol_cmd = DeclareLaunchArgument(
        'serial_protocol', default_value='cjson',
        description='cjson=普通兵种(lc_serial) binary=哨兵(lc_rm_serial)')
    declare_outpost_y_cmd = DeclareLaunchArgument(
        'outpost_y', default_value='0.0',
        description='前哨站在 odom 系的位置，位置门需要它，设 0 关闭位置门')
    serial_protocol = LaunchConfiguration('serial_protocol')
    declare_serial_protocol_cmd = DeclareLaunchArgument(
        'serial_protocol', default_value='cjson',
        description='cjson=普通兵种(lc_serial) binary=哨兵(lc_rm_serial)')

    params_file = os.path.join(
        get_package_share_directory('bringup'), 'config', 'params.yaml')
    with open(params_file, 'r') as f:
        camera_params = yaml.safe_load(f)['/mv_camera']['ros__parameters']
    with open(params_file, 'r') as f:
        detector_params = yaml.safe_load(f)['/armor_detector']['ros__parameters']
    with open(params_file, 'r') as f:
        controller_params = yaml.safe_load(f)['/gimbal_controller']['ros__parameters']
    with open(params_file, 'r') as f:
        serial_params = yaml.safe_load(f)['/lc_serial_driver']['ros__parameters']

    robot_description = Command(['xacro ', os.path.join(
        get_package_share_directory('rm_description'), 'urdf', 'gimbal_b3.urdf.xacro')])

    # 相机 + 装甲板检测 + 前哨站目标选择进同一个容器
    camera_outpost_container = ComposableNodeContainer(
        name='camera_outpost_container',
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
                package='armor_detector',
                plugin='rm_auto_aim::ArmorDetectorNode',
                name='armor_detector',
                parameters=[detector_params],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='outpost_detector',
                plugin='outpost_auto_aim::OutpostTargetNode',
                name='outpost_target_node',
                parameters=[{
                    # 实车在 odom 系跟踪，需要 tf 树 odom←gimbal←camera
                    'target_frame': 'odom',
                    'camera_frame': 'camera_optical_frame',
                    'slot_z': [1.10, 1.27, 1.45],
                    'z_tol': 0.08,
                    'r_initial': 0.28,
                    'rotate_direction': 0,
                    'outpost_x': outpost_x,
                    'outpost_y': outpost_y,
                    'debug': True,
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
        condition=IfCondition(PythonExpression(["'", serial_protocol, "' == 'cjson'"])),
    )
    # 哨兵：二进制帧协议（921600，CRC16），和电控烧的协议必须一致
    rm_serial_node = Node(
        package='lc_serial_test',
        executable='lc_serial_test',
        namespace='',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(PythonExpression(["'", serial_protocol, "' == 'binary'"])),
    )

    delay_serial_node = TimerAction(period=1.0, actions=[serial_node])
    delay_rm_serial_node = TimerAction(period=1.0, actions=[rm_serial_node])
    delay_controller_node = TimerAction(period=1.0, actions=[gimbal_controller_node])

    return LaunchDescription([
        declare_outpost_x_cmd,
        declare_outpost_y_cmd,
        declare_serial_protocol_cmd,
        camera_outpost_container,
        robot_state_publisher,
        delay_serial_node,
        delay_rm_serial_node,
        delay_controller_node,
    ])
