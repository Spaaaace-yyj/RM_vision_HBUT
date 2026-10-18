"""内嵌相机版实车启动：detector 自己调相机 SDK，不起 mv_camera。

其他链路和 armor_launch.py 保持一致（tracker / overlay / gimbal_controller / serial），
只是把相机那一段换掉。两者不要同时起，否则抢同一台相机。

  ros2 launch bringup armor_embedded_launch.py
  ros2 launch bringup armor_embedded_launch.py use_serial:=False
"""

from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import Command, LaunchConfiguration
from launch.conditions import IfCondition
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory

import os
import yaml


def generate_launch_description():

    use_serial = LaunchConfiguration('use_serial')
    use_sim_time = LaunchConfiguration('use_sim_time')
    tracker_overlay_debug = LaunchConfiguration('tracker_overlay_debug')

    declare_use_serial_cmd = DeclareLaunchArgument(
        'use_serial',
        default_value='True',
        description='Whether use serial port')
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use /clock simulated time')
    declare_tracker_overlay_debug_cmd = DeclareLaunchArgument(
        'tracker_overlay_debug',
        default_value='True',
        description='Whether start tracker debug overlay node')

    params_file = os.path.join(
        get_package_share_directory('bringup'), 'config', 'params.yaml')

    with open(params_file, 'r') as f:
        detector_params = yaml.safe_load(f)['/armor_detector']['ros__parameters']
    with open(params_file, 'r') as f:
        processor_params = yaml.safe_load(f)['/armor_tracker']['ros__parameters']
    with open(params_file, 'r') as f:
        controller_params = yaml.safe_load(f)['/gimbal_controller']['ros__parameters']
    with open(params_file, 'r') as f:
        serial_params = yaml.safe_load(f)['/lc_serial_driver']['ros__parameters']

    robot_description = Command(['xacro ', os.path.join(
        get_package_share_directory('rm_description'), 'urdf', 'gimbal_b3.urdf.xacro')])

    # detector 自己拿相机，独立进程，不用组件容器
    detector_node = Node(
        package='armor_detector',
        executable='armor_detector_node',
        name='armor_detector',
        output='screen',
        emulate_tty=True,
        parameters=[detector_params, {
            'use_embedded_camera': True,
            'camera_info_url': 'package://bringup/config/camera_info.yaml',
            'use_sim_time': use_sim_time,
        }],
        arguments=['--ros-args', '--log-level', 'armor_detector:=INFO'],
    )

    tracker_node = Node(
        package='armor_tracker',
        executable='armor_tracker_node',
        output='screen',
        emulate_tty=True,
        parameters=[processor_params, {'use_sim_time': use_sim_time}],
        arguments=['--ros-args', '--log-level', 'armor_tracker:=DEBUG'],
    )

    tracker_overlay_node = Node(
        package='tracker_debug_overlay',
        executable='tracker_overlay_node',
        name='tracker_overlay_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'debug_image_topic': '/detector/result_img',
            'target_topic': '/tracker/target',
            'camera_info_topic': '/camera_info',
            'output_topic': '/debug/tracker_overlay_img',

            'camera_frame_id': '',

            'small_armor_width': 0.135,
            'large_armor_width': 0.23,
            'armor_height': 0.125,

            'draw_rect': True,
            'draw_center': True,
            'draw_text': True,

            'tf_timeout_sec': 0.03,
            'sync_queue_size': 20,
        }, {'use_sim_time': use_sim_time}],
        condition=IfCondition(tracker_overlay_debug),
        arguments=['--ros-args', '--log-level', 'tracker_overlay_node:=INFO'],
    )

    gimbal_controller_node = Node(
        package='gimbal_controller',
        executable='gimbal_controller',
        output='screen',
        emulate_tty=True,
        parameters=[controller_params, {'use_sim_time': use_sim_time}],
    )

    serial_node = Node(
        package='lc_serial',
        executable='lc_serial_node',
        namespace='',
        output='screen',
        emulate_tty=True,
        parameters=[serial_params],
        condition=IfCondition(use_serial),
        arguments=['--ros-args', '--log-level', 'lc_serial:=INFO'],
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description,
                     'publish_frequency': 1000.0,
                     'use_sim_time': use_sim_time}],
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'rate': 600}],
        condition=IfCondition(use_serial),
    )

    delay_serial_node = TimerAction(
        period=1.0,
        actions=[serial_node],
    )

    delay_tracker_node = TimerAction(
        period=1.5,
        actions=[tracker_node],
    )

    delay_controller_node = TimerAction(
        period=1.0,
        actions=[gimbal_controller_node],
    )

    return LaunchDescription([
        declare_use_serial_cmd,
        declare_use_sim_time_cmd,
        declare_tracker_overlay_debug_cmd,

        detector_node,
        robot_state_publisher,
        joint_state_publisher,

        delay_serial_node,
        delay_tracker_node,
        delay_controller_node,

        tracker_overlay_node,
    ])
