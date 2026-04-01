from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer, Node 

from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import Command, PythonExpression, LaunchConfiguration
from launch.conditions import IfCondition
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory

import os
import yaml


def generate_launch_description():

    use_serial = LaunchConfiguration('use_serial')
    debug_mode = LaunchConfiguration('debug_mode')
    ros_bag_mode = LaunchConfiguration('ros_bag_mode')
    
    declare_use_serial_cmd = DeclareLaunchArgument(
        'use_serial',
        default_value='True',
        description='Whether use serial port')
    declare_debug_mode_cmd = DeclareLaunchArgument(
        'debug_mode',
        default_value='False',
    )
    declare_ros_bag_cmd = DeclareLaunchArgument(
        'ros_bag_mode',
        default_value='False',
    )

    # params file path
    params_file = os.path.join(
        get_package_share_directory('bringup'), 'config', 'params.yaml')

    # load params for composable node
    with open(params_file, 'r') as f:
        camera_params = yaml.safe_load(f)['/mv_camera']['ros__parameters']
    with open(params_file, 'r') as f:
        detector_params = yaml.safe_load(f)['/armor_detector']['ros__parameters']
    with open(params_file, 'r') as f:
        processor_params = yaml.safe_load(f)['/armor_tracker']['ros__parameters']
    with open(params_file, 'r') as f:
        controller_params = yaml.safe_load(f)['/gimbal_controller']['ros__parameters']
    with open(params_file, 'r') as f:
        serial_params = yaml.safe_load(f)['/lc_serial_driver']['ros__parameters']
    with open(params_file, 'r') as f:
        video_params = yaml.safe_load(f)['/video_pub']['ros__parameters']

    # robot_description
    robot_description = Command(['xacro ', os.path.join(
        get_package_share_directory('rm_description'), 'urdf', 'gimbal_b3.urdf.xacro')])
        
    detector_node = ComposableNode(
        package='armor_detector',
        plugin='rm_auto_aim::ArmorDetectorNode',
        name='armor_detector',
        parameters=[detector_params],
        extra_arguments=[{'use_intra_process_comms': True}],
        # arguments=['--ros-args', '--log-level', 'armor_detector:=DEBUG'],
    )

    # 相机和观测节点注册
    mv_camera_detector_container = ComposableNodeContainer(
        name='camera_detector_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='mindvision_camera',
                plugin='mindvision_camera::MVCameraNode',
                name='camera_node',
                condition=IfCondition(PythonExpression(["not ", ros_bag_mode])),
                parameters=[camera_params, {'use_sensor_data_qos': False}],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            detector_node
        ],
        condition=IfCondition(PythonExpression(["not ", debug_mode])),
        output='screen',
    )

    video_detector_container = ComposableNodeContainer(
        name='video_detector_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='video_pub',
                plugin='video_pub::VideoPub',
                name='camera_node',
                parameters=[video_params],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            detector_node
        ],
        output='screen',
    )
    
    tracker_node = Node(
        package='armor_tracker',
        executable='armor_tracker_node',
        output='screen',
        emulate_tty=True,
        parameters=[processor_params],
        arguments=['--ros-args', '--log-level', 'armor_tracker:=DEBUG'],
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
        condition=IfCondition(use_serial),
        arguments=['--ros-args', '--log-level', 'lc_serial:=INFO'],
    )

    serial_test_node = Node(
        package='lc_serial_test',
        executable='lc_serial_test',
        namespace='',
        output='screen',
        emulate_tty=True,
        # parameters=[serial_params],
        condition=IfCondition(use_serial),
        arguments=['--ros-args', '--log-level', 'serial_test:=INFO'],
    )

    debug_aruco_detector = Node(
        package='aruco_detector',
        executable='aruco_detector',
        namespace='',
        output='screen',
        condition=IfCondition(debug_mode),
        arguments=['--ros-args', '--log-level', 'aruco_detector:=INFO'],
    )

    debug_dji_camera = Node(
        package='dji_action4_camera',
        executable='camera_node',
        namespace='',
        output='screen',
        condition=IfCondition(debug_mode),
        arguments=['--ros-args', '--log-level', 'camera_node:=INFO'],
    )
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description,
                     'publish_frequency': 1000.0}]
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'rate': 600}],
        condition=IfCondition(PythonExpression(["not ", use_serial]))
    )

    delay_serial_node = TimerAction(
        period=1.0,
        actions=[serial_test_node],
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
        declare_debug_mode_cmd,
        declare_ros_bag_cmd,
        # 启动相机和观测节点
        mv_camera_detector_container,
        # debug_dji_camera,
        # debug_aruco_detector,

        # video_detector_container,
        robot_state_publisher,
        joint_state_publisher,

        # serial_node,
        delay_serial_node,
        delay_tracker_node,
        delay_controller_node,
    ])