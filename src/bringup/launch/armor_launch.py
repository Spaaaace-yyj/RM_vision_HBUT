from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer, Node 
from launch_ros.substitutions import FindPackageShare

from launch.actions import DeclareLaunchArgument, GroupAction, TimerAction
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory

import os
import yaml


def generate_launch_description():

    use_serial = LaunchConfiguration('use_serial')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_mujoco = LaunchConfiguration('use_mujoco')
    debug_mode = LaunchConfiguration('debug_mode')
    ros_bag_mode = LaunchConfiguration('ros_bag_mode')
    tracker_overlay_debug = LaunchConfiguration('tracker_overlay_debug')
    
    declare_use_serial_cmd = DeclareLaunchArgument(
        'use_serial',
        default_value='True',
        description='Whether use serial port')
    declare_debug_mode_cmd = DeclareLaunchArgument(
        'debug_mode',
        default_value='False',
        description='Use aruco detector'
    )
    declare_ros_bag_cmd = DeclareLaunchArgument(
        'ros_bag_mode',
        default_value='False',
    )
    declare_tracker_overlay_debug_cmd = DeclareLaunchArgument(
        'tracker_overlay_debug',
        default_value='True',
        description='Whether start tracker debug overlay node'
    )
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='False',
        description='Use /clock simulated time')
    declare_use_mujoco_cmd = DeclareLaunchArgument(
        'use_mujoco', default_value='False',
        description='Use external MuJoCo images, TF and gimbal topics; skip hardware nodes')
    declare_mujoco_compressed_cmd = DeclareLaunchArgument(
        'mujoco_subscribe_compressed', default_value='False',
        description='MuJoCo image transport: False for /image_raw, True for compressed')

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

    declare_detector_mode_cmd = DeclareLaunchArgument(
        'detector_mode', default_value=detector_params.get('detector_mode', 'traditional'),
        description='traditional / neural')
    declare_neural_backend_cmd = DeclareLaunchArgument(
        'neural_backend', default_value=detector_params.get('neural_backend', 'onnxruntime'),
        description='onnxruntime / openvino (neural mode)')

    # 仿真不需要 rm_description/xacro，延迟到真车节点启动时再查找。
    robot_description = Command(['xacro ', PathJoinSubstitution([
        FindPackageShare('rm_description'), 'urdf', 'gimbal_b3.urdf.xacro'])])
        
    detector_node = ComposableNode(
        package='armor_detector',
        plugin='rm_auto_aim::ArmorDetectorNode',
        name='armor_detector',
        parameters=[detector_params, {
            'use_sim_time': use_sim_time,
            'detector_mode': LaunchConfiguration('detector_mode'),
            'neural_backend': LaunchConfiguration('neural_backend'),
        }],
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
                condition=UnlessCondition(ros_bag_mode),
                parameters=[camera_params, {'use_sensor_data_qos': False}],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            detector_node
        ],
        condition=UnlessCondition(debug_mode),
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

    # main 的 MuJoCo 模式：独立 detector 订阅仿真图像，不打开 SDK 相机。
    mujoco_detector_node = Node(
        package='armor_detector',
        executable='armor_detector_node',
        name='armor_detector',
        output='screen',
        emulate_tty=True,
        parameters=[detector_params, {
            'use_sim_time': use_sim_time,
            'use_embedded_camera': False,
            'subscribe_compressed': LaunchConfiguration('mujoco_subscribe_compressed'),
            'detector_mode': LaunchConfiguration('detector_mode'),
            'neural_backend': LaunchConfiguration('neural_backend'),
            # 渲染抗锯齿使仿真灯条略短，沿用 main 的配对阈值。
            'armor.min_light_ratio': 0.6,
        }],
        condition=IfCondition(use_mujoco),
        arguments=['--ros-args', '--log-level', 'armor_detector:=INFO'],
    )

    # 对应 main 的 sim_autoaim.yaml：odom 位于 pitch 轴高度。
    mujoco_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='mujoco_base_to_odom',
        output='screen',
        arguments=['--x', '0', '--y', '0', '--z', '0.26',
                   '--frame-id', 'base_link', '--child-frame-id', 'odom'],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_mujoco),
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

            # 让节点自动用 camera_info.header.frame_id
            # 如果投影方向不对，再改成你的 optical frame，比如 camera_optical_frame
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
        name='gimbal_controller',
        output='screen',
        emulate_tty=True,
        parameters=[controller_params, {
            'use_sim_time': use_sim_time,
            'use_sim_interface': use_mujoco,
        }],
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
    # 如果要这个通讯把下面的延迟启动替换
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
                     'publish_frequency': 1000.0,
                     'use_sim_time': use_sim_time}]
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'rate': 600, 'use_sim_time': use_sim_time}],
        condition=UnlessCondition(use_serial)
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

    delay_mujoco_detector_node = TimerAction(
        period=1.0,
        actions=[mujoco_detector_node],
    )

    return LaunchDescription([
        declare_use_serial_cmd,
        declare_debug_mode_cmd,
        declare_ros_bag_cmd,
        declare_tracker_overlay_debug_cmd,
        declare_use_sim_time_cmd,
        declare_use_mujoco_cmd,
        declare_mujoco_compressed_cmd,
        declare_detector_mode_cmd,
        declare_neural_backend_cmd,
        # 真车节点仅在非仿真模式启动。
        GroupAction(
            condition=UnlessCondition(use_mujoco),
            actions=[mv_camera_detector_container, robot_state_publisher,
                     joint_state_publisher, delay_serial_node],
        ),
        # debug_dji_camera,
        # debug_aruco_detector,

        # video_detector_container,
        delay_mujoco_detector_node,
        mujoco_odom_tf,
        delay_tracker_node,
        delay_controller_node,

        # debug
        tracker_overlay_node,
    ])
