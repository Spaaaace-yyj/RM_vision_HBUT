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
    # 实车能量机关链路：相机 + buff 检测 + 云台控制 + 串口，一条命令跑通
    buff_mode = LaunchConfiguration('buff_mode')
    buff_color = LaunchConfiguration('buff_color')

    declare_buff_mode_cmd = DeclareLaunchArgument(
        'buff_mode', default_value='2', description='1=小符 2=大符')
    declare_buff_color_cmd = DeclareLaunchArgument(
        'buff_color', default_value='1', description='蓝队打红符')
    detector_format = LaunchConfiguration('detector_format')
    declare_detector_format_cmd = DeclareLaunchArgument(
        'detector_format', default_value='zju',
        description='检测模型格式：zju=浙大原版 szu=深大5点模型(快一倍)')
    serial_protocol = LaunchConfiguration('serial_protocol')
    declare_serial_protocol_cmd = DeclareLaunchArgument(
        'serial_protocol', default_value='cjson',
        description='cjson=普通兵种(lc_serial) binary=哨兵(lc_rm_serial)')

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
                    'detector_format': detector_format,
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
        declare_buff_mode_cmd,
        declare_buff_color_cmd,
        declare_detector_format_cmd,
        declare_serial_protocol_cmd,
        camera_buff_container,
        robot_state_publisher,
        delay_serial_node,
        delay_rm_serial_node,
        delay_controller_node,
    ])
