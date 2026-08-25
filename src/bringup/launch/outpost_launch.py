from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription


def generate_launch_description():
    # ---- 可调 launch 参数 ----
    video_path = LaunchConfiguration('video_path')
    outpost_x = LaunchConfiguration('outpost_x')
    outpost_y = LaunchConfiguration('outpost_y')

    detector_format = LaunchConfiguration('detector_format')
    declare_detector_format_cmd = DeclareLaunchArgument(
        'detector_format', default_value='zju',
        description='检测模型格式：zju=浙大原版 szu=深大5点模型(快一倍)')
    declare_video_path_cmd = DeclareLaunchArgument(
        'video_path', default_value='outpost.mp4',
        description='video_pub 回放的视频文件名（放 video_pub/video/ 下）')
    declare_outpost_x_cmd = DeclareLaunchArgument(
        'outpost_x', default_value='0.0',
        description='前哨站在 odom 系的位置，位置门需要它，设 0 关闭位置门')
    declare_outpost_y_cmd = DeclareLaunchArgument(
        'outpost_y', default_value='0.0',
        description='前哨站在 odom 系的位置，位置门需要它，设 0 关闭位置门')

    detector_node = ComposableNode(
        package='armor_detector',
        plugin='rm_auto_aim::ArmorDetectorNode',
        name='armor_detector',
        parameters=[{
            'detect_color': 0,
            'classifier_threshold': 0.6,
            'ignore_classes': ['negative'],
        }],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    outpost_node = ComposableNode(
        package='outpost_detector',
        plugin='outpost_auto_aim::OutpostTargetNode',
        name='outpost_target_node',
        parameters=[{
            # 离线调试：输入就是目标系，跳过 tf；实车改 odom 并配好 tf 树
            'target_frame': 'camera_optical_frame',
            # 三块板离地高度，实车按场地实测改
            'slot_z': [1.10, 1.27, 1.45],
            'z_tol': 0.08,
            # 旋转半径，模块机械尺寸，实车按实测改
            'r_initial': 0.28,
            # 旋转方向：0=第一窗口自动测，1/-1=手动指定（每局固定随机）
            'rotate_direction': 0,
            'temp_lost_time': 1.5,
            'lost_time': 5.0,
            'tracking_thres': 5,
            'outpost_x': outpost_x,
            'outpost_y': outpost_y,
            'debug': True,
        }],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    # ---- 一个容器跑 video_pub + 检测 + 前哨站选择，进程内零拷贝传图 ----
    outpost_container = ComposableNodeContainer(
        name='outpost_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='video_pub',
                plugin='video_pub::VideoPub',
                name='video_node',
                parameters=[{
                    'video_path': video_path,
                    'fps': 30,
                    'camera_info_url': 'package://video_pub/config/camera_info_buff.yaml',
                    'use_sensor_data_qos': True,
                }],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            detector_node,
            outpost_node,
        ],
        output='screen',
    )

    return LaunchDescription([
        declare_detector_format_cmd,
        declare_video_path_cmd,
        declare_outpost_x_cmd,
        declare_outpost_y_cmd,
        outpost_container,
    ])
