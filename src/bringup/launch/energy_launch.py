from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    # ---- 可调 launch 参数，覆盖示例: ros2 launch bringup energy_launch.py buff_mode:=1 ----
    buff_mode = LaunchConfiguration('buff_mode')
    buff_color = LaunchConfiguration('buff_color')
    video_path = LaunchConfiguration('video_path')

    detector_format = LaunchConfiguration('detector_format')
    declare_detector_format_cmd = DeclareLaunchArgument(
        'detector_format', default_value='zju',
        description='检测模型格式：zju=浙大原版 szu=深大5点模型(快一倍)')
    declare_buff_mode_cmd = DeclareLaunchArgument(
        'buff_mode', default_value='2',
        description='1=小符(固定转速KF) 2=大符(RANSAC正弦拟合)')
    declare_buff_color_cmd = DeclareLaunchArgument(
        'buff_color', default_value='0',
        description='0=打蓝符(离线视频是蓝符) 1=打红符(实车蓝队用)')
    declare_video_path_cmd = DeclareLaunchArgument(
        'video_path', default_value='buff.mp4',
        description='video_pub 回放的视频文件名（放 video_pub/video/ 下）')

    # 模型装在 buff_detector 包的 share 目录
    model_path = os.path.join(
        get_package_share_directory('buff_detector'), 'model', 'buff.onnx')

    # ---- 能量机关全链路节点：检测到发布 Target 一条龙 ----
    buff_detector_node = ComposableNode(
        package='buff_detector',
        plugin='buff_auto_aim::BuffDetectorNode',
        name='buff_detector_node',
        parameters=[{
            'detector_format': detector_format,
                    'model_path': model_path,
            'buff_mode': buff_mode,
            'buff_color': buff_color,
            # 离线调试：相机系就是目标系，跳过 tf 变换
            # 实车：改成 odom，需要 tf 树 odom←gimbal←camera
            'target_frame': 'camera_optical_frame',
            'debug_image': True,
        }],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    # ---- 一个容器跑 video_pub 和 buff_detector，进程内零拷贝传图 ----
    video_buff_container = ComposableNodeContainer(
        name='buff_container',
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
                    # 1280x768 假内参匹配 buff.mp4，实车换真实标定文件
                    'camera_info_url': 'package://video_pub/config/camera_info_buff.yaml',
                    'use_sensor_data_qos': True,
                }],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            buff_detector_node,
        ],
        output='screen',
    )

    return LaunchDescription([
        declare_detector_format_cmd,
        declare_buff_mode_cmd,
        declare_buff_color_cmd,
        declare_video_path_cmd,
        video_buff_container,
    ])
