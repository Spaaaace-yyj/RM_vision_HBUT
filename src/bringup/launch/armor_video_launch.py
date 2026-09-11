"""装甲板离线视频联调：视频当相机 + 检测节点 + 看图 + 模式切换

原来要开 5 个终端，现在一条命令：

  # 蓝方视频，传统识别模式（默认）
  ros2 launch bringup armor_video_launch.py

  # 直接进神经网络模式
  ros2 launch bringup armor_video_launch.py detector_mode:=neural

  # 红方视频（红蓝靠 detect_color 区分：0=蓝 1=红）
  ros2 launch bringup armor_video_launch.py video_path:=red_rotate_fast.mp4 detect_color:=1

  # 检测跟不上就降帧率；不想自动开看图窗口就 rqt:=false
  ros2 launch bringup armor_video_launch.py fps:=30 rqt:=false

起来之后随时在线切模式（不用重启）：
  ros2 param set /armor_detector detector_mode neural
  ros2 param set /armor_detector detector_mode traditional

视频放哪：video_pub 只认自己 share 目录里的文件名，
  cp xxx.mp4 src/video_pub/video/ && colcon build --packages-select video_pub
"""

from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer, Node
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory

import os
import yaml


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'video_path', default_value='blue_rotate_fast.mp4',
            description='video_pub 回放的视频文件名（要放在 video_pub/video/ 下）'),
        DeclareLaunchArgument(
            'detect_color', default_value='0',
            description='只识别这个颜色：0=蓝方 1=红方'),
        DeclareLaunchArgument(
            'detector_mode', default_value='traditional',
            description='traditional=传统识别（原有流程） neural=神经网络（深大模型）+传统融合'),
        DeclareLaunchArgument(
            'binary_thres', default_value='140',
            description='传统流程二值化阈值（神经网络模式下融合找灯条也用它）'),
        DeclareLaunchArgument(
            'fps', default_value='60',
            description='视频回放帧率，检测跟不上就降到 30'),
        DeclareLaunchArgument(
            'debug', default_value='true',
            description='发布调试图（/detector/result_img 等），关了就只能看话题'),
        DeclareLaunchArgument(
            'neural_conf_threshold', default_value='0.65',
            description='神经网络置信度阈值（仅 neural 模式生效）'),
        DeclareLaunchArgument(
            'use_tf', default_value='true',
            description='发 odom→camera_optical_frame 静态 TF；离线没有云台 TF，不发的话 PnP 出不了位姿'),
        DeclareLaunchArgument(
            'use_tracker', default_value='false',
            description='同时起 armor_tracker，看能不能锁上'),
        DeclareLaunchArgument(
            'rqt', default_value='true',
            description='自动打开 rqt_image_view 看 /detector/result_img'),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])


def launch_setup(context, *args, **kwargs):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    def arg_bool(name):
        return arg(name).strip().lower() in ('true', '1', 'yes', 'on')

    def arg_int(name):
        return int(arg(name))

    def arg_float(name):
        return float(arg(name))

    video_name = arg('video_path')
    detect_color = arg_int('detect_color')
    detector_mode = arg('detector_mode').strip().lower()
    if detector_mode not in ('traditional', 'neural'):
        detector_mode = 'traditional'

    params_file = os.path.join(get_package_share_directory('bringup'), 'config', 'params.yaml')
    with open(params_file, 'r') as f:
        all_params = yaml.safe_load(f)

    actions = []

    # 视频不在 video_pub 的 share 目录里时，直接给出可用列表，省得只看到 "frame is empty"
    video_dir = os.path.join(get_package_share_directory('video_pub'), 'video')
    video_full = os.path.join(video_dir, video_name)
    if os.path.isfile(video_full):
        actions.append(LogInfo(msg=f'[armor_video] 回放视频: {video_full}'))
    else:
        available = []
        if os.path.isdir(video_dir):
            available = sorted(
                name for name in os.listdir(video_dir) if name.endswith(('.mp4', '.avi')))
        actions.append(LogInfo(msg=(
            f'[armor_video][警告] 没找到视频 {video_full}；'
            f'把视频拷到 src/video_pub/video/ 后跑 colcon build --packages-select video_pub，'
            f'或用 video_path:= 指定。当前可用: {available}')))

    # 检测参数：先用 params.yaml 里那套，再按 launch 参数覆盖
    detector_params = dict(all_params['/armor_detector']['ros__parameters'])
    detector_params.update({
        'subscribe_compressed': False,
        'debug': arg_bool('debug'),
        'detect_color': detect_color,
        'binary_thres': arg_int('binary_thres'),
        'detector_mode': detector_mode,
        'neural_conf_threshold': arg_float('neural_conf_threshold'),
    })

    # 视频和检测放同一个容器，进程内零拷贝传图（和 armor_launch.py 里 video_detector_container 一样）
    detector_container = ComposableNodeContainer(
        name='armor_video_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='video_pub',
                plugin='video_pub::VideoPub',
                name='video_node',
                parameters=[{
                    'video_path': video_name,
                    'fps': arg_int('fps'),
                    # 1280x1024，和 rotate_fast 系列视频一致（实车标定文件）
                    'camera_info_url': 'package://video_pub/config/camera_info.yaml',
                    'use_sensor_data_qos': True,
                }],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='armor_detector',
                plugin='rm_auto_aim::ArmorDetectorNode',
                name='armor_detector',
                parameters=[detector_params],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen',
    )
    actions.append(detector_container)

    # 离线没有云台/雷达，补一个静态 TF，PnP 才有位姿可发
    actions.append(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='armor_video_static_tf',
        arguments=['--frame-id', 'odom', '--child-frame-id', 'camera_optical_frame'],
        condition=IfCondition(arg('use_tf')),
        output='log',
    ))

    # 想看锁定效果再加
    actions.append(Node(
        package='armor_tracker',
        executable='armor_tracker_node',
        name='armor_tracker',
        parameters=[dict(all_params['/armor_tracker']['ros__parameters'])],
        condition=IfCondition(arg('use_tracker')),
        output='screen',
        emulate_tty=True,
    ))

    # 看图窗口延迟 3 秒起：等 /detector/result_img 话题出来再开，否则 rqt 会开成空白窗口。
    # rqt_image_view 的 shebang 是 /usr/bin/env python3，PATH 前面如果挂了别的 python
    # （比如 ~/.platformio/penv），那个环境里没有 PyQt5 就会起不来，
    # 所以这里给看图进程单独把系统 python 放最前面，ROS 的 PYTHONPATH 不受影响。
    rqt_env = {'PATH': '/usr/bin:/bin:' + os.environ.get('PATH', '')}
    actions.append(TimerAction(period=3.0, actions=[Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='rqt_image_view',
        arguments=['/detector/result_img'],
        additional_env=rqt_env,
        condition=IfCondition(arg('rqt')),
        output='log',
    )]))

    actions.append(LogInfo(msg=(
        f'[armor_video] 视频={video_name} 颜色={"红" if detect_color == 1 else "蓝"} '
        f'模式={detector_mode}\n'
        f'  切模式: ros2 param set /armor_detector detector_mode neural\n'
        f'  切模式: ros2 param set /armor_detector detector_mode traditional\n'
        f'  换颜色: ros2 param set /armor_detector detect_color 1\n'
        f'  调阈值: ros2 param set /armor_detector neural_conf_threshold 0.8\n'
        f'  看图  : ros2 run rqt_image_view rqt_image_view /detector/result_img')))

    return actions
