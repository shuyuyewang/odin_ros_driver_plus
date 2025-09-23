# USAGE: ros2 launch odin_ros_driver odin1_ros2.launch.py
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get package directory
    package_dir = get_package_share_directory('odin_ros_driver')

    # === Common launch args ===
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(package_dir, 'config', 'control_command.yaml'),
        description='Path to the control config YAML file'
    )
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(package_dir, 'config', 'odin_ros2.rviz'),
        description='Path to RViz2 config file'
    )

    # === 1) Host SDK main node (设备/驱动入口) ===
    host_sdk_node = Node(
        package='odin_ros_driver',
        executable='host_sdk_sample',
        name='host_sdk_sample',
        output='screen',
        # arguments=['--ros-args', '--log-level', 'debug'],
        parameters=[{
            'config_file': LaunchConfiguration('config_file')
        }]
    )

    # === 2) 点云→深度图 (pcd2depth) ===
    pcd2depth_config_path = os.path.join(package_dir, 'config', 'control_command.yaml')
    with open(pcd2depth_config_path, 'r') as f:
        pcd2depth_params = yaml.safe_load(f)
    pcd2depth_calib_path = os.path.join(package_dir, 'config', 'calib.yaml')
    pcd2depth_params['calib_file_path'] = pcd2depth_calib_path
    pcd2depth_node = Node(
        package='odin_ros_driver',
        executable='pcd2depth_ros2_node',
        name='pcd2depth_ros2_node',
        output='screen',
        parameters=[pcd2depth_params]
    )

    # === 3) RViz2 ===
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')]
    )

    # === 4) 新增：点云建图并保存 (slam_mapper_node) ===
    #
    # 说明：
    # - 订阅 /odin1/cloud_slam（你的驱动可通过 sendcloudslam 开启发布）【17】
    # - 用 TF 转到 map 帧融合 + 体素降采样，发布 /odin1/cloud_map
    # - 提供服务 /save_map 立即保存；也可设置 save_interval_sec 定时保存
    #
    maps_dir_default = os.path.join(package_dir, 'maps')  # 保存目录默认放包里（你也可以改成 ~/.ros/maps）

    slam_mapper_node = Node(
        package='odin_ros_driver',          # 你的包名不变
        executable='slam_mapper_node',      # 你按照上条消息添加的可执行名
        name='slam_mapper',
        output='screen',
        parameters=[{
            'cloud_topic': '/odin1/cloud_slam',   # 输入点云（驱动开关 sendcloudslam=1 时发布）【17】
            'map_frame': 'map',                   # 目标坐标系（需有 TF 到 map；sendodom=1 可提供相关里程计/TF）【17】
            'leaf_size': 0.05,                    # 体素大小(米)——越大越稀疏
            'min_range': 0.1,
            'max_range': 60.0,
            'save_dir': maps_dir_default,         # 保存目录
            'save_format': 'ply',                 # 'ply' 或 'pcd'
            'save_interval_sec': 0,               # >0 则定时保存（秒）；0 表示不定时
            'map_topic': '/odin1/cloud_map',      # 发布的全局地图
        }]
    )

    # Create launch description
    ld = LaunchDescription()
    ld.add_action(config_file_arg)
    ld.add_action(rviz_config_arg)
    ld.add_action(host_sdk_node)
    ld.add_action(pcd2depth_node)
    ld.add_action(rviz_node)
    ld.add_action(slam_mapper_node)  # ★ 新增建图节点
    return ld
