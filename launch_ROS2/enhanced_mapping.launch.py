#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 声明启动参数
    input_topic_arg = DeclareLaunchArgument(
        'input_topic',
        default_value='/odin/points',
        description='Input point cloud topic'
    )
    
    save_directory_arg = DeclareLaunchArgument(
        'save_directory',
        default_value='./slam_maps',
        description='Directory to save maps and trajectories'
    )
    
    leaf_size_arg = DeclareLaunchArgument(
        'leaf_size',
        default_value='0.1',
        description='Voxel grid leaf size for downsampling'
    )
    
    max_range_arg = DeclareLaunchArgument(
        'max_range',
        default_value='50.0',
        description='Maximum range for point cloud filtering'
    )
    
    min_range_arg = DeclareLaunchArgument(
        'min_range',
        default_value='0.5',
        description='Minimum range for point cloud filtering'
    )
    
    keyframe_distance_threshold_arg = DeclareLaunchArgument(
        'keyframe_distance_threshold',
        default_value='1.0',
        description='Distance threshold for keyframe creation'
    )
    
    keyframe_angle_threshold_arg = DeclareLaunchArgument(
        'keyframe_angle_threshold',
        default_value='0.3',
        description='Angle threshold for keyframe creation (radians)'
    )
    
    max_map_size_arg = DeclareLaunchArgument(
        'max_map_size',
        default_value='1000000',
        description='Maximum number of points in accumulated map'
    )
    
    loop_closure_distance_threshold_arg = DeclareLaunchArgument(
        'loop_closure_distance_threshold',
        default_value='3.0',
        description='Distance threshold for loop closure detection'
    )
    
    loop_closure_score_threshold_arg = DeclareLaunchArgument(
        'loop_closure_score_threshold',
        default_value='0.6',
        description='ICP fitness score threshold for loop closure verification'
    )
    
    # Enhanced SLAM Mapping Node
    mapping_node = Node(
        package='odin_ros_driver',
        executable='mapping_node',
        name='enhanced_slam_mapping',
        output='screen',
        parameters=[{
            'input_topic': LaunchConfiguration('input_topic'),
            'save_directory': LaunchConfiguration('save_directory'),
            'leaf_size': LaunchConfiguration('leaf_size'),
            'icp_max_correspondence_distance': 1.0,
            'icp_transformation_epsilon': 1e-6,
            'icp_euclidean_fitness_epsilon': 1e-6,
            'icp_max_iterations': 50,
            'max_range': LaunchConfiguration('max_range'),
            'min_range': LaunchConfiguration('min_range'),
            'keyframe_distance_threshold': LaunchConfiguration('keyframe_distance_threshold'),
            'keyframe_angle_threshold': LaunchConfiguration('keyframe_angle_threshold'),
            'max_map_size': LaunchConfiguration('max_map_size'),
            'loop_closure_distance_threshold': LaunchConfiguration('loop_closure_distance_threshold'),
            'loop_closure_score_threshold': LaunchConfiguration('loop_closure_score_threshold'),
            'pose_graph_optimization_interval': 10,
            'trajectory_save_interval': 100,
        }],
        remappings=[
            ('/mapping/accumulated_map', '/slam/map'),
            ('/mapping/odometry', '/slam/odometry'),
            ('/mapping/trajectory', '/slam/trajectory'),
        ]
    )
    
    # 启动信息
    launch_info = LogInfo(
        msg=[
            'Enhanced SLAM Mapping System Started!\n',
            'Input Topic: ', LaunchConfiguration('input_topic'), '\n',
            'Save Directory: ', LaunchConfiguration('save_directory'), '\n',
            'Services Available:\n',
            '  - /mapping/save_map (std_srvs/srv/Trigger)\n',
            '  - /mapping/reset (std_srvs/srv/Trigger)\n',
            '  - /mapping/enable_loop_closure (std_srvs/srv/SetBool)\n',
            'Topics Published:\n',
            '  - /slam/map (sensor_msgs/msg/PointCloud2)\n',
            '  - /slam/odometry (nav_msgs/msg/Odometry)\n',
            '  - /slam/trajectory (nav_msgs/msg/Path)\n'
        ]
    )
    
    return LaunchDescription([
        input_topic_arg,
        save_directory_arg,
        leaf_size_arg,
        max_range_arg,
        min_range_arg,
        keyframe_distance_threshold_arg,
        keyframe_angle_threshold_arg,
        max_map_size_arg,
        loop_closure_distance_threshold_arg,
        loop_closure_score_threshold_arg,
        launch_info,
        mapping_node,
    ])