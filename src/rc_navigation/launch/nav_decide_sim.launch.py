#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_context import LaunchContext

def generate_launch_description():
    config_file_path = PathJoinSubstitution([
        FindPackageShare('rc_navigation'),
        'config',  
        'nav_decide_sim.yaml'
    ])
    
    config_path_arg = DeclareLaunchArgument(
        'config_file',
        default_value=config_file_path,
        description='全局参数配置文件路径'
    )
    
    # 创建上下文用于解析路径
    context = LaunchContext()
    resolved_config_path = config_file_path.perform(context)

    nodes = [
        # RandomPointGenerator节点
        Node(
            package='my_algorithm',
            executable='random_generate.py',
            name='random_generate',
            parameters=[LaunchConfiguration('config_file')]
        ),
        # ObstacleExtractor节点
        Node(
            package='change_laser',
            executable='obstacle_extractor_node',
            name='obstacle_extractor',
            parameters=[LaunchConfiguration('config_file')]
        ),
        # OptimalPointSelector节点
        Node(
            package='my_algorithm',
            executable='logic.py',
            name='optimal_point_selector',
            parameters=[LaunchConfiguration('config_file')]
        ),
        # OptimalGoalNavigator节点
        Node(
            package='my_algorithm',
            executable='nav_behave.py',
            name='optimal_goal_navigator',
            parameters=[LaunchConfiguration('config_file')]
        )
    ]
    
    return LaunchDescription([
        config_path_arg,
        LogInfo(msg="启动导航决策..."),
        *nodes,
        LogInfo(msg=f"参数文件路径: {resolved_config_path}") 
    ])