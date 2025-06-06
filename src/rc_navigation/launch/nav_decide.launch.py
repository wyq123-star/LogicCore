#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取功能包路径
    my_algorithm_pkg = get_package_share_directory('my_algorithm')
    change_laser_pkg = get_package_share_directory('change_laser')
    
    # 动态参数声明
    params = [
        # 共享参数
        DeclareLaunchArgument('center_x', default_value='3.5', description='圆心X坐标'),
        DeclareLaunchArgument('center_y', default_value='-14.0', description='圆心Y坐标'),
        
        # RandomPointGenerator 参数
        DeclareLaunchArgument('map_x', default_value='7.0', description='地图X尺寸'),
        DeclareLaunchArgument('map_y', default_value='-14.0', description='地图Y尺寸'),
        DeclareLaunchArgument('origin_x', default_value='0.0', description='原点X坐标'),
        DeclareLaunchArgument('origin_y', default_value='0.0', description='原点Y坐标'),
        DeclareLaunchArgument('radius_min', default_value='3.0', description='最小半径'),
        DeclareLaunchArgument('radius_max', default_value='4.0', description='最大半径'),
        DeclareLaunchArgument('num_points', default_value='36', description='候选点数量'),
        DeclareLaunchArgument('safety_distance', default_value='0.5', description='安全距离'),
        DeclareLaunchArgument('publish_mode', default_value='fixed', description='发布模式(fixed/dynamic)'),
        DeclareLaunchArgument('continuous_publish', default_value='true', description='持续发布'),
        DeclareLaunchArgument('publish_frequency', default_value='2.0', description='发布频率(Hz)'),
        DeclareLaunchArgument('half_circle', default_value='true', description='半圆环模式'),
        DeclareLaunchArgument('min_generation_interval', default_value='0.5', description='最小生成间隔(s)'),
        
        # OptimalPointSelector 参数
        DeclareLaunchArgument('refer_point_x', default_value='3.5', description='参考点X坐标'),
        DeclareLaunchArgument('refer_point_y', default_value='0.0', description='参考点Y坐标'),
        DeclareLaunchArgument('comd_r', default_value='2.5', description='指令半径'),
        DeclareLaunchArgument('a', default_value='1.0', description='障碍物评分权重'),
        DeclareLaunchArgument('b', default_value='1.0', description='角度评分权重'),
        DeclareLaunchArgument('c', default_value='1.0', description='半径匹配评分权重'),
        DeclareLaunchArgument('max_diff', default_value='3.0', description='最大半径差值'),
        
        # ObstacleExtractor 参数
        DeclareLaunchArgument('costmap_topic', default_value='/local_costmap/costmap', description='代价地图话题'),
        
        # OptimalGoalNavigator 参数
        DeclareLaunchArgument('goal_timeout', default_value='60.0', description='导航超时时间(秒)'),
        DeclareLaunchArgument('max_failures', default_value='20', description='最大连续失败次数')
    ]

    # 创建节点
    random_point_node = Node(
        package='my_algorithm',
        executable='random_generate.py',
        name='random_generate',
        parameters=[
            {'map_x': LaunchConfiguration('map_x')},
            {'map_y': LaunchConfiguration('map_y')},
            {'origin_x': LaunchConfiguration('origin_x')},
            {'origin_y': LaunchConfiguration('origin_y')},
            {'center_x': LaunchConfiguration('center_x')},
            {'center_y': LaunchConfiguration('center_y')},
            {'radius_min': LaunchConfiguration('radius_min')},
            {'radius_max': LaunchConfiguration('radius_max')},
            {'num_points': LaunchConfiguration('num_points')},
            {'safety_distance': LaunchConfiguration('safety_distance')},
            {'publish_mode': LaunchConfiguration('publish_mode')},
            {'continuous_publish': LaunchConfiguration('continuous_publish')},
            {'publish_frequency': LaunchConfiguration('publish_frequency')},
            {'half_circle': LaunchConfiguration('half_circle')},
            {'min_generation_interval': LaunchConfiguration('min_generation_interval')}
        ],
        output='screen'
    )

    obstacle_extractor_node = Node(
        package='change_laser',
        executable='obstacle_extractor_node',
        name='obstacle_extractor',
        parameters=[
            {'costmap_topic': LaunchConfiguration('costmap_topic')}
        ],
        output='screen'
    )

    optimal_point_node = Node(
        package='my_algorithm',
        executable='logic.py',
        name='optimal_point_selector',
        parameters=[
            {'center_x': LaunchConfiguration('center_x')},
            {'center_y': LaunchConfiguration('center_y')},
            {'refer_point_x': LaunchConfiguration('refer_point_x')},
            {'refer_point_y': LaunchConfiguration('refer_point_y')},
            {'comd_r': LaunchConfiguration('comd_r')},
            {'a': LaunchConfiguration('a')},
            {'b': LaunchConfiguration('b')},
            {'c': LaunchConfiguration('c')},
            {'max_diff': LaunchConfiguration('max_diff')}
        ],
        output='screen'
    )

    optimal_goal_navigator_node = Node(
        package='my_algorithm',
        executable='nav_behave.py',
        name='optimal_goal_navigator',
        parameters=[
            {'goal_timeout': LaunchConfiguration('goal_timeout')},
            {'max_failures': LaunchConfiguration('max_failures')}
        ],
        output='screen'
    )

    # 创建启动描述
    return LaunchDescription([
        *params,
        LogInfo(msg="启动导航决策系统..."),
        random_point_node,
        obstacle_extractor_node,
        optimal_point_node,
        optimal_goal_navigator_node,
        LogInfo(msg="所有节点已启动，系统运行中")
    ])