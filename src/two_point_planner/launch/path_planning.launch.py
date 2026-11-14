# -*- coding: utf-8 -*-
"""
path_planning.launch.py — 一键启动整个演示：

    ros2 launch two_point_planner path_planning.launch.py

包含 4 个进程：
  1. nav2_map_server      —— 把 mymap.pgm/yaml 发布为 /map（占据栅格/costmap）
  2. nav2_lifecycle_manager —— 自动激活 map_server（它是生命周期节点，不激活不发布）
  3. astar_planner        —— 本包的 A* 两点最短路径节点
  4. rviz2                —— 可视化（加载包内预置的 rviz 配置）

可用参数：
  map_yaml       地图 yaml 路径（默认使用包内 maps/mymap.yaml）
  use_rviz       是否启动 RViz（默认 true；无桌面环境时设 false）
  planner        规划器：jps（默认，跳点搜索，快）或 astar（带衰减层，走中线）
  robot_radius   机器人等效半径（米），距障碍小于此值的区域致死，默认 0.38
  inflation      衰减层外边界（米），仅 astar 模式生效，默认 1.20
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('two_point_planner')
    default_map = os.path.join(pkg_share, 'maps', 'mymap.yaml')
    default_rviz = os.path.join(pkg_share, 'rviz', 'astar.rviz')

    map_yaml = LaunchConfiguration('map_yaml')
    use_rviz = LaunchConfiguration('use_rviz')
    planner = LaunchConfiguration('planner')
    robot_radius = LaunchConfiguration('robot_radius')
    inflation = LaunchConfiguration('inflation')

    return LaunchDescription([
        DeclareLaunchArgument('map_yaml', default_value=default_map,
                              description='地图 yaml 文件路径'),
        DeclareLaunchArgument('use_rviz', default_value='true',
                              description='是否启动 RViz'),
        DeclareLaunchArgument('planner', default_value='jps',
                              description='规划器：jps 或 astar'),
        DeclareLaunchArgument('robot_radius', default_value='0.38',
                              description='机器人等效半径（米）'),
        DeclareLaunchArgument('inflation', default_value='1.20',
                              description='代价衰减层外边界（米）'),

        # 1. 地图服务器：读取 yaml → 发布 /map（transient_local latched）
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_yaml}],
        ),

        # 2. 生命周期管理器：configure + activate map_server
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[{'autostart': True,
                         'node_names': ['map_server']}],
        ),

        # 3. 两点最短路径规划节点（JPS / A* 双模式）
        Node(
            package='two_point_planner',
            executable='astar_planner',
            name='astar_planner',
            output='screen',
            parameters=[{'planner': planner,
                         'robot_radius': robot_radius,
                         'inflation_radius': inflation}],
        ),

        # 4. RViz 可视化
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', default_rviz],
            condition=IfCondition(use_rviz),
        ),
    ])
