#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2025 <Ming2zun:https://github.com/Ming2zun/Pure-tracking-slam-automatic-navigation-system>
#                <喵了个水蓝蓝:https://www.bilibili.com/video/BV1kzEwzuEFw?spm_id_from=333.788.videopod.sections&vd_source=134c12873ff478ea447a06d652426f8f>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    ld = LaunchDescription()
    
    rviz_config_file = '/home/boxing/di_pan_ws/src/nav_slam/config/rviz_config'
    package_name = 'nav_slam'
    config_dir = get_package_share_directory(package_name)
    #构建rviz配置文件的路径
    rviz_config_file = os.path.join(config_dir,'config','rviz.rviz')

    
    
    astar = Node( # 基于2d图规划路径
        package='nav_slam',
        executable='astar',
        name='astar',
        output='screen',
    )
    map_pub = Node( # 基于优化后的点构建2d地图/lio_sam/mapping/cloud_registered
        package='nav_slam',
        executable='map_pub',
        name='map_pub',
        output='screen',
    )
    odom_map_tf = Node(# 发布odom到map的坐标转换
        package='nav_slam',
        executable='odom_map_tf',
        name='odom_map_tf',
        output='screen',
    )
    points_pub_map = Node( # 发布优化后的点云
        package='nav_slam',
        executable='points_pub_map',
        name='points_pub_map',
        output='screen',
    )
    start_nav = Node(
        package='nav_slam',
        executable='start_nav',
        name='start_nav',
        output='screen',
    )
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
    )
  
   
    # ld.add_action(astar)
    # ld.add_action(map_pub)
    ld.add_action(odom_map_tf)
    ld.add_action(points_pub_map)
    ld.add_action(start_nav)

    ld.add_action(rviz2_node)


   
    

    return ld
