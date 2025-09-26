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


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 获取各个包的路径
    package1_share = FindPackageShare(package='gazebo_modele')
    package2_share = FindPackageShare(package='nav_slam')
    
    # 定义要包含的launch文件路径
    launch1_path = PathJoinSubstitution(
        [package1_share, 'launch', 'mapviz.launch.py']
    )
    
    launch2_path = PathJoinSubstitution(
        [package2_share, 'launch', '2dpoints.launch.py']
    )

    return LaunchDescription([
        # 包含第一个launch文件
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch1_path),
            # 可以在这里传递参数给被包含的launch文件
            # launch_arguments={'param_name': 'value'}.items()
        ),
        
        # 包含第二个launch文件
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch2_path)
        ),
        
        # 可以继续添加更多的launch文件...
    ])
