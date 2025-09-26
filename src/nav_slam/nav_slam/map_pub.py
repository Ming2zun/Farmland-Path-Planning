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

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
# 保留
# 定义一个ROS节点类，用于生成障碍物网格地图
class ObstacleGridNode(Node):
    def __init__(self):
        super().__init__('obstacle_grid_node')

        # 声明并获取参数
        self.declare_parameter('grid_width', 60.0)  # 地图宽度
        self.declare_parameter('grid_height', 60.0)  # 地图高度
        self.declare_parameter('resolution', 0.1)  # 地图分辨率
        # self.declare_parameter('min_height', -1.1)  # 点云最小高度
        self.declare_parameter('min_height', 0.1)  # 点云最小高度
        self.declare_parameter('max_height', 2.0)  # 点云最大高度
        self.declare_parameter('obstacle_radius', 0.2)  # 障碍物半径
        # 获取参数值
        self.grid_width = self.get_parameter('grid_width').get_parameter_value().double_value
        self.grid_height = self.get_parameter('grid_height').get_parameter_value().double_value
        self.resolution = self.get_parameter('resolution').get_parameter_value().double_value
        self.min_height = self.get_parameter('min_height').get_parameter_value().double_value
        self.max_height = self.get_parameter('max_height').get_parameter_value().double_value
        self.obstacle_radius = self.get_parameter('obstacle_radius').get_parameter_value().double_value
        # 初始化障碍物和膨胀层集合
        self.obstacles = set()
        self.dilated_obstacles_layer1 = set()
        self.dilated_obstacles_layer2 = set()
        self.dilated_obstacles_layer3 = set()
        # 初始化OccupancyGrid消息
        self.grid_combined = OccupancyGrid()
        self.grid_combined.header.frame_id = 'map'  # 设置参考系
        self.grid_combined.info.width = int(self.grid_width / self.resolution)  # 计算地图宽度（单元格数）
        self.grid_combined.info.height = int(self.grid_height / self.resolution)  # 计算地图高度（单元格数）
        self.grid_combined.info.resolution = self.resolution  # 设置分辨率
        self.grid_combined.data = [-1] * (self.grid_combined.info.width * self.grid_combined.info.height)  # 初始化地图数据为未知（-1）
        # 创建订阅者和发布者
        self.pointcloud_sub = self.create_subscription(PointCloud2, '/mapokk', self.pointcloud_callback, 10)  # 订阅点云话题
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)  # 订阅里程计话题
        self.grid_combined_pub = self.create_publisher(OccupancyGrid, 'combined_grid', 10)  # 发布综合网格地图
        self.odom_data = None  # 初始化里程计数据为None

    def odom_callback(self, msg):
        # 处理里程计数据回调
        self.odom_data = msg
    def pointcloud_callback(self, msg):
        # 处理点云数据回调
        if self.odom_data is None:
            return  # 如果没有里程计数据，返回
        # 获取地图原点位置
        origin_x = self.odom_data.pose.pose.position.x
        origin_y = self.odom_data.pose.pose.position.y
        # 读取点云数据
        points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        radius_cells = int(self.obstacle_radius / self.resolution)  # 计算障碍物半径对应的单元格数
        new_obstacles = set()
        new_dilated_obstacles_layer1 = set()
        new_dilated_obstacles_layer2 = set()
        new_dilated_obstacles_layer3 = set()
        for x, y, z in points:
            if self.min_height <= z <= self.max_height:
                # 计算点在地图中的坐标
                center_x = int((x + self.grid_width / 2) / self.resolution)
                center_y = int((y + self.grid_height / 2) / self.resolution)

                # 标记障碍物单元格
                if 0 <= center_x < self.grid_combined.info.width and 0 <= center_y < self.grid_combined.info.height:
                    index = center_y * self.grid_combined.info.width + center_x
                    new_obstacles.add(index)  # 添加到新障碍物集合

                # 膨胀障碍物单元格
                for layer, dilated_set in enumerate([new_dilated_obstacles_layer1, new_dilated_obstacles_layer2, new_dilated_obstacles_layer3]):
                    for dx in range(-(layer + 1) * radius_cells, (layer + 1) * radius_cells + 1):
                        for dy in range(-(layer + 1) * radius_cells, (layer + 1) * radius_cells + 1):
                            if dx**2 + dy**2 <= ((layer + 1) * radius_cells)**2:
                                grid_x = center_x + dx
                                grid_y = center_y + dy
                                if 0 <= grid_x < self.grid_combined.info.width and 0 <= grid_y < self.grid_combined.info.height:
                                    index = grid_y * self.grid_combined.info.width + grid_x
                                    dilated_set.add(index)  # 添加到新膨胀层集合
        # 更新障碍物和膨胀层集合
        self.obstacles.update(new_obstacles)
        self.dilated_obstacles_layer1.update(new_dilated_obstacles_layer1)
        self.dilated_obstacles_layer2.update(new_dilated_obstacles_layer2)
        self.dilated_obstacles_layer3.update(new_dilated_obstacles_layer3)

        # 更新综合网格地图
        self.update_combined_grid()

    def update_combined_grid(self):
        # 初始化综合网格数据
        self.grid_combined.data = [-1] * (self.grid_combined.info.width * self.grid_combined.info.height)  # 初始化地图数据为未知（-1）
        # 标记障碍物为黑色（100）
        for index in self.obstacles:
            if self.grid_combined.data[index] != 100:  # 避免重复标记
                self.grid_combined.data[index] = 100  # 标记为障碍物
        # 标记三层膨胀层为不同颜色
        for index in self.dilated_obstacles_layer1 - self.obstacles:
            if self.grid_combined.data[index] == -1:
                self.grid_combined.data[index] = 5  # 第一层膨胀层
        for index in self.dilated_obstacles_layer2 - self.dilated_obstacles_layer1:
            if self.grid_combined.data[index] == -1:
                self.grid_combined.data[index] = -8  # 第二层膨胀层
        for index in self.dilated_obstacles_layer3 - self.dilated_obstacles_layer2:
            if self.grid_combined.data[index] == -1:
                self.grid_combined.data[index] = -120 # 第三层膨胀层
        # 更新综合网格地图消息头
        self.grid_combined.header.stamp = self.get_clock().now().to_msg()
        self.grid_combined.header.frame_id = 'map'
        self.grid_combined.info.origin.position.x = -self.grid_width / 2
        self.grid_combined.info.origin.position.y = -self.grid_height / 2
        self.grid_combined.info.origin.position.z = 0.0  # 假设没有垂直偏移
        # 发布综合网格地图
        self.grid_combined_pub.publish(self.grid_combined)

def main(args=None):
    # ROS2初始化
    rclpy.init(args=args)
    node = ObstacleGridNode()
    rclpy.spin(node)  # 进入主循环
    rclpy.shutdown()  # 关闭ROS2节点

if __name__ == '__main__':
    main()