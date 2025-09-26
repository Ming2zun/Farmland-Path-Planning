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
import numpy as np
import heapq
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
import math
from rclpy.qos import QoSProfile
import scipy.interpolate as si
import numpy as np
from nav_msgs.msg import Odometry
from scipy.interpolate import BSpline
import time

expansion_size = 5  # 扩展障碍物大小，用于成本图中的障碍物膨胀

# 处理成本图数据，扩展障碍物
def costmap(data, width, height, resolution):
    data = np.array(data).reshape(height, width)  # 重塑数据为矩阵
    # 扩展障碍物
    # 使用 NumPy 的广播机制来替代循环
    wall_mask = data == 100
    for i in range(-expansion_size, expansion_size + 1):
        for j in range(-expansion_size, expansion_size + 1):
            if i == 0 and j == 0:
                continue
            shifted_mask = np.roll(wall_mask, (i, j), axis=(0, 1))
            data[shifted_mask] = 100
    data = data * resolution  # 将成本图中的值乘以分辨率
    return data

def bezier_smoothing(array, num_points):
    try:
        array = np.array(array)
        x = array[:, 0]
        y = array[:, 1]
        
        # 计算基于弦长的参数t
        dx = np.diff(x, prepend=x[0])
        dy = np.diff(y, prepend=y[0])
        chord_lengths = np.sqrt(dx**2 + dy**2)  # 弦长
        t = np.concatenate(([0], np.cumsum(chord_lengths)))  # 累积弦长作为参数t
        t /= t[-1]  # 规范化到[0, 1]
        
        k = num_points-1  # 贝塞尔曲线的阶数，这里选择三次贝塞尔曲线
        
        # 添加重复的节点，确保有足够的节点来定义样条
        t_knots = np.concatenate(([0]*k, t, [1]*k))
        # 根据新的节点数组调整x和y的长度
        x_padded = np.pad(x, (k, k), 'edge')
        y_padded = np.pad(y, (k, k), 'edge')
        
        # 创建B样条对象
        spline_x = BSpline(t_knots, x_padded, k, extrapolate=False)
        spline_y = BSpline(t_knots, y_padded, k, extrapolate=False)
        
        # 基于等间距的t_new重新采样
        t_new = np.linspace(0, 1, num_points)
        x_smoothed = spline_x(t_new)
        y_smoothed = spline_y(t_new)
        
        path = np.column_stack((x_smoothed, y_smoothed))
    except Exception as e:
        # print(f"Error encountered: {e}")
        path = array
    return path

# A*算法
def astar(start, goal, grid):
    def heuristic(a, b):
        return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)  # 使用欧几里得距离作为启发式函数
    rows, cols = grid.shape
    open_set = []
    heapq.heappush(open_set, (0 + heuristic(start, goal), 0, start))
    came_from = {}
    cost_so_far = {start: 0}
    closed_set = set()  # 使用集合来存储已访问的节点
    while open_set:
        _, current_cost, current = heapq.heappop(open_set)
        if current == goal:
            # 构建路径
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            return path
        if current in closed_set:  # 检查节点是否已被访问过
            continue
        closed_set.add(current)  # 标记为已访问
        for d in [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)]:
            neighbor = (current[0] + d[0], current[1] + d[1])
            if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols and grid[neighbor] != 100:
                new_cost = cost_so_far[current] + grid[neighbor]
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    priority = new_cost + heuristic(goal, neighbor)
                    heapq.heappush(open_set, (priority, new_cost, neighbor))
                    came_from[neighbor] = current
    return []  # No path found

# 导航控制节点类
class NavigationControl(Node):
    def __init__(self):
        super().__init__('Navigation')  # 初始化ROS 2节点
        # 创建订阅器订阅地图数据
        self.map_subscription = self.create_subscription(OccupancyGrid, 'combined_grid', self.map_callback, 10)
        self.path_publisher = self.create_publisher(Path, 'path', 10)
        self.path_publisher2 = self.create_publisher(Path, 'path2', 10)
        self.odom_subscriber = self.create_subscription(Odometry,'/odom',self.odom_callback,10)
        self.pose_subscriber = self.create_subscription(PoseStamped,'/goal_pose',self.goal_callback,10)
        self.x = 0.0
        self.y =0.0
        self.goal = None
        self.create_timer(0.1, self.publish_path)
        self.path = None
        self.path2 = None
        
    def goal_callback(self,msg):
        self.goal = (msg.pose.position.x,msg.pose.position.y)
    def odom_callback(self,msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
    
    # 地图数据回调函数
    def map_callback(self, msg):
        if self.goal is None:
            # print('no goal')
            return
        distance = abs(math.hypot(self.x - self.goal[0], self.y - self.goal[1]))
        # print(distance)
        if distance > 0.2:
            path = []
            resolution = msg.info.resolution  # 获取地图分辨率
            originX = msg.info.origin.position.x  # 获取地图原点x坐标
            originY = msg.info.origin.position.y  # 获取地图原点y坐标
            column = int((self.x - originX) / resolution)  # 计算机器人的x坐标对应的列索引
            row = int((self.y - originY) / resolution)  # 计算机器人的y坐标对应的行索引
            columnH = int((self.goal[0] - originX) / resolution)  # 计算目标位置的x坐标对应的列索引
            rowH = int((self.goal[1] - originY) / resolution)  # 计算目标位置的y坐标对应的行索引
            data = costmap(msg.data, msg.info.width, msg.info.height, resolution)  # 处理成本图数据
            data[row][column] = 1  # 将机器人位置标记为可通行区域
            # 将-1到5之间的值设为1，其余值设为100
            data[(data >= -2) & (data <= 5)] = 1
            data[(data < -2) | (data > 5)] = 100 #根据地图信息标记
            start = (row, column)
            goal = (rowH, columnH)
            path = astar(start, goal, data)
            paths = [(p[1] * resolution + originX, p[0] * resolution + originY) for p in path]
            self.path =paths
            self.path2 = bezier_smoothing(paths, len(paths))  # 减少平滑后的点数
            if len(self.path) > 5:
                self.path = self.path
                self.path2 = self.path2
                
            else:
                pass
            # self.publish_path(paths)#发布平滑后的路径
        else:
            # print("reach goal----nav stop")
            pass
    
    # 发布路径
    def publish_path(self):
        if self.path is None or len(self.path)==0:
            # print('no path')
            return
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        for (y, x) in self.path:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = float(y)
            pose.pose.position.y = float(x)
            path_msg.poses.append(pose)
        self.path_publisher.publish(path_msg)


        path2_msg = Path()
        path2_msg.header.frame_id = 'map'
        for (y, x) in self.path2:
            pose2 = PoseStamped()
            pose2.header.frame_id = 'map'
            pose2.pose.position.x = float(y)
            pose2.pose.position.y = float(x)
            path2_msg.poses.append(pose2)
        self.path_publisher2.publish(path2_msg)

# 主函数
def main(args=None):
    rclpy.init(args=args)  # 初始化ROS 2
    navigation_control = NavigationControl()  # 创建导航控制节点实例
    rclpy.spin(navigation_control)  # 运行节点
    navigation_control.destroy_node()  # 销毁节点
    rclpy.shutdown()  # 关闭ROS 2

# 如果直接运行此文件，则执行主函数
if __name__ == '__main__':
    main()