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
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import math
import numpy as np
from scipy.spatial import KDTree
from nav_msgs.msg import Path
import yaml
import os
# 纯追踪控制器
class PurePursuitController:
    def __init__(self, lookahead_distance):
        self.lookahead_distance = lookahead_distance

    def calculate_steering_angle(self, vehicle_pose, path_points):
        # 找到离车辆最近的路径点
        closest_point_idx = KDTree(path_points[:, :2]).query(vehicle_pose[:2])[1]
        
        # 动态选择最合适的路径点作为目标点
        for i in range(len(path_points)):
            lookahead_point_idx = (closest_point_idx + i) % len(path_points)
            target_point = path_points[lookahead_point_idx]
            dx, dy = target_point[0] - vehicle_pose[0], target_point[1] - vehicle_pose[1]
            distance_to_target = math.sqrt(dx**2 + dy**2)
            if distance_to_target >= self.lookahead_distance:
                break
        
        # 计算车辆到目标点的向量
        dx, dy = target_point[0] - vehicle_pose[0], target_point[1] - vehicle_pose[1]
        # 计算目标角度
        target_angle = math.atan2(dy, dx)
        # 计算转向角
        steering_angle = target_angle - vehicle_pose[2]
        # 确保转向角在-pi到pi之间
        while steering_angle > math.pi:
            steering_angle -= 2 * math.pi
        while steering_angle < -math.pi:
            steering_angle += 2 * math.pi
        return steering_angle, target_point

# ROS 2节点
class PathFollowingNode(Node):
    def __init__(self):
        super().__init__('path_following_node')
        # 创建纯追踪控制器
        self.pure_pursuit = PurePursuitController(lookahead_distance=0.5)
        # 创建路径点
        self.path_points = None
        # 创建订阅者
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odometry_callback, 10)
        # 创建发布者
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        # 创建路径点订阅者
        self.path_subscriber = self.create_subscription(Path, '/path', self.path_callback, 10)
        # 变量初始化
        self.current_odom = None
        
        self.stop_flag = False  # 新增的停止标志
        self.path_received = False  # 添加路径是否已接收的标志
        # self.get_logger().info('ready--------ok----to---nav')

    def path_callback(self, msg):
        self.path_points_list = [[point.pose.position.x, point.pose.position.y] for point in msg.poses]
        # 将列表转换为 numpy 数组
        self.path_points = np.array(self.path_points_list)
        assert self.path_points.ndim == 2, "path_points must be a 2D array"
        # 对路径点进行插值
        self.path_points = self.interpolate_path(self.path_points)
        self.path_received = True  # 设置路径接收标志
        # print('received path ready to nav-------------')

    def interpolate_path(self, points, segment_length=0.1):
        interpolated_points = []
        for i in range(len(points) - 1):
            start_point = points[i]
            end_point = points[i+1]
            # 计算两点之间的距离
            distance = np.linalg.norm(end_point - start_point)
            # 计算所需点的数量（包括起点）
            num_points = int(distance / segment_length) + 1
            # 生成线性插值点
            t_values = np.linspace(0, 1, num_points)
            interpolated_segment = start_point + (end_point - start_point)[np.newaxis, :] * t_values[:, np.newaxis]
            interpolated_points.append(interpolated_segment)
        # 将所有插值点合并成一个数组
        return np.vstack(interpolated_points)

    def quaternion_to_yaw(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    def odometry_callback(self, msg):
        self.current_xy = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        if not self.path_received:
            return  # 如果还没有接收到路径，则直接返回
        self.current_odom = msg
        # 提取位置和朝向
        pose = [msg.pose.pose.position.x, msg.pose.pose.position.y, self.quaternion_to_yaw(msg.pose.pose.orientation)]
        
        # 纯追踪控制器计算转向角和目标点
        steering_angle, target_point = self.pure_pursuit.calculate_steering_angle(pose, self.path_points)
        
        # 计算到路径终点的距离
        distance_to_end = np.linalg.norm(np.array(pose[:2]) - self.path_points[-1])
        
        # 停止条件
        if distance_to_end < 0.2:  # 0.2m作为接近阈值
            speed = 0.0
            steering_angle = 0.0
            self.path_received = False
            # self.get_logger().info('Naving node.success..')
        else:
            # 根据转向角大小调整速度
            # 使用 sin 函数来调整速度，当转向角接近 0 时，速度接近最大值；当转向角增大时，速度逐渐减小
            # 使用多项式函数调整速度
            def sigmoid(x):
                return 1 / (1 + math.exp(-x))

            # 将转向角缩放到 [-1, 1] 范围
            scaled_steering_angle = steering_angle / (math.pi / 2)

            speed = max(0.6, 1.5 - 1.5*math.sin(0.6*abs(steering_angle)))

    

        # 发布速度和转向角
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = speed
        cmd_vel_msg.angular.z = steering_angle
        self.cmd_vel_publisher.publish(cmd_vel_msg)

    

        # self.get_logger().info(f'v: {speed:.2f}, ang: {steering_angle:.2f}, dist_to_end: {distance_to_end:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = PathFollowingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()