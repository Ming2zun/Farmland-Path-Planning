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
from sensor_msgs.msg import NavSatFix

import threading
import time
import random
class GPSPublisher(Node):

    def __init__(self):
        super().__init__('gps_publisher')
        
        self.publisher = self.create_publisher(NavSatFix, 'gps/fix', 10)
        self.timer = self.create_timer(0.5, self.publish_gps_data)  # 每 1 秒钟调用一次 publish_gps_data
            
    def publish_gps_data(self):
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gps_link'
        a = random.uniform(0.0000022, 0.0000062)
        # self.latitude = 32.215543502
        # self.longitude = 119.50056749 
# "latitude": 31.553574 ,
#  "longitude": 104.557124,
#  "altitude": 492.105,
        self.latitude = 31.552624
        self.longitude = 104.560187
        altitude = 0.0
# 112.45°E, 37.95°N
        # 添加其他字段如时间戳等
        msg.latitude=self.latitude
        msg.longitude=self.longitude
        msg.altitude = 0.0
        msg.status.status=2
        msg.status.service=1
        msg.position_covariance = [0.0001, 0.0, 0.0,
                                   0.0, 0.0001, 0.0,
                                   0.0, 0.0, 0.0001]
        msg.position_covariance_type = 2
        self.publisher.publish(msg)
        self.get_logger().info("Published GPS data")

def main(args=None):
    rclpy.init(args=args)
    gps_publisher = GPSPublisher()
    rclpy.spin(gps_publisher)
    gps_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()