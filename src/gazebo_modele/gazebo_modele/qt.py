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
import sys
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PolygonStamped
from std_msgs.msg import String
import json
from PySide6.QtWidgets import QApplication, QWidget
from gazebo_modele.Ui_nav2 import Ui_navigation
from gazebo_modele.farmland_path_planning import Coordinateself
import yaml
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class GUIRunner(threading.Thread):
    def __init__(self,node):
        super().__init__()
        self.daemon = True
        self.node = node
        
    def run(self):
        app = QApplication(sys.argv)
        window = MyWindow()
        window.init_ros_publisher(self.node)
        window.show()
        sys.exit(app.exec())

class MyWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.ui = Ui_navigation()
        self.ui.setupUi(self)
        self.publisher_ = None  # 创建一个发布者实例
        self.msg = String()
        self.ui.pb_leading.clicked.connect(self.on_button_clicked)
        self.ui.pb_na.clicked.connect(self.on_path_pub)
        
    def on_path_pub(self):
        if self.publisher_ is not None:
           
            self.msg.data = json.dumps({'wide': self.wide,
                                   'path_order':1,
                                   })
            self.publisher_.publish(self.msg)
            self.ui.L_save.setText('已经发布路径，导航中--------')
    def on_button_clicked(self):
        self.wide = float(self.ui.l_wide.text())
        self.speed = float(self.ui.l_speed.text())
        self.angle = float(self.ui.l_angle.text())
        self.rad = float(self.ui.l_rad.text())
        self.ui.L_save.setText(f'已经更新参数:\n'
                       f'wide: {self.wide}\n'
                       f'speed: {self.speed}\n'
                       f'angle: {self.angle}\n'
                       f'rad: {self.rad}')

       
        
    def init_ros_publisher(self, node):
        self.publisher_ = node.create_publisher(String, '/qt_control', 10)
class CoordinateSubscriber(Node):
    def __init__(self):
        super().__init__('coordinate_subscriber')
        self.subscription = self.create_subscription(PolygonStamped,'/polygon',self.listener_callback,10)
        self.sub = self.create_subscription(String, '/str_control', self.congfig_back,10)
        self.wide_sub = self.create_subscription(String, '/qt_control', self.update_wide, 10)
        self.path_publisher = self.create_publisher(Path, '/path', 10)
        self.wide = 12

    def update_wide(self, msg):
        try:
            data = json.loads(msg.data)
            self.wide = float(data['wide'])
            print(f'qt已经更新: wide:--{self.wide}--')
        except (json.JSONDecodeError, KeyError, ValueError) as e:
            self.get_logger().error('Failed to parse JSON: %s' % str(e))
    def congfig_back(self, msg):
        try:
            data = json.loads(msg.data)
            self.wide = float(data['wide'])
            print(f'mqtt已经更新: wide:--{self.wide}--')
        except (json.JSONDecodeError, KeyError, ValueError) as e:
            self.get_logger().error('Failed to parse JSON: %s' % str(e))

    def listener_callback(self, msg):
        # 创建坐标转换器对象并进行转换
        transformer = Coordinateself()
        
        or_points = [(point.x, point.y) for point in msg.polygon.points]
        path_lis, angel_for_back = transformer.s_rote(or_points, self.wide)
        dx = float(or_points[0][0] )
        dy = float(or_points[0][1] ) 

        last_p=path_lis[-1]
        if len(path_lis) % 2 != 0:
                # 如果是奇数，去掉最后一个元素
            ok_l=path_lis[:-1]
        else:
                # 如果是偶数，直接返回原列表
            ok_l=path_lis
        dx = float(or_points[0][0] )
        dy = float(or_points[0][1] ) 
        path_li= transformer.point_extraction(ok_l,self.wide)
        path_li.append(last_p)
        path_li.insert(0,path_lis[0])


        path_list=transformer.back_transform(path_li,-angel_for_back)
        path_list  = [(ax + dx, ay + dy) for ax, ay in path_list ]
        path_list = [list(point) for point in path_list]

        
        # 创建 Path 消息
        path_msg = Path()
        path_msg.header.frame_id = 'map'  # 假设地图坐标系为 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()

        # for point in adjusted_points:
        for point in path_list:
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = 'map'
            pose_stamped.header.stamp = path_msg.header.stamp
            pose_stamped.pose.position.x = point[0]
            pose_stamped.pose.position.y = point[1]
            pose_stamped.pose.position.z = 0.0
            pose_stamped.pose.orientation.x = 0.0
            pose_stamped.pose.orientation.y = 0.0
            pose_stamped.pose.orientation.z = 0.0
            pose_stamped.pose.orientation.w = 1.0
            path_msg.poses.append(pose_stamped)

        # 发布 Path 消息
        self.path_publisher.publish(path_msg)
       

def main(args=None):
    rclpy.init(args=args)
    
    coordinate_subscriber = CoordinateSubscriber()
    
    # 启动GUI线程
    gui_runner = GUIRunner(coordinate_subscriber)
    gui_runner.start()
    
    try:
        rclpy.spin(coordinate_subscriber)
    finally:
        coordinate_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()