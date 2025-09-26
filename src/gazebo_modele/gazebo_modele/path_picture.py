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
import yaml
import matplotlib.pyplot as plt


class Pathpicture:
    def plot_from_yaml(self,list1, list2):
        list2 = list2 + [list2[0]]
        x1, y1 = zip(*list1)
        x2, y2 = zip(*list2)
        
        
        # 绘制第一条线
        plt.plot(x1, y1, label='Line 1', marker='o', linestyle='-')
        
        # 绘制第二条线
        plt.plot(x2, y2, label='Line 2', marker='s', linestyle='-', color='orange')

        plt.xlabel('X values')
        plt.ylabel('Y values')
        plt.title('Connected Coordinate Points')
        plt.legend()
        plt.grid(True)
        plt.show()

# 使用示例
