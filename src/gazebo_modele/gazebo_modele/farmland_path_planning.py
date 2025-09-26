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
import math
import numpy as np
class Coordinateself:
#--------------------------------------------------------------------------
# 坐标转换函数

    def transform(self, points):
        """
        将给定的4个坐标点进行坐标系转换
        参数:
        points (list): 包含4个坐标点的列表, 例如 [(3,6),(18,5),(19,28),(4,25)]
        返回:
        list: 转换后的4个坐标点
        """
        # 新坐标系的前2个点
        distance = ((points[1][0] - points[0][0])**2 + (points[1][1] - points[0][1])**2)**0.5
        new_points = [(0,0), (distance, 0)]
        # 先进行平移
        b = []
        for point in points:
            b.append((point[0] - points[0][0], point[1] - points[0][1]))
        # 计算旋转角度
        theta = -math.atan2(points[1][1] - points[0][1], points[1][0] - points[0][0])     
        # 进行旋转
        c = []
        for point in b:
            c.append((
                point[0] * math.cos(theta) - point[1] * math.sin(theta) + new_points[0][0],
                point[0] * math.sin(theta) + point[1] * math.cos(theta) + new_points[0][1]
            ))
        return c ,theta
#--------------------------------------------------------------------------
# 回转函数，再转换回来
    def back_transform(self,points, angle):
        """
        将给定的坐标点列表绕原点旋转一定角度
        参数:
        points (list): 包含坐标点的列表
        angle (float): 旋转角度，单位为弧度
        返回:
        list: 旋转后的坐标点列表
        """
        rotated_points = []
        for point in points:
            # 计算旋转后的坐标点
            x = point[0] * math.cos(angle) - point[1] * math.sin(angle)
            y = point[0] * math.sin(angle) + point[1] * math.cos(angle)
            rotated_points.append((x, y))
        return rotated_points
#--------------------------------------------------------------------------
# 更具坐标生成直线方程函数
    @staticmethod
    def calculate_slope_intercept(point1, point2):
        x1, y1 = point1
        x2, y2 = point2
        if x1 == x2:
            slope = float('inf')  # 斜率为无穷大表示垂直于x轴的直线
            intercept = x1  # 在垂直于x轴的直线上，intercept就是x轴的值
        else:
            slope = (y2 - y1) / (x2 - x1)
            intercept = y1 - slope * x1
        return slope, intercept

    def calculate_line_equations(self, points):
        line_equations = []
        for i in range(len(points) - 1):
            line_equations.append(self.calculate_slope_intercept(points[i], points[i + 1]))
        line_equations.append(self.calculate_slope_intercept(points[-1], points[0]))
        return line_equations
#--------------------------------------------------------------------------
# 选择方程求x点函数
    def calculate_x_coordinate(self,slope, intercept, known_y):
        x = (known_y - intercept) / slope
        return x
#--------------------------------------------------------------------------
# 选择方程求y点函数
    def calculate_y_coordinate(self,slope, intercept, known_x):
        y = slope * known_x + intercept
        return y

#--------------------------------------------------------------------------
    def move_horizontally(self,line_equations, with_co, y_plass, limit3,limit4):
    # 计算线性方程
        line_equation =self.small_quad_xiao(line_equations, with_co)
        l2 = line_equation[1]
        l3 = line_equation[2]
        l4 = line_equation[3]
        x2_list, x3_list, x4_list = [], [], []
        
        if limit3 < limit4:
            while (y_plass) < limit4:
                x_4 = self.calculate_x_coordinate(l4[0], l4[1], y_plass)
                if y_plass < limit3 and y_plass < limit4:
                    x_2 = self.calculate_x_coordinate(l2[0], l2[1], y_plass)
                    x2_list.append((x_2,y_plass))
                    
                else:
                    x_3 = self.calculate_x_coordinate(l3[0], l3[1], y_plass)
                    if x_3>=x_4: 
                        x3_list.append((x_3,y_plass))
                    else:
                        x3_list.append((x_4,y_plass))
                        x4_list.append((x_4,y_plass))
                        break
                x4_list.append((x_4,y_plass))
                y_plass += with_co
        else:
            while (y_plass) < limit3:
                x_2 = self.calculate_x_coordinate(l2[0], l2[1], y_plass)
                if y_plass < limit4 and y_plass < limit3:
                    
                    x_4 = self.calculate_x_coordinate(l4[0], l4[1], y_plass)
                    x4_list.append((x_4,y_plass))
                else:
                    x_3 = self.calculate_x_coordinate(l3[0], l3[1], y_plass)
                    if x_3<=x_2: 
                        x3_list.append((x_3,y_plass))
                    else:
                        x3_list.append((x_2,y_plass))
                        x2_list.append((x_2,y_plass))
                        break
                
                x2_list.append((x_2,y_plass))
                y_plass += with_co

        return  x2_list, x3_list, x4_list
#--------------------------------------------------------------------------
    def small_quad(self,line_equations, with_co):
        # 返回一个小四边形
        l2 = line_equations[1]
        l3 = line_equations[2]
        l4 = line_equations[3]
        #  x 轴的夹角（弧度）
        angle_2 = math.atan(l2[0])
        angle_3 = math.atan(l3[0])
        angle_4 = math.atan(l4[0])
       
        y_1 = with_co
        # y_2 = move_2/l2[0]
        if l2[0] < 0:
            y_2 = -abs(with_co/math.cos(angle_2))
        else:
            y_2 = abs(with_co/math.cos(angle_2))
        y_3 = -abs(with_co/math.cos(angle_3))
        if l4[0] <0:
            y_4 = abs(with_co/math.cos(angle_4))
        else:
            y_4 = -abs(with_co/math.cos(angle_4))
        
        y_list = [y_1,y_2,y_3,y_4]

        line_equ_new =  [(x, y + y_list[i]) for i, (x, y) in enumerate(line_equations)]
        return line_equ_new
#--------------------------------------------------------------------------

    def small_quad_xiao(self,line_equations, with_c):
        # 返回一个小四边形
        with_co = with_c/2
        l2 = line_equations[1]
        l3 = line_equations[2]
        l4 = line_equations[3]
        #  x 轴的夹角（弧度）
        angle_2 = math.atan(l2[0])
        angle_3 = math.atan(l3[0])
        angle_4 = math.atan(l4[0])
       
        y_1 = with_co
        # y_2 = move_2/l2[0]
        if l2[0] < 0:
            y_2 = -abs(with_co/math.cos(angle_2))
        else:
            y_2 = abs(with_co/math.cos(angle_2))
        y_3 = -abs(with_co/math.cos(angle_3))
        if l4[0] <0:
            y_4 = abs(with_co/math.cos(angle_4))
        else:
            y_4 = -abs(with_co/math.cos(angle_4))
        
        y_list = [y_1,y_2,y_3,y_4]

        line_equ_new =  [(x, y + y_list[i]) for i, (x, y) in enumerate(line_equations)]
        return line_equ_new
#--------------------------------------------------------------------------


    # 计算了两条直线的交点
    def find_Xpoint(self,line1, line2):
        m1, b1 = line1
        m2, b2 = line2
        x = (b2 - b1) / (m1 - m2)
        y = m1 * x + b1
        return (x, y)


    def intersection_points(self,line_equations,with_co):
#计算4条线的交点
        def intersection_point(m1, b1, m2, b2):
            # 计算交点的x坐标
            x = (b2 - b1) / (m1 - m2)
            # 使用任何一条线的方程来计算交点的y坐标
            y = m1 * x + b1
            return x, y

        # 存储交点的列表
        b = []
        for i in range(len(line_equations) - 1):
            m1, b1 = line_equations[i]
            m2, b2 = line_equations[i + 1]
            intersection = intersection_point(m1, b1, m2, b2)
            b.append(intersection)
        
        # 计算最后一条线与第一条线的交点
        m1, b1 = line_equations[-1]
        m2, b2 = line_equations[0]
        intersection = intersection_point(m1, b1, m2, b2)
        b.append(intersection)
        c=(b[3][0]+abs(with_co / math.sin(math.atan(line_equations[3][0]))),b[3][1])
        b.reverse()
        b.append(c)
        return b
    
    def loop (self,line_equations,with_co):
        #遍历回环收割路径，直到完成作业宽度
        c=[]
        a=0
        while a<20:
            equ_i = self.small_quad(line_equations,with_co)
            b=self.intersection_points(equ_i,with_co)
            if (b[3][0]-b[0][0])<=with_co and (b[2][0]-b[1][0])<with_co:
                a=100
                c.append(b[0])
                c.append(b[1])
                
            else:
                if (b[1][1]-b[0][1])<with_co and (b[2][1]-b[3][1])<with_co:
                    a=100
                    c.append(b[0])
                    c.append(b[1])
                    
                else:
                    a=0
                    line_equations = equ_i
                c=c+b
        return c
    
    def complete_path(self,list_a, list_b):
        # 传入2个列表生成一个s 型路径
        new_list = []
        new_list.append(list_a[0])
        for i in range(1, len(list_a), 2):
            new_list.extend(list_b[i-1:i+1])
            new_list.extend(list_a[i:i+2])
        if len(list_a) % 2 == 1:
            new_list.append(list_b[-1])
        return new_list


# 直接生成s型路径--------------------------------------------------------------------------------------
    def s_rote(self,or_points,working_wide):
        with_co = working_wide
        y_plass = working_wide/2
        points,angel_for_back = self.transform(or_points)
        line_equations = self.calculate_line_equations(points)
        limit3y = points[2][1]
        limit4y = points[3][1]
        x2_list, x3_list, x4_list = self.move_horizontally(line_equations, with_co, y_plass, limit3y,limit4y)
        # # 获取每个列表的长度
        len_list2 = len(x2_list)
        len_list4 = len(x4_list)
        if len_list4 > len_list2 :
            list_he = x2_list + x3_list
            list_max = x4_list
        else:
            list_he = x4_list + x3_list
            list_max = x2_list
        list5 = self.complete_path(list_max, list_he)
        dx = float(or_points[0][0] )
        dy = float(or_points[0][1] ) 
        # 使用列表推导式来更新列表a中的点,旋转，平移
        # po = self.back_transform(list5, -angel_for_back)
        # list5  = [(ax + dx, ay + dy) for ax, ay in po ]
        return list5,angel_for_back
# 直接生成s型路径--------------------------------------------------------------------------------------


# 直接生成o型路径------------------------------------------------------------------------------------
    def o_rote(self,or_points,with_co):
        points,angel_for_back = self.transform(or_points)

        line_equations = self.calculate_line_equations(points)

        p =self.loop (line_equations,with_co)
        dx = float(or_points[0][0] )
        dy = float(or_points[0][1] ) 
# 旋转，平移
        po = self.back_transform(p, -angel_for_back)
        path_list  = [(ax + dx, ay + dy) for ax, ay in po ]

        return path_list 
# 直接生成o型路径------------------------------------------------------------------------------------

# ABC和角度提取点提取
    def point_extraction(self,security_route,d):
        list_all=[]
        def process_points(A, B, C):
    # 计算线段AC和线段AB的角度
            angle_AC = math.atan2(C[1] - A[1], C[0] - A[0])
            angle_AB = math.atan2(B[1] - A[1], B[0] - A[0])
            # 计算角CAB的度数
            angle_CAB_radians = abs(angle_AB - angle_AC)
            # 如果角CAB超过pi（180度），则需要进行调整，因为我们寻找的是内角
            if angle_CAB_radians > math.pi:
                angle_CAB_radians = 2 * math.pi - angle_CAB_radians
            # 将弧度转换为度数
            # angle_CAB_degrees = math.degrees(angle_CAB_radians)
            return angle_CAB_radians
        for i in range(0, len(security_route) - 2, 2):
            A = security_route[i+1]
            B = security_route[i+2]
            C = security_route[i]
            angel=process_points(A, B, C)
            dx=C[0]-A[0]
            list_c,midle=self.interpolate_points_on_arc(A, B,angel,d)
            po_num = int(d/2)
            if angel >= 1.57:
                po_ns = 4*po_num
                po_n = po_num
            else:
                po_ns = po_num
                po_n = 4*po_num
            points1=self.interpolate_points_on_circle(list_c[0][0],list_c[0][1],list_c[0][2],midle,po_ns,dx)
            points2=self.interpolate_points_on_circle(list_c[1][0],list_c[1][1],midle,list_c[1][2],po_n,dx)
            # print(angel,po_ns,po_n)
            points_ALL=points1+points2
            points_ALL.insert(0,A)
            points_ALL.append(B)
            list_all+=points_ALL
            
        return list_all
    # 传入圆心，半径，起始点，结束点，插值数量进行插值
    def interpolate_points_on_circle(self,center, radius, point1, point2, num_points, dx):
        angle1 = np.arctan2(point1[1] - center[1], point1[0] - center[0])
        angle2 = np.arctan2(point2[1] - center[1], point2[0] - center[0])
        
        # 根据dx判断方向
        if dx > 0:  # 顺时针
            if angle2 > angle1:
                angle2 -= 2 * np.pi
        else:       # 逆时针
            if angle2 < angle1:
                angle2 += 2 * np.pi    
        # 创建角度数组
        angles = np.linspace(angle1, angle2, num_points + 1)
        # 计算坐标
        x_coords = center[0] + radius * np.cos(angles)
        y_coords = center[1] + radius * np.sin(angles)
        # 获取插值点，去除起始点的重复
        points = list(zip(x_coords[1:], y_coords[1:]))
        return points
# 各个圆心计算
    def interpolate_points_on_arc(self,A, B, ang_le ,d):
        x1, y1 = A
        x2, y2 = B
        l_distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        def angle_with_y_axis(po1, po2):
            x11,y11 = po1
            x22,y22 =po2
            if x22 - x11 == 0:  # 防止除以零的情况
                return 0.0 if y11 == y22 else math.pi / 2
            slope = (y22 - y11) / (x22 - x11)
            angle_with_x_axis = math.atan(abs(slope))
            angle_with_y_axis = math.pi / 2 - angle_with_x_axis
            return angle_with_y_axis
        
        if ang_le >= 1.57:
            rb=0.25*d*math.sin(ang_le)
            ra=0.5*abs(l_distance/(math.sin(ang_le)))-rb
            orb=(x2,y2-rb)
            ora=(x1,y1+ra)
            l_c = math.sqrt((x2 - x1)**2+(y2-y1-rb)**2)
            an=angle_with_y_axis(orb, A)
            r3 = abs((l_c*l_c -rb*rb)/(2*(l_c*math.cos(an)-rb)))
            or3=(x1,y1+r3)
            list2=[[or3,r3,A],[orb,rb,B]]
            mide=self.tangent_points(list2[0][0], list2[0][1], list2[1][0], list2[1][1])
            # print(list2[0],'2')
            return list2,mide
        else:
            ra=0.25*d*math.sin(ang_le)
            rb=0.5*abs(l_distance/(math.sin(ang_le)))-ra
            ora=(x1,y1+ra)
            
            orb=(x2,y2-rb)
            l_c = math.sqrt((x2 - x1)**2+(y2-y1-ra)**2)
            an=angle_with_y_axis(ora, B)
            r3 = abs((l_c*l_c -ra*ra)/(2*(l_c*math.cos(an)-ra)))
            or3=(x2,y2-r3)
            
            list2=[[ora,ra,A],[or3,r3,B]]
            # print(list2[0])
            mide=self.tangent_points(list2[1][0], list2[1][1], list2[0][0], list2[0][1])
            return list2,mide
# 计算切点
    def tangent_points(self,c1, r1, c2, r2):
        # 计算两个圆心之间的距离
        distance = ((c2[0] - c1[0]) ** 2 + (c2[1] - c1[1]) ** 2) ** 0.5
        dx, dy = c2[0] - c1[0], c2[1] - c1[1]
        # 单位向量
        unit_dx, unit_dy = dx / distance, dy / distance
        # 切点坐标计算
        point1 = (c1[0] + r1 * unit_dx, c1[1] + r1 * unit_dy)
        
        return point1