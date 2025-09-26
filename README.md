# Farmland-Path-Planning
## 这里是ROS2—humble分支，集成路径规划加导航
# 需要安装：mapviz，pyqt6，pyside6

###  启动仿真
```
ros2 launch gazebo_modele gazebo.launch.py
```
<img width="1476" height="804" alt="image" src="https://github.com/user-attachments/assets/150644d9-4a39-466f-989b-68955e358afb" />
包含3维激光雷达，imu，gps，相机传感器。


###  启动导航
导航沿用之前的工艺详情：https://github.com/Ming2zun/Pure-tracking-slam-automatic-navigation-system
```

ros2 launch nav_slam 2dpoints.launch.py
```
###  启动mapviz
```
ros2 launch gazebo_modele mapviz.launch.py
```
<img width="2754" height="1602" alt="image" src="https://github.com/user-attachments/assets/0973b3a0-5aa4-41d3-ab6e-382a404fbe76" />


If you want to know, please contact: clibang2022@163.com
