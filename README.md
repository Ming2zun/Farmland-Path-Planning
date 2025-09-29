# Farmland-Path-Planning
- **版本**: 1.0
## 提供了ROS2-humble分支-已经集成到ros2系统，切换分支查看

<img width="398" height="273" alt="ChatGPT Image 2025年9月29日 10_26_05" src="https://github.com/user-attachments/assets/2fc6db5e-63c0-4ebc-b1a9-ed1903825092" />


1.提供了简单的python接口可运行：s_rote.py 和 o_rote.py

2.上传坐标点时，请按照下图原理解析，按照顺序确定田块的边界点p1 ,p2,p3,p4（必须按照顺序）

3.s_rote.py可直接生成全覆盖路径包含掉头路径，可直接计算覆盖率并保存路径点到yaml文件，pub_path_topic.py可把保存的路径点发布一次到ROS2话题，用后续导航。


![全覆盖路径规划](https://github.com/user-attachments/assets/8396f629-0bed-46e8-b17f-d93cff43deb8)

Farmland cover path planning
Framland-Path-Planning algorithm can pass in any farmland boundary point, and generate path navigation points covering the whole farmland according to the job width.
视频展示：https://www.bilibili.com/video/BV1jahNeLEM1/?spm_id_from=333.999.0.0
![farmpp](https://github.com/Ming2zun/Framland-Path-Planning/assets/140699846/363ea8a1-7ec7-41c3-944d-d618d275e6ff)

If you want to know, please contact: clibang2022@163.com
