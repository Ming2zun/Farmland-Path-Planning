# Farmland-Path-Planning
- **当前版本**: 1.0
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

## Farmland-Path-Planning即将迎来升级版本：Farmland-Path-Planning2（FPP2）
# FPP2现与“丘沃智能科技有限公司”合作赋能农机田间自动路径规划
# FPP2属于私密状态暂不开源
# 提供了Ω型转弯
<img width="1988" height="1136" alt="image" src="https://github.com/user-attachments/assets/8eac93a9-7965-4e75-9cfd-770404bea28e" />

# 提供可以倒车的鱼尾形状倒车（覆盖率99%以上）
<img width="2080" height="1202" alt="image" src="https://github.com/user-attachments/assets/458376cb-da54-4daa-9484-2577ebbbae4c" />

# 不能倒车但是窄距作业的跨行掉头模式(覆盖率98%以上)
<img width="1978" height="1124" alt="image" src="https://github.com/user-attachments/assets/738a8fd3-9aa6-4a8c-a105-e77ac4bd3f10" />
# 保留了传统的不考虑转弯半径的U掉头（覆盖率98%以上）
<img width="2076" height="1190" alt="image" src="https://github.com/user-attachments/assets/3bd9a1a1-20ea-4409-a8b9-fc7ce58caca1" />

以上图片均以实际算法生成图片



