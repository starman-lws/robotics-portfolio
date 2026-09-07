# Robotics Portfolio | Wenshao Lyu

机器人算法与软件作品集，展示机械臂运动学、路径规划、状态估计与移动底盘控制方面的核心实现、验证和实机集成经验。主要使用 MATLAB、C++、Python、ROS 2 与 ESP32。

## 项目导航

| 项目 | 解决的问题与个人贡献 | 核心技术 | 验证方式 |
|---|---|---|---|
| [UR5e 运动学与奇异性分析](ur5e-kinematics/README.md) | 实现正运动学、Jacobian、奇异性诊断和简化配置有效性检查，构建固定肘关节后的等效五自由度模型 | MATLAB、标准 DH、SVD、几何碰撞检测 | 模型一致性与数值差分检查；几何可视化 |
| [UR5e 路径规划与视觉引导操作](ur5e-planning-manipulation/README.md) | 实现视觉定位与路径规划核心算法，处理矩形物体的位置与朝向耦合，联动机械臂完成拾放和倾斜插入 | ArUco、单应变换、APF、二维／SE(2) A*、RTDE | 原始集成流程通过实机测试；作品集模块完成离线回归 |
| [ROS 2 差速底盘控制](ros2-differential-drive/README.md) | 基于教程完成底盘控制流程学习实现、参数配置、通信与设备集成和实机调试 | C++、Python、PI、micro-ROS、ESP32、URDF | 实机联调；未提供量化性能基准 |
| [IMU 与平面观测融合估计](imu-plane-ekf/README.md) | 实现点云平面拟合、法向量关联、陀螺仪偏置估计及六维增广状态 EKF 核心算法 | MATLAB、ZYX Euler、SVD、EKF、Joseph 更新 | 记录数据回放与合成数值验证 |

## UR5e 运动学与奇异性分析

从标准 DH 参数出发计算末端位姿和几何 Jacobian，结合解析奇异因子与 SVD 指标诊断肩、肘、腕奇异性。将连杆近似为零厚度线段进行球体／AABB 相交检查，并验证肘关节固定在 90° 后的五自由度等效模型。

<img src="ur5e-kinematics/docs/images/workspace_combined.png" alt="UR5e 工作空间与组合障碍物的几何可视化" width="680">

图示为工作空间与障碍物的几何关系，不代表完整机械臂实体碰撞验证。

[算法说明](ur5e-kinematics/README.md) · [核心代码](ur5e-kinematics/src/)

## UR5e 路径规划与视觉引导操作

利用 ArUco 与单应变换恢复平面场景位置及朝向；正方形物体采用 APF／二维 A*，矩形物体采用 APF／SE(2) A*，在 A* 搜索和平滑阶段检查中心及角点采样距离约束。通过 RTDE 与真空吸盘完成视觉引导拾放及 45° 倾斜插入的实机任务。

<img src="ur5e-planning-manipulation/docs/images/large_item_astar_result.png" alt="矩形物体 SE(2) A* 离线路径规划结果" width="680">

三个离线场景的 SE(2) A* 均到达目标，平滑路径通过设定的采样约束。上图为离线结果；原始集成流程已由作者在 UR5e 上完成实机测试，作品集模块化整理后的回归不作为新一轮实机测试。

[算法与验证说明](ur5e-planning-manipulation/README.md) · [核心代码](ur5e-planning-manipulation/src/)

## ROS 2 与 ESP32 差速底盘控制

车体速度指令经差速逆运动学转换为左右轮目标速度，由编码器反馈与双轮 PI 调节电机输出；micro-ROS 连接上位机与 ESP32，Python Launch 编排 TF、URDF 和雷达节点，并集成 TCP-伪串口桥接。

已完成上位机指令、底盘控制和里程计反馈的实机联调。该项目基于 FishBot／鱼香 ROS 教程学习完成，个人贡献与第三方组件范围在项目说明中单独列出。

[系统结构与实机范围](ros2-differential-drive/README.md) · [底盘固件](ros2-differential-drive/motion_control/) · [ROS 2 集成代码](ros2-differential-drive/fishbot_ws/src/fishbot_bringup/)

## IMU 与平面观测融合估计

针对陀螺仪偏置导致的姿态积分漂移，实现六维增广状态 EKF，联合估计三轴姿态与恒定偏置；从给定点云拟合平面，并利用已关联的有向法向量进行观测修正。

<img src="imu-plane-ekf/docs/images/ekf_bias_estimate.png" alt="记录数据回放中注入恒定偏置后的 EKF 偏置估计曲线" width="680">

图示来自记录数据回放，用于观察注入恒定偏置后的收敛行为。合成检查用于核对公式及协方差数值性质，不作为传感器精度或实时性能基准。

[模型与验证说明](imu-plane-ekf/README.md) · [EKF 核心代码](imu-plane-ekf/augmented-state-ekf/)

## 阅读与使用

- 本仓库以代码和算法展示为主；各项目 README 包含公式、接口、结果及适用范围。
- 数据集、部分第三方运行依赖和完整硬件环境未全部分发，不能将作品集视为一键运行系统。
- ROS 发布副本中的 Wi-Fi 配置为占位值，Agent 地址为文档示例地址，运行前需要自行配置。
- 实机操作依赖正确的设备配置与工作区检查；本次作品集整理不触发硬件运行。
- 参考来源、第三方快照版本与许可范围见 [SOURCES.md](SOURCES.md)。
