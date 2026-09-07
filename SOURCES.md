# 来源与许可说明

本作品集整理 Wenshao Lyu 的机器人算法实现、验证与系统集成项目。个人贡献以各项目 README 为准。

## ROS 参考来源

差速底盘基于 FishBot／鱼香 ROS 教程学习完成，个人工作包括核心流程理解与实现、参数配置、通信及设备集成、实机调试。第三方驱动和桥接组件不作为个人原创算法声明。

以下组件保留本地源码快照、作者说明及已有许可文件，采用普通目录收录，不使用 Git 子模块。提交号取自原始目录的 Git 引用，用于标识快照基线；本地文件可能包含配置或集成修改，不声称与上游提交逐字相同。

| 组件 | 上游仓库 | 本地基线提交 |
|---|---|---|
| [micro-ROS-Agent](ros2-differential-drive/fishbot_ws/src/micro-ROS-Agent/) | [micro-ROS/micro-ROS-Agent](https://github.com/micro-ROS/micro-ROS-Agent) | `52abdf5a9897637b2539cb1c239e1ec4fa92e01c` |
| [micro_ros_msgs](ros2-differential-drive/fishbot_ws/src/micro_ros_msgs/) | [micro-ROS/micro_ros_msgs](https://github.com/micro-ROS/micro_ros_msgs) | `e65ab21bd0733ebff2af6317d573cc584efa5893` |
| [ros_serial2wifi](ros2-differential-drive/fishbot_ws/src/ros_serial2wifi/) | [fishros/ros_serial2wifi](https://github.com/fishros/ros_serial2wifi) | `bd677e5cea11632a875e5f98d9d54ddf209f87f3` |
| [ydlidar_ros2](ros2-differential-drive/fishbot_ws/src/ydlidar_ros2/) | [fishros/ydlidar_ros2](https://github.com/fishros/ydlidar_ros2) | `180d5847450888789c2bbbb971be66055da41bfb` |

固件还调用 `micro_ros_platformio`、`Esp32McpwmMotor` 和 `Esp32PcntEncoder`，相关依赖未完整随作品集分发，详见 ROS 项目说明。

## 许可范围

各组件已有 LICENSE、NOTICE、文件头及第三方许可说明保留原样。本作品集未添加覆盖全部代码的统一开源许可证；不得将某个子目录的许可证自动套用到其他内容。源码快照中未附独立许可文件的组件，也不据此推定获得额外授权。

UR5e 与 EKF 展示核心算法实现及验证，未附原始课程材料、外部传感器数据集或完整硬件运行环境。

## 发布副本的文档调整

修正 YDLIDAR 配置文件相对路径；去掉未随快照提供的上游图片引用；将 micro_ros_msgs 缺失 NOTICE 的链接改为文字并注明缺失；去掉 micro-ROS Agent 原有 TODO 链接。以上只涉及文档，未补写或替换任何许可证。
