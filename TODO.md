📂 ROS 2 Humble 完整课程大纲
01 ROS2 简介 (Introduction)

ROS 2 的核心概念、架构优势及与 ROS 1 的区别。

02 安装 Humble (Installation)

Ubuntu 22.04 下的安装配置、环境检测。

03 集成开发环境 (IDE Setup)

VS Code 配置、插件安装、调试环境搭建。

04 工作区 (Workspace)

Colcon 编译系统、工作空间覆盖（Overlaying）。

05 功能包 (Packages)

创建 C++ 和 Python 功能包、依赖管理 (package.xml, CMakeLists.txt, setup.py)。

06 节点 (Nodes)

面向对象编程风格的节点编写、生命周期管理。

07 话题通讯 (Topics)

发布者（Publisher）与订阅者（Subscriber）模型、QoS 策略。

08 服务通讯 (Services)

客户端（Client）与服务端（Server）模型、同步与异步调用。

09 动作通讯 (Actions)

长周期任务处理、反馈机制（Feedback）、取消目标。

10 TF2 坐标变换 (TF2 Transform)

坐标系管理、静态/动态广播、监听变换、TF 树工具。

11 自定义接口消息 (Custom Interfaces)

.msg, .srv, .action 文件定义及编译依赖配置。

12 参数服务案例 (Parameters)

动态参数配置、参数回调、YAML 参数文件加载。

13 元功能包 (Metapackages)

组织大型项目、虚包依赖管理。

14 分布式通讯 (Distributed Communication)

多机通讯配置、ROS_DOMAIN_ID 设置。

15 DDS (Data Distribution Service)

DDS 原理、中间件切换（CycloneDDS / FastDDS）、RMW 配置。

16 时间相关 API (Time APIs)

ROS Time、Duration、Rate 及与系统时间的转换。

17 常用命令工具 (CLI Tools)

ros2 topic/node/service/param/bag 等命令行工具全解。

18 RViz2 使用 (RViz2 Visualization)

数据可视化配置、插件使用、保存配置 (.rviz)。

19 Rqt 工具箱 (Rqt Tools)

图表绘制 (rqt_plot)、节点图 (rqt_graph)、日志查看 (rqt_console)。

20 Launch 配置 (Launch Files)

Python Launch 文件编写、节点启动管理、参数传递、命名空间。

21 录制回放 (Rosbag2)

数据包录制、回放、信息查看及 SQLite3 存储格式。

22 URDF 模型 (URDF Modeling)

机器人统一描述格式、Xacro 宏定义、Joint/Link 配置。

23 Gazebo 仿真 (Gazebo Simulation)

Gazebo Classic / Ignition (Gazebo) 集成、仿真环境搭建、传感器插件。

24 摄像头预览 (Camera Preview)

USB 摄像头驱动、image_transport、压缩图像传输。

25 摄像头校准 (Camera Calibration)

相机内参标定、畸变矫正 (camera_calibration 包)。

26 AR 视觉 (AR Vision)

基于 ArUco Marker 的位姿估计、增强现实应用基础。