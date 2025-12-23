# 18 RViz2 使用 (RViz2 Visualization)

## 18.1 RViz2 概述

### 18.1.1 什么是 RViz2

**RViz2 (ROS Visualization 2)** 是 ROS 2 的 3D 可视化工具，用于显示机器人模型、传感器数据、TF 坐标系等信息。

```
RViz2 界面布局：

┌──────────────────────────────────────────────────────────┐
│  菜单栏: File Panels View Tools Help                     │
├──────────────┬─────────────────────────────────┬─────────┤
│              │                                 │         │
│  左侧面板     │       3D 视图区域               │  右侧   │
│              │                                 │  面板   │
│  ┌─────────┐ │                                 │         │
│  │ Displays│ │                                 │ ┌─────┐ │
│  ├─────────┤ │         [机器人模型/点云]       │ │Views│ │
│  │ Add     │ │                                 │ ├─────┤ │
│  ├─────────┤ │                                 │ │Time │ │
│  │ TF      │ │                                 │ ├─────┤ │
│  ├─────────┤ │                                 │ │Tools│ │
│  │ Camera  │ │                                 │ ├─────┤ │
│  ├─────────┤ │                                 │ │Selection│
│  │ ...     │ │                                 │ └─────┘ │
│  └─────────┘ │                                 │         │
├──────────────┴─────────────────────────────────┴─────────┤
│  状态栏: FPS | TF | Grid | Time                     │
└──────────────────────────────────────────────────────────┘
```

### 18.1.2 RViz2 功能

| 功能 | 说明 |
|-----|------|
| **3D 模型显示** | 显示 URDF/机器人模型 |
| **传感器数据** | 点云、图像、激光扫描 |
| **TF 可视化** | 坐标变换树 |
| **导航可视化** | 路径、地图、规划 |
| **数据回放** | 与 rosbag 集成 |

## 18.2 启动 RViz2

### 18.2.1 基本启动

```bash
# 启动 RViz2
ros2 run rviz2 rviz2

# 启动带配置文件
ros2 run rviz2 rviz2 --display-config config/my_config.rviz

# 从 launch 文件启动
```

### 18.2.2 从 Launch 文件启动

```python
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', '/path/to/config.rviz'],
            output='screen'
        )
    ])
```

## 18.3 Displays 面板

### 18.3.1 添加显示项

```
常用显示类型：

├── Grid                    # 网格
├── TF                     # 坐标变换
├── RobotModel             # 机器人模型
├── Camera                 # 相机图像
├── LaserScan              # 激光雷达
├── PointCloud             # 点云
├── Path                   # 路径
├── Pose                   # 位姿
├── Marker                 # 标记
└── Map                    # 地图
```

### 18.3.2 配置 Grid（网格）

| 设置 | 说明 |
|-----|------|
| Reference Frame | 参考坐标系 |
| Size | 网格大小 |
| Plane | 网格平面 (XY/XZ/YZ) |
| Cell Size | 单元格大小 |

### 18.3.3 配置 TF（坐标变换）

| 设置 | 说明 |
|-----|------|
| Show Axes | 显示坐标轴 |
| Show Arrows | 显示箭头 |
| Scale | 缩放比例 |
| Marker Scale | 标记大小 |

### 18.3.4 配置 RobotModel（机器人模型）

| 设置 | 说明 |
|-----|------|
| Description Source | URDF 来源 |
| Description Topic | robot_state |
| TF Prefix | TF 前缀 |
| Visual Enabled | 显示可视化 |
| Collision Enabled | 显示碰撞体 |

## 18.4 传感器数据可视化

### 18.4.1 LaserScan（激光雷达）

```bash
# 启动激光雷达显示
# 在 Displays 中添加 -> LaserScan
# 设置 Topic 为 /scan
```

| 设置 | 说明 |
|-----|------|
| Topic | 激光数据话题 |
| Size (m) | 射线长度 |
| Color Transformer | 颜色映射 |
| Min/Max Intensity | 强度范围 |

### 18.4.2 PointCloud（点云）

| 设置 | 说明 |
|-----|------|
| Topic | 点云话题 |
| Size (m) | 点大小 |
| Color Transformer | 颜色变换 |
| Decay Time | 点保留时间 |

### 18.4.3 Camera（相机）

```bash
# 启动相机显示
# 添加 -> Camera
# 设置 Image Topic 为 /camera/image_raw
```

| 设置 | 说明 |
|-----|------|
| Image Topic | 图像话题 |
| Camera Info Topic | 相机信息 |
| Image Rendering | 图像渲染 |

## 18.5 视图控制

### 18.5.1 鼠标控制

| 操作 | 功能 |
|-----|------|
| 左键拖动 | 旋转视图 |
| 中键拖动 | 平移视图 |
| 滚轮 | 缩放 |
| Ctrl+左键 | 选择对象 |
| Shift+左键 | 选择框架 |

### 18.5.2 视图预设

```
Views 菜单：

├── Orbit (默认)          # 轨道视图
├── Top                   # 俯视图
├── Left                  # 左视图
├── Right                 # 右视图
├── Front                 # 前视图
└── FPS                   # 第一人称
```

### 18.5.3 相机配置

| 设置 | 说明 |
|-----|------|
| Focal Point (焦点) | 视图中心点 |
| Distance (距离) | 相机距离 |
| Pitch (俯仰) | 垂直角度 |
| Yaw (偏航) | 水平角度 |

## 18.6 保存和加载配置

### 18.6.1 保存配置

```
File → Save Config As → my_config.rviz
```

### 18.6.2 加载配置

```bash
# 命令行加载
rviz2 -d my_config.rviz

# 或在 RViz2 中
File → Open Config → my_config.rviz
```

### 18.6.3 配置文件位置

```bash
# 默认配置位置
~/.rviz2/default.rviz

# 包内配置
$(find my_package)/rviz/config.rviz
```

## 18.7 Tools 面板

### 18.7.1 可用工具

| 工具 | 快捷键 | 功能 |
|-----|-------|------|
| Move Camera | M | 移动相机 |
| Select | S | 选择对象 |
| Focus | F | 聚焦对象 |
| Measure | - | 测量距离 |
| Publish Point | P | 发布点 |
| Initial Pose | I | 设置初始位姿 |
| 2D Pose Estimate | G | 2D 位姿估计 |
| 2D Goal Pose | - | 2D 目标位姿 |
| Navigation 2D Goal | N | 导航目标 |

### 18.7.2 发布点

```
1. 选择 Tool: Publish Point
2. 在 3D 视图中点击
3. 点发布到 /clicked_point 话题
```

```bash
# 订阅点击的点
ros2 topic echo /clicked_point
```

## 18.8 标记 (Markers)

### 18.8.1 可视化标记

```cpp
#include "visualization_msgs/msg/marker.hpp"

void publish_marker() {
    auto marker = visualization_msgs::msg::Marker();

    marker.header.frame_id = "base_link";
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "my_namespace";
    marker.id = 0;

    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // 位置
    marker.pose.position.x = 1.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;

    // 方向
    marker.pose.orientation.w = 1.0;

    // 尺寸
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;

    // 颜色
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    marker_pub_->publish(marker);
}
```

### 18.8.2 标记类型

| 类型 | 说明 |
|-----|------|
| `ARROW` | 箭头 |
| `CUBE` | 立方体 |
| `SPHERE` | 球体 |
| `CYLINDER` | 圆柱 |
| `LINE_STRIP` | 线条 |
| `POINTS` | 点集 |
| `TEXT_VIEW_FACING` | 文本 |
| `MESH_RESOURCE` | 网格模型 |

## 18.9 面板配置

### 18.9.1 添加面板

```
Panels → Add New Panel:

├── Time                     # 时间控制
├── Selection                # 选择信息
├── Views                    # 视图选择
├── Tool Properties          # 工具属性
├── Camera                   # 相机配置
└── Favorites                # 收藏夹
```

### 18.9.2 常用面板

**Time 面板:**
- 控制仿真时间
- 回放控制

**Selection 面板:**
- 显示选中对象信息
- TF 信息

## 18.10 高级功能

### 18.10.1 多视图

```
Views → New View
可以创建多个 3D 视图，不同视角同时显示
```

### 18.10.2 屏幕截图

```
File → Screenshot
或快捷键: Ctrl+S
```

### 18.10.3 录制视频

```
Tools → Record Video
```

## 18.11 最佳实践

| 建议 | 说明 |
|-----|------|
| **保存配置** | 为每个应用保存专用配置 |
| **固定框架** | 设置 Fixed Frame 为合适坐标系 |
| **组织显示** | 禁用不需要的显示项 |
| **使用插件** | 利用 RViz 插件扩展功能 |

## 18.12 下一步

1. **[19 Rqt Tools](./19-rqt-tools.md)** - 学习 Rqt 工具
2. **[20 Launch](./20-launch.md)** - 学习 Launch 配置

