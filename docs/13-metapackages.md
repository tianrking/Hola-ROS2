# 13 元功能包 (Metapackages)

## 13.1 元功能包概述

### 13.1.1 什么是元功能包

**元功能包 (Metapackage)** 是一个特殊的 ROS 2 功能包，本身不包含代码，而是用于将多个相关功能包组织在一起。它类似于一个"虚包"，作为其他包的逻辑分组。

```
元功能包概念图：

┌─────────────────────────────────────────────────┐
│          navigation2 (元功能包)                   │
│                                                 │
│   ┌──────────┐  ┌──────────┐  ┌──────────┐    │
│   │nav2_costmap│ │nav2_planner│ │nav2_controller│   │
│   └──────────┘  └──────────┘  └──────────┘    │
│                                                 │
│   ┌──────────┐  ┌──────────┐  ┌──────────┐    │
│   │nav2_behaviors│ │ nav2_core │ │ nav2_bt_navigator│ │
│   └──────────┘  └──────────┘  └──────────┘    │
└─────────────────────────────────────────────────┘
```

### 13.1.2 元功能包的用途

| 用途 | 说明 |
|-----|------|
| **组织** | 将相关功能包分组 |
| **简化安装** | 一次安装多个包 |
| **依赖管理** | 统一管理依赖关系 |
| **文档化** | 清晰的项目结构 |

## 13.2 创建元功能包

### 13.2.1 创建命令

```bash
# 创建元功能包
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake --dependencies <package1> <package2> <package3> my_metapackage
```

### 13.2.2 元功能包结构

```
my_metapackage/
├── CMakeLists.txt
├── package.xml
└── METAPackage.xml    # 可选的元数据文件
```

## 13.3 package.xml 配置

### 13.3.1 基本配置

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_robot_metapackage</name>
  <version>1.0.0</version>
  <description>Metapackage for my robot components</description>
  <maintainer email="user@example.com">User Name</maintainer>
  <license>Apache-2.0</license>

  <!-- 元功能包标识 -->
  <export>
    <metapackage/>
  </export>

  <!-- 依赖的成员包 -->
  <exec_depend>my_robot_driver</exec_depend>
  <exec_depend>my_robot_controller</exec_depend>
  <exec_depend>my_robot_navigation</exec_depend>
  <exec_depend>my_robot_viz</exec_depend>

  <!-- 构建工具 -->
  <buildtool_depend>ament_cmake</buildtool_depend>
</package>
```

### 13.3.2 完整示例

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_robot_suite</name>
  <version>2.0.0</version>
  <description>
    Complete suite of packages for my robot including drivers,
    controllers, navigation, and visualization tools.
  </description>
  <maintainer email="developer@example.com">Developer</maintainer>
  <maintainer email="contributor@example.com">Contributor</maintainer>
  <license>Apache-2.0</license>

  <url type="website">https://github.com/username/my_robot_suite</url>
  <url type="bugtracker">https://github.com/username/my_robot_suite/issues</url>
  <url type="repository">https://github.com/username/my_robot_suite</url>

  <author email="author@example.com">Author Name</author>

  <export>
    <metapackage/>
  </export>

  <!-- 核心包 -->
  <exec_depend>my_robot_msgs</exec_depend>
  <exec_depend>my_robot_common</exec_depend>

  <!-- 硬件驱动 -->
  <exec_depend>my_robot_camera</exec_depend>
  <exec_depend>my_robot_lidar</exec_depend>
  <exec_depend>my_robot_motor</exec_depend>

  <!-- 控制与导航 -->
  <exec_depend>my_robot_controller</exec_depend>
  <exec_depend>my_robot_navigation</exec_depend>

  <!-- 工具与可视化 -->
  <exec_depend>my_robot_viz</exec_depend>
  <exec_depend>my_robot_tools</exec_depend>

  <buildtool_depend>ament_cmake</buildtool_depend>
</package>
```

## 13.4 CMakeLists.txt 配置

### 13.4.1 最小配置

```cmake
cmake_minimum_required(VERSION 3.8)
project(my_robot_metapackage)

find_package(ament_cmake REQUIRED)

# 元功能包不需要任何编译步骤
ament_package()
```

### 13.4.2 添加元数据

```cmake
cmake_minimum_required(VERSION 3.8)
project(my_robot_metapackage)

find_package(ament_cmake REQUIRED)

# 设置包描述
set(METAPACKAGE_PROJECT_DESC
  "Metapackage containing packages for my robot")

# 配置
ament_package()
```

## 13.5 元功能包类型

### 13.5.1 按功能组织

```
my_robot_project/
├── my_robot_msgs/          # 消息定义
├── my_robot_driver/        # 硬件驱动
├── my_robot_controller/    # 控制器
├── my_robot_navigation/    # 导航
├── my_robot_viz/           # 可视化
└── my_robot_bringup/       # 启动配置
```

### 13.5.2 按平台组织

```
my_robot_suite/
├── my_robot_core/          # 核心功能
├── my_robot_simulation/    # 仿真相关
├── my_robot_hardware/      # 硬件接口
└── my_robot_apps/          # 应用程序
```

## 13.6 编译和安装

### 13.6.1 编译元功能包

```bash
# 编译元功能包（实际编译所有依赖的包）
colcon build --packages-select my_robot_metapackage

# 编译元功能包及其依赖
colcon build --packages-up-to my_robot_metapackage
```

### 13.6.2 安装元功能包

```bash
# 通过 apt 安装元功能包（将安装所有依赖）
sudo apt install ros-humble-my-robot-metapackage

# 从源码编译后，可以通过元功能包安装所有组件
source install/setup.bash
```

## 13.7 常见元功能包示例

### 13.7.1 navigation2

```bash
# 安装完整的导航功能包
sudo apt install ros-humble-navigation2

# 这将安装：
# - nav2_bringup
# - nav2_controller
# - nav2_planner
# - nav2_recoveries
# - nav2_bt_navigator
# - nav2_lifecycle_manager
# ... 等
```

### 13.7.2 desktop

```bash
# 安装 ROS 2 桌面环境
sudo apt install ros-humble-desktop

# 包含：
# - RViz2
# - tf2_tools
# - ros2_helpers
# 等
```

## 13.8 最佳实践

### 13.8.1 命名规范

| 规范 | 示例 | 说明 |
|-----|------|------|
| 以 `_metapackage` 结尾 | `my_robot_metapackage` | 清楚标识 |
| 使用功能组名称 | `navigation2` | 描述功能域 |
| 使用项目名称 | `my_robot_suite` | 组织项目包 |

### 13.8.2 版本管理

```xml
<!-- 元功能包版本应与项目版本同步 -->
<version>2.0.0</version>

<!-- 所有成员包应兼容该版本 -->
<exec_depend>my_package1 version_gte=1.0.0</exec_depend>
<exec_depend>my_package2 version_gte=1.0.0</exec_depend>
```

### 13.8.3 依赖策略

| 依赖类型 | 何时使用 |
|---------|---------|
| `exec_depend` | 元功能包的主要依赖 |
| `depend` | 简化依赖声明 |
| `test_depend` | 测试时需要的包 |

## 13.9 元功能包 vs 普通包

| 特性 | 元功能包 | 普通包 |
|-----|---------|-------|
| **包含代码** | 否 | 是 |
| **编译产物** | 无 | 库/可执行文件 |
| **主要用途** | 组织其他包 | 实现功能 |
| **安装效果** | 安装依赖的所有包 | 安装该包 |
| **build_type** | `ament_cmake` (任意) | `ament_cmake`/`ament_python` |

## 13.10 常见问题

### 13.10.1 元功能包编译错误

**问题:** 元功能包找不到依赖的包

**解决:**
```bash
# 确保所有依赖的包在源码空间中
ls ~/ros2_ws/src/

# 或通过 apt 安装
sudo apt install ros-humble-<package-name>
```

### 13.10.2 循环依赖

**问题:** 元功能包与成员包之间形成循环依赖

**解决:** 避免成员包依赖元功能包，元功能包应该是"顶层"包

## 13.11 下一步

1. **[14 Distributed Communication](./14-distributed.md)** - 学习分布式通信
2. **[15 DDS](./15-dds.md)** - 学习 DDS 中间件

