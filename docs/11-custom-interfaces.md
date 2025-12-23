# 11 自定义接口消息 (Custom Interfaces)

## 11.1 接口消息概述

### 11.1.1 接口类型

ROS 2 支持三种自定义接口类型：

| 类型 | 文件扩展名 | 用途 |
|-----|----------|------|
| **消息 (Message)** | `.msg` | 话题通信的数据结构 |
| **服务 (Service)** | `.srv` | 服务通信的请求/响应 |
| **动作 (Action)** | `.action` | 动作通信的目标/结果/反馈 |

### 11.1.2 为什么需要自定义接口

- ROS 2 标准接口不能满足特定需求
- 需要传递特定的数据结构
- 提高代码可读性和可维护性

## 11.2 创建接口包

### 11.2.1 创建专用接口包

```bash
# 创建接口包
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake my_interfaces

# 或创建 Python 接口包
ros2 pkg create --build-type ament_python my_interfaces_py
```

### 11.2.2 包结构

```
my_interfaces/
├── CMakeLists.txt
├── package.xml
├── msg/
│   ├── Num.msg              # 自定义消息
│   └── Sphere.msg
├── srv/
│   └── AddThreeInts.srv     # 自定义服务
└── action/
    └── Fibonacci.action     # 自定义动作
```

### 11.2.3 package.xml 配置

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_interfaces</name>
  <version>1.0.0</version>
  <description>Custom ROS 2 interfaces</description>
  <maintainer email="user@example.com">User</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <!-- 消息生成依赖 -->
  <build_depend>rosidl_default_generators</build_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>

  <!-- 动作接口依赖 -->
  <build_depend>action_msgs</build_depend>

  <member_of_group>rosidl_interface_packages</member_of_group>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

## 11.3 自定义消息 (Messages)

### 11.3.1 创建 .msg 文件

**msg/Num.msg:**
```
int64 num
bool flag
string name
float64[] values
```

**msg/Sphere.msg:**
```
geometry_msgs/Point center
float64 radius
```

**msg/PointWithCovariance.msg:**
```
geometry_msgs/Point point
float64[9] covariance
```

### 11.3.2 消息类型

| 基本类型 | 说明 |
|---------|------|
| `bool` | 布尔值 |
| `int8`, `uint8` | 8位整数 |
| `int16`, `uint16` | 16位整数 |
| `int32`, `uint32` | 32位整数 |
| `int64`, `uint64` | 64位整数 |
| `float32`, `float64` | 浮点数 |
| `string` | 字符串 |
| `duration` | 时间段 |
| `time` | 时间戳 |

**数组类型:**
```
int32[] numbers          # 动态数组
int32[10] fixed_array    # 固定大小数组
string[] names           # 字符串数组
```

## 11.4 自定义服务 (Services)

### 11.4.1 创建 .srv 文件

**srv/AddThreeInts.srv:**
```
# 请求部分
int64 a
int64 b
int64 c
# 响应部分
int64 sum
bool success
string message
```

**srv/SetLed.srv:**
```
uint64 led_id
bool state
bool success
```

### 11.4.2 复杂服务示例

**srv/GetMap.srv:**
```
# 请求
# 响应
nav_msgs/MapMetaData map_info
int8[] data
# 反馈（如果有）
```

## 11.5 自定义动作 (Actions)

### 11.5.1 创建 .action 文件

**action/Fibonacci.action:**
```
# Goal - 目标定义
int32 order
# Result - 结果定义
int32[] sequence
bool success
# Feedback - 反馈定义
int32[] partial_sequence
float32 progress
```

**action/MoveTo.action:**
```
# Goal
geometry_msgs/PoseStamped target_pose
float64 tolerance
# Result
bool success
geometry_msgs/PoseStamped final_pose
# Feedback
geometry_msgs/PoseStamped current_pose
float32 distance_remaining
```

### 11.5.2 动作结构说明

```action
# 动作文件格式
# 分隔符 --- 分隔三个部分

# --- 之前: Goal (发送给服务端的目标)
# --- 和 --- 之间: Result (任务完成后返回的结果)
# --- 之后: Feedback (任务执行过程中的状态更新)
```

## 11.6 CMakeLists.txt 配置

### 11.6.1 完整配置

```cmake
cmake_minimum_required(VERSION 3.8)
project(my_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)

# 生成消息
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Num.msg"
  "msg/Sphere.msg"
  "srv/AddThreeInts.srv"
  "action/Fibonacci.action"
  DEPENDENCIES std_msgs geometry_msgs action_msgs
)

# 安装
ament_package()
```

### 11.6.2 Python 接口包配置

对于 `ament_python` 类型的包，在 `setup.py` 中：

```python
from setuptools import setup

package_name = 'my_interfaces_py'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/msg', ['msg/Num.msg']),
        ('share/' + package_name + '/srv', ['srv/AddThreeInts.srv']),
        ('share/' + package_name + '/action', ['action/Fibonacci.action']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@example.com',
    description='Custom ROS 2 interfaces',
    license='Apache-2.0',
    tests_require=['pytest'],
)
```

## 11.7 编译和使用

### 11.7.1 编译接口包

```bash
cd ~/ros2_ws
colcon build --packages-select my_interfaces

# Source 环境
source install/setup.bash
```

### 11.7.2 验证接口

```bash
# 查看自定义消息
ros2 interface show my_interfaces/msg/Num

# 查看自定义服务
ros2 interface show my_interfaces/srv/AddThreeInts

# 查看自定义动作
ros2 interface show my_interfaces/action/Fibonacci
```

### 11.7.3 使用自定义接口

**C++ 使用示例:**

```cpp
#include "my_interfaces/msg/num.hpp"
#include "my_interfaces/srv/add_three_ints.hpp"

void create_message() {
    auto msg = my_interfaces::msg::Num();
    msg.num = 42;
    msg.flag = true;
    msg.name = "example";
    msg.values = {1.0, 2.0, 3.0};
}
```

**Python 使用示例:**

```python
from my_interfaces.msg import Num
from my_interfaces.srv import AddThreeInts

def create_message():
    msg = Num()
    msg.num = 42
    msg.flag = True
    msg.name = 'example'
    msg.values = [1.0, 2.0, 3.0]
```

## 11.8 接口依赖

### 11.8.1 消息中的消息依赖

```msg
# 在自定义消息中使用其他消息类型
std_msgs/Header header
geometry_msgs/Pose pose
sensor_msgs/JointState joint_state
```

### 11.8.2 CMakeLists.txt 中的依赖声明

```cmake
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MultiType.msg"
  DEPENDENCIES
    std_msgs
    geometry_msgs
    sensor_msgs
)
```

## 11.9 常见问题

### 11.9.1 编译错误

**问题:** 找不到自定义接口

**解决:**
```bash
# 1. 确保接口包已编译
colcon build --packages-select my_interfaces

# 2. Source 环境
source install/setup.bash

# 3. 清理并重新编译
rm -rf build install log
colcon build
```

### 11.9.2 依赖问题

**问题:** 消息依赖的其他包未找到

**解决:** 在 CMakeLists.txt 中声明所有依赖

```cmake
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MyMessage.msg"
  DEPENDENCIES std_msgs geometry_msgs
)
```

## 11.10 最佳实践

| 原则 | 说明 |
|-----|------|
| **语义化命名** | 接口名应清晰描述其用途 |
| **模块化** | 相关接口放在同一包中 |
| **版本控制** | 接口变更时更新版本号 |
| **文档化** | 为接口添加注释说明 |

## 11.11 下一步

1. **[12 Parameters](./12-parameters.md)** - 学习参数服务
2. **[20 Launch](./20-launch.md)** - 学习启动配置

