# 05 功能包 (Packages)

## 5.1 功能包概述

### 5.1.1 什么是功能包

**功能包 (Package)** 是 ROS 2 中组织代码的基本单元。它包含用于实现特定功能的源代码、配置文件、数据文件、构建脚本和文档。功能包可以被其他包依赖、使用和共享。

```
功能包概念图：

┌────────────────────────────────────────────────────┐
│                   功能包 (Package)                   │
│                                                    │
│  ┌────────────────────────────────────────────┐   │
│  │              package.xml                    │   │  元信息
│  │         (包描述文件、依赖声明)               │   │
│  └────────────────────────────────────────────┘   │
│                                                    │
│  ┌────────────┐  ┌────────────┐  ┌────────────┐  │
│  │CMakeLists  │  │  setup.py  │  │   源代码    │  │
│  │   .txt     │  │            │  │            │  │
│  │(C++ 构建配置)│ │(Python配置)│  │            │  │
│  └────────────┘  └────────────┘  └────────────┘  │
│                                                    │
│  ┌────────────┐  ┌────────────┐  ┌────────────┐  │
│  │   配置文件 │  │   启动文件  │  │    资源     │  │
│  │   .yaml   │  │  .launch   │  │   (urdf/    │  │
│  │            │  │            │  │  meshes)   │  │
│  └────────────┘  └────────────┘  └────────────┘  │
│                                                    │
└────────────────────────────────────────────────────┘
```

### 5.1.2 功能包的命名规则

- 只能包含小写字母、数字和下划线
- 必须以字母开头
- 推荐使用描述性的名称
- 避免使用 ROS 2 保留名称

**有效名称示例：**

| 名称 | 状态 | 说明 |
|-----|------|------|
| `my_robot_controller` | 有效 | 推荐格式 |
| `camera_driver` | 有效 | 简洁描述性 |
| `2d_navigation` | **无效** | 不能以数字开头 |
| `my-package` | **无效** | 不能使用连字符 |
| `MyPackage` | **无效** | 不能使用大写字母 |

### 5.1.3 功能包类型

| 类型 | 构建系统 | 主要语言 | 用途 |
|-----|---------|---------|------|
| **ament_cmake** | CMake | C++ | C++ 节点、混合项目 |
| **ament_python** | setuptools | Python | 纯 Python 项目 |
| **ament_cmake_python** | CMake + Python | 混合 | C++ 和 Python 混合项目 |

## 5.2 创建功能包

### 5.2.1 创建 C++ 功能包

使用 `ros2 pkg create` 命令创建 C++ 功能包：

```bash
cd ~/ros2_ws/src

# 创建基本 C++ 包
ros2 pkg create --build-type ament_cmake my_cpp_pkg

# 创建带有依赖的 C++ 包
ros2 pkg create --build-type ament_cmake \
    --dependencies rclcpp std_msgs \
    my_robot_controller

# 创建完整的 C++ 包结构
ros2 pkg create --build-type ament_cmake \
    --dependencies rclcpp std_msgs geometry_msgs \
    --node-name my_node \
    --library-name my_library \
    my_cpp_pkg
```

**常用参数：**

| 参数 | 说明 |
|-----|------|
| `--build-type` | 构建类型 (ament_cmake/ament_python) |
| `--dependencies` | 包依赖列表 |
| `--node-name` | 创建示例节点 |
| `--library-name` | 创建库目标 |
| `--description` | 包描述信息 |

### 5.2.2 创建 Python 功能包

```bash
cd ~/ros2_ws/src

# 创建基本 Python 包
ros2 pkg create --build-type ament_python my_py_pkg

# 创建带有依赖的 Python 包
ros2 pkg create --build-type ament_python \
    --dependencies rclpy std_msgs \
    my_python_package

# 创建包含节点的 Python 包
ros2 pkg create --build-type ament_python \
    --dependencies rclpy \
    --node-name my_node \
    my_py_pkg
```

### 5.2.3 功能包目录结构

**C++ 包结构：**

```
my_cpp_pkg/
├── CMakeLists.txt          # CMake 构建配置
├── package.xml             # 包元信息
├── src/                    # C++ 源代码
│   └── my_node.cpp
├── include/               # 头文件
│   └── my_cpp_pkg/
│       └── my_header.hpp
├── launch/                # Launch 文件
│   └── my_launch.py
├── config/                # 配置文件
│   └── params.yaml
├── resource/              # 资源文件
└── test/                  # 测试代码
```

**Python 包结构：**

```
my_py_pkg/
├── setup.py               # Python 构建配置
├── setup.cfg              # Python 配置
├── package.xml            # 包元信息
├── my_py_pkg/             # Python 包目录
│   ├── __init__.py
│   └── my_node.py
├── launch/                # Launch 文件
│   └── my_launch.py
├── config/                # 配置文件
│   └── params.yaml
├── resource/              # 资源文件
└── test/                  # 测试代码
    └── test_copyright.py
    └── test_flake8.py
    └── test_pep257.py
```

## 5.3 package.xml 配置

### 5.3.1 package.xml 基本结构

`package.xml` 是功能包的元数据文件，定义了包的基本信息和依赖关系：

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <!-- 基本信息 -->
  <name>my_package</name>
  <version>1.0.0</version>
  <description>My ROS 2 package description</description>
  <maintainer email="user@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <!-- 构建工具 -->
  <buildtool_depend>ament_cmake</buildtool_depend>

  <!-- 依赖项 -->
  <depend>rclcpp</depend>
  <depend>std_msgs</depend>

  <!-- 测试依赖 -->
  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <!-- 导出信息 -->
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### 5.3.2 依赖类型详解

| 依赖类型 | 说明 | 示例 |
|---------|------|------|
| `<depend>` | 编译、运行、测试都依赖 | `<depend>rclcpp</depend>` |
| `<build_depend>` | 仅编译时依赖 | `<build_depend>message_generation</build_depend>` |
| `<build_export_depend>` | 编译和导出依赖 | `<build_export_depend>some_msg_pkg</build_export_depend>` |
| `<exec_depend>` | 仅运行时依赖 | `<exec_depend>python3-numpy</exec_depend>` |
| `<test_depend>` | 仅测试时依赖 | `<test_depend>ament_cmake_gtest</test_depend>` |
| `<buildtool_depend>` | 构建工具依赖 | `<buildtool_depend>ament_cmake</buildtool_depend>` |

### 5.3.3 常用依赖项

**核心依赖：**

```xml
<!-- C++ 支持 -->
<depend>rclcpp</depend>

<!-- Python 支持 -->
<depend>rclpy</depend>

<!-- 标准消息 -->
<depend>std_msgs</depend>

<!-- 几何消息 -->
<depend>geometry_msgs</depend>

<!-- 传感器消息 -->
<depend>sensor_msgs</depend>

<!-- 导航消息 -->
<depend>nav_msgs</depend>

<!-- TF2 变换 -->
<depend>tf2</depend>
<depend>tf2_ros</depend>
<depend>tf2_geometry_msgs</depend>
```

### 5.3.4 完整 package.xml 示例

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_robot_package</name>
  <version>1.0.0</version>
  <description>My robot control package with advanced features</description>
  <maintainer email="developer@example.com">Developer Name</maintainer>
  <license>Apache-2.0</license>

  <url type="website">https://github.com/username/my_robot_package</url>
  <url type="bugtracker">https://github.com/username/my_robot_package/issues</url>
  <url type="repository">https://github.com/username/my_robot_package</url>

  <author email="contributor@example.com">Contributor Name</author>

  <!-- Build tool -->
  <buildtool_depend>ament_cmake</buildtool_depend>

  <!-- Core dependencies -->
  <depend>rclcpp</depend>
  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>

  <!-- Additional dependencies -->
  <exec_depend>tf2_ros</exec_depend>
  <exec_depend>tf2_geometry_msgs</exec_depend>

  <!-- Test dependencies -->
  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>
  <test_depend>ament_cmake_gtest</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

## 5.4 CMakeLists.txt 配置 (C++)

### 5.4.1 基本结构

```cmake
cmake_minimum_required(VERSION 3.8)
project(my_cpp_pkg)

# 默认为 C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# 包含目录
include_directories(include)

# 可执行文件
add_executable(my_node src/my_node.cpp)

# 依赖项
ament_target_dependencies(my_node
  rclcpp
  std_msgs
)

# 安装目标
install(TARGETS
  my_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装 Python 模块
ament_package()
```

### 5.4.2 添加可执行文件

```cmake
# 创建可执行文件
add_executable(talker src/talker.cpp)
add_executable(listener src/listener.cpp)

# 链接依赖
ament_target_dependencies(talker
  rclcpp
  std_msgs
)

ament_target_dependencies(listener
  rclcpp
  std_msgs
)

# 安装可执行文件
install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME}
)
```

### 5.4.3 添加库

```cmake
# 创建库
add_library(my_library
  src/my_library.cpp
)

# 库可见性
target_include_directories(my_library PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(my_library
  rclcpp
)

# 安装库
install(TARGETS my_library
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include/
)
```

### 5.4.4 安装额外文件

```cmake
# 安装 Launch 文件
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch/
)

# 安装配置文件
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config/
)

# 安装 URDF/Mesh 文件
install(DIRECTORY urdf meshes
  DESTINATION share/${PROJECT_NAME}/
)
```

### 5.4.5 测试配置

```cmake
# 启用测试
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  # GTest 测试
  ament_add_gtest(test_my_library test/test_my_library.cpp)
  target_link_libraries(test_my_library my_library)

  # Lint 检查
  ament_lint_auto_find_test_dependencies()
endif()
```

## 5.5 setup.py 配置 (Python)

### 5.5.1 基本结构

```python
from setuptools import setup

package_name = 'my_py_pkg'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/my_launch.py']),
        ('share/' + package_name + '/config', ['config/params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='user@example.com',
    description='My ROS 2 Python package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_node = my_py_pkg.my_node:main',
        ],
    },
)
```

### 5.5.2 setup.cfg 配置

```ini
[develop]
script-dir=$base/lib/my_py_pkg
[install]
install-scripts=$base/lib/my_py_pkg
```

### 5.5.3 添加可执行节点

在 `setup.py` 中使用 `entry_points`：

```python
entry_points={
    'console_scripts': [
        # 节点名 = 模块路径:函数名
        'talker = my_py_pkg.talker:main',
        'listener = my_py_pkg.listener:main',
        'camera_node = my_py_pkg.camera:main',
    ],
}
```

## 5.6 包的编译与安装

### 5.6.1 编译单个包

```bash
cd ~/ros2_ws

# 编译指定包
colcon build --packages-select my_cpp_pkg

# 编译并显示详细输出
colcon build --packages-select my_cpp_pkg --event-handlers console_direct+
```

### 5.6.2 编译多个包

```bash
# 编译多个指定包
colcon build --packages-select pkg1 pkg2 pkg3

# 编译除了某些包之外的所有包
colcon build --packages-skip pkg_to_skip
```

### 5.6.3 常用编译选项

```bash
# 使用符号链接安装（开发时推荐）
colcon build --symlink-install

# Debug 模式编译
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug

# Release 模式编译
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# 生成编译命令数据库
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

## 5.7 依赖管理

### 5.7.1 查看包依赖

```bash
# 查看包的直接依赖
ros2 pkg dependencies my_package

# 查看依赖的所有包（递归）
ros2 pkg dependencies my_package --all

# 查看哪些包依赖于此包
ros2 pkg dependents my_package

# 查看包的元信息
ros2 pkg xml my_package
```

### 5.7.2 使用 rosdep 管理系统依赖

```bash
# 安装包的所有系统依赖
cd ~/ros2_ws
rosdep install -r --from-paths src --ignore-src -y

# 仅安装特定包的依赖
rosdep install --from-paths src/my_package --ignore-src -y
```

### 5.7.3 依赖关系图

```
依赖关系示例：

my_robot_app (顶层应用)
├── 依赖
│   ├── my_robot_controller (控制器)
│   │   ├── 依赖 rclcpp
│   │   ├── 依赖 std_msgs
│   │   └── 依赖 geometry_msgs
│   ├── my_sensor_driver (传感器驱动)
│   │   └── 依赖 sensor_msgs
│   └── navigation2 (导航)
│       └── 依赖 tf2
└── 构建依赖
    └── ament_cmake
```

## 5.8 包的发布与共享

### 5.8.1 准备发布

确保包包含以下文件：

```
my_package/
├── README.md              # 项目说明
├── LICENSE                # 许可证文件
├── package.xml            # 包元信息
├── CMakeLists.txt / setup.py
├── src/                   # 源代码
├── include/               # 头文件（C++）
├── launch/                # Launch 文件
├── config/                # 配置文件
└── test/                  # 测试代码
```

### 5.8.2 创建 README.md

```markdown
# My ROS 2 Package

## Description

Brief description of what this package does.

## Features

- Feature 1
- Feature 2
- Feature 3

## Dependencies

- ROS 2 Humble
- rclcpp
- std_msgs
- geometry_msgs

## Building

```bash
cd ~/ros2_ws
colcon build --packages-select my_package
```

## Usage

```bash
source install/setup.bash
ros2 run my_package my_node
```

## License

Apache-2.0
```

### 5.8.3 发布到 GitHub

```bash
# 初始化 git 仓库
cd ~/ros2_ws/src/my_package
git init

# 添加 .gitignore
cat > .gitignore << 'EOF'
build/
install/
log/
*.pyc
__pycache__/
.vscode/
.idea/
EOF

# 提交代码
git add .
git commit -m "Initial commit"

# 推送到 GitHub
git remote add origin https://github.com/username/my_package.git
git push -u origin main
```

## 5.9 最佳实践

### 5.9.1 包设计原则

| 原则 | 说明 | 示例 |
|-----|------|------|
| **单一职责** | 每个包专注于一个功能 | 传感器驱动和控制逻辑分开 |
| **低耦合** | 最小化包间依赖 | 使用消息接口通信 |
| **高内聚** | 相关功能放在同一包 | TF 转换工具放一起 |
| **可重用** | 设计为可被其他包使用 | 通用工具库 |

### 5.9.2 命名约定

**包名约定：**

```
好的命名：
- camera_driver        (清晰描述功能)
- robot_controller    (描述功能域)
- path_planning       (描述算法)

不好的命名：
- my_package          (不够描述性)
- stuff              (完全不描述)
- pkg1               (无意义)
```

**节点命名：**

```
推荐格式: <function>_<type>_node

camera_driver_node
lidar_processor_node
path_planner_node
robot_state_publisher
```

### 5.9.3 目录组织最佳实践

```
大型项目组织示例：

my_robot_project/
├── my_robot_msgs/           # 自定义消息
├── my_robot_driver/         # 硬件驱动
├── my_robot_controller/     # 控制器
├── my_robot_navigation/     # 导航配置
├── my_robot_viz/            # 可视化配置
└── my_robot_bringup/        # 启动配置
```

## 5.10 常见问题

### 5.10.1 找不到包

**问题：** `ros2 pkg list` 看不到新创建的包

**解决方法：**

```bash
# 1. 检查 package.xml 语法
xmllint --noout package.xml

# 2. 确保在工作空间的 src 目录中
ls ~/ros2_ws/src/

# 3. 重新编译
cd ~/ros2_ws
colcon build --packages-select <package_name>

# 4. Source 环境
source install/setup.bash
```

### 5.10.2 依赖问题

**问题：** 编译时提示找不到依赖

**解决方法：**

```bash
# 1. 确保依赖已安装
ros2 pkg list | grep <dependency_name>

# 2. 如果是系统依赖，使用 rosdep
rosdep install --from-paths src --ignore-src -y

# 3. 检查 package.xml 中的依赖声明是否正确
cat package.xml | grep <dependency_name>
```

### 5.10.3 CMake 配置问题

**问题：** CMake 找不到包

**解决方法：**

```bash
# 1. 确保 source 了 ROS 2 环境
source /opt/ros/humble/setup.bash

# 2. 检查 CMakeLists.txt 中的 find_package
find_package(rclcpp REQUIRED)

# 3. 清理并重新编译
rm -rf build install
colcon build
```

## 5.11 下一步

学习功能包后，您可以：

1. **[06 Nodes](./06-nodes.md)** - 编写节点代码
2. **[07 Topics](./07-topics.md)** - 学习话题通信

---
**✅ 05 功能包 - 已完成**
