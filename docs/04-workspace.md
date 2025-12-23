# 04 工作区 (Workspace)

## 4.1 工作区概述

### 4.1.1 什么是工作区

**工作区 (Workspace)** 是 ROS 2 中用于组织和管理功能包的目录结构。它是一个包含源码、编译产物和安装文件的根目录，是进行 ROS 2 开发的基础环境。

```
工作空间概念图：

┌────────────────────────────────────────────────────┐
│                   ROS 2 工作空间                    │
│                                                    │
│  ┌─────────┐  ┌─────────┐  ┌─────────┐           │
│  │ Package │  │ Package │  │ Package │           │
│  │   A     │  │   B     │  │   C     │           │
│  └─────────┘  └─────────┘  └─────────┘           │
│       │            │            │                 │
│       └────────────┴────────────┘                 │
│                    │                              │
│              Colcon 编译系统                       │
│                    │                              │
│       ┌────────────┴────────────┐                 │
│       ▼                         ▼                 │
│  ┌─────────┐              ┌─────────┐             │
│  │  build  │              │ install │             │
│  └─────────┘              └─────────┘             │
└────────────────────────────────────────────────────┘
```

### 4.1.2 工作区的目录结构

一个标准的 ROS 2 工作空间包含以下目录：

```
~/ros2_ws/
├── build/           # 编译中间文件目录
│   ├── package_1/
│   ├── package_2/
│   └── ...
├── install/         # 安装文件目录（可执行文件、库、脚本）
│   ├── package_1/
│   ├── package_2/
│   ├── setup.bash   # 环境设置脚本（重要）
│   ├── setup.zsh
│   └── local_setup.bash
├── log/             # 编译和测试日志
│   ├── build_/
│   ├── test_/
│   └── ...
└── src/             # 源码目录（功能包放这里）
    ├── package_1/
    ├── package_2/
    └── ...
```

**目录说明：**

| 目录 | 用途 | 是否版本控制 |
|-----|------|------------|
| `src/` | 存放功能包源码 | 是 |
| `build/` | 编译中间产物 | 否 |
| `install/` | 最终安装文件 | 否 |
| `log/` | 编译和测试日志 | 否 |

### 4.1.3 工作区类型

| 类型 | 路径示例 | 用途 |
|-----|---------|------|
| **系统工作区** | `/opt/ros/humble/` | 安装的 ROS 2 软件 |
| **用户工作区** | `~/ros2_ws/` | 个人开发工作空间 |
| **覆盖工作区** | `~/overlay_ws/` | 扩展或覆盖已有包 |

## 4.2 创建工作空间

### 4.2.1 创建基础工作空间

```bash
# 创建工作空间目录
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# 查看目录结构
tree -L 2
```

**预期输出：**
```
~/ros2_ws/
└── src/
```

### 4.2.2 初始化工作空间

工作空间无需显式初始化。当您在 `src/` 目录中添加包并编译时，colcon 会自动识别和构建工作空间。

```bash
# 添加一个示例包到 src/ 目录
cd ~/ros2_ws/src
# 这里可以克隆或创建功能包

# 返回工作空间根目录
cd ~/ros2_ws
```

## 4.3 Colcon 编译系统

### 4.3.1 Colcon 简介

**Colcon** (Command Line COmpiler for CONstituents) 是 ROS 2 的推荐构建工具，替代了 ROS 1 的 Catkin。

**Colcon 特点：**

| 特性 | 描述 |
|-----|------|
| **并行构建** | 支持多包并行编译 |
| **增量编译** | 只重新编译修改过的包 |
| **扩展性** | 通过插件扩展功能 |
| **无中心构建** | 每个包独立构建，减少依赖 |

### 4.3.2 安装 Colcon

```bash
# 安装 colcon 和常用扩展
sudo apt install -y python3-colcon-common-extensions

# 验证安装
colcon --help
```

### 4.3.3 Colcon 基本用法

**编译整个工作空间：**

```bash
cd ~/ros2_ws
colcon build
```

**编译单个包：**

```bash
colcon build --packages-select <package_name>
```

**编译多个包：**

```bash
colcon build --packages-select <pkg1> <pkg2> <pkg3>
```

**编译时跳过某些包：**

```bash
colcon build --packages-skip <package_name>
```

### 4.3.4 常用编译选项

| 选项 | 说明 | 示例 |
|-----|------|------|
| `--symlink-install` | 使用符号链接，便于开发调试 | `colcon build --symlink-install` |
| `--cmake-args` | 传递 CMake 参数 | `--cmake-args -DCMAKE_BUILD_TYPE=Debug` |
| `--parallel-workers` | 设置并行工作数 | `--parallel-workers 4` |
| `--event-handlers` | 事件处理器 | `--event-handlers console_direct+` |
| `--cmake-force-configure` | 强制重新配置 | 每次都重新运行 CMake |

**常用组合命令：**

```bash
# 开发调试配置（推荐）
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug

# 发布版本配置
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# 生成编译命令数据库（配合 VS Code）
colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

# 详细输出（调试编译问题）
colcon build --event-handlers console_direct+
```

### 4.3.5 查看编译结果

```bash
# 查看编译摘要
cat log/latest_build/build_summary.csv

# 查看详细日志
cat log/latest_build/<package_name>/build_stdout.log
```

## 4.4 工作空间覆盖 (Overlaying)

### 4.4.1 覆盖机制

ROS 2 工作空间支持层层覆盖，上层工作空间会覆盖下层工作空间中的同名包。

```
┌────────────────────────────────────────────────────────┐
│                    覆盖机制示意图                        │
├────────────────────────────────────────────────────────┤
│                                                        │
│  ┌──────────────────────────────────────────────┐     │
│  │        /opt/ros/humble (系统层)               │     │
│  │  ├── nav2_bringup                            │     │
│  │  ├── navigation2                             │     │
│  │  └── ...                                     │     │
│  └──────────────────────────────────────────────┘     │
│                        ▲                               │
│                        │ Source 顺序                    │
│  ┌──────────────────────────────────────────────┐     │
│  │     ~/ros2_ws (基础工作空间)                  │     │
│  │  ├── my_package                              │     │
│  │  └── ...                                     │     │
│  └──────────────────────────────────────────────┘     │
│                        ▲                               │
│                        │                               │
│  ┌──────────────────────────────────────────────┐     │
│  │    ~/overlay_ws (覆盖工作空间)                │     │
│  │  └── nav2_bringup (修改后的版本)              │     │
│  └──────────────────────────────────────────────┘     │
│                                                        │
│  结果：overlay_ws 中的 nav2_bringup 会覆盖系统版本      │
│                                                        │
└────────────────────────────────────────────────────────┘
```

### 4.4.2 设置覆盖工作空间

**创建覆盖工作空间：**

```bash
# 创建主工作空间
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
# ... 添加包并编译 ...

# 创建覆盖工作空间
mkdir -p ~/overlay_ws/src
cd ~/overlay_ws/src
# ... 克隆要修改的包 ...
```

**设置环境变量（Source 顺序很重要）：**

```bash
# 正确的 source 顺序（从下到上）
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
source ~/overlay_ws/install/setup.bash
```

**添加到 ~/.bashrc：**

```bash
# 编辑 bashrc
nano ~/.bashrc

# 添加以下内容（注意顺序）
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
source ~/overlay_ws/install/setup.bash
```

### 4.4.3 查看工作空间优先级

```bash
# 查看当前工作空间栈
ros2 pkg prefix --all

# 或者使用 ros2 doctor 检查
ros2 doctor --report
```

## 4.5 环境设置详解

### 4.5.1 setup.bash 脚本

编译完成后，必须 source 环境设置脚本才能使用新编译的包：

```bash
# Source 工作空间环境
source install/setup.bash

# 验证环境变量
echo $ROS_DOMAIN_ID
echo $AMENT_PREFIX_PATH
echo $LD_LIBRARY_PATH
```

**setup.bash 做了什么：**

1. 设置 `AMENT_PREFIX_PATH`（包搜索路径）
2. 设置 `LD_LIBRARY_PATH`（库搜索路径）
3. 设置 `PATH`（可执行文件搜索路径）
4. 设置 `PYTHONPATH`（Python 模块搜索路径）
5. 提供命令自动补全

### 4.5.2 setup.bash vs local_setup.bash

| 脚本 | 用途 |
|-----|------|
| `setup.bash` | 设置当前环境并扩展上游工作空间 |
| `local_setup.bash` | 仅设置当前工作空间，不扩展上游 |

**使用场景：**

```bash
# 开发时使用（推荐）
source install/setup.bash

# 仅测试本地包（避免干扰）
source install/local_setup.bash
```

### 4.5.3 永久设置环境

**方法 1：修改 ~/.bashrc**

```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

**方法 2：创建独立的启动脚本**

```bash
# 创建 ~/ros2_env.sh
cat > ~/ros2_env.sh << 'EOF'
#!/bin/bash
# ROS 2 环境设置脚本

# Source ROS 2 基础环境
source /opt/ros/humble/setup.bash

# Source 主工作空间
source ~/ros2_ws/install/setup.bash

# Source 覆盖工作空间（如果有）
# source ~/overlay_ws/install/setup.bash

# 显示当前配置
echo "ROS 2 Environment Loaded"
echo "ROS_DISTRO: $ROS_DISTRO"
echo "RMW: $RMW_IMPLEMENTATION"
EOF

chmod +x ~/ros2_env.sh
```

## 4.6 包管理

### 4.6.1 列出工作空间中的包

```bash
# 列出所有可用包
ros2 pkg list

# 列出特定包的路径
ros2 pkg prefix <package_name>

# 列出工作空间 src 目录中的包
find src -maxdepth 2 -name package.xml -exec dirname {} \;
```

### 4.6.2 查看包信息

```bash
# 查看包的描述信息
ros2 pkg xml <package_name>

# 查看包的依赖
ros2 pkg dependencies <package_name>

# 导出包列表
ros2 pkg list > packages_list.txt
```

### 4.6.3 包的依赖关系

```
依赖关系示例：

my_robot_package
├── 依赖 (depend)
│   ├── rclcpp
│   ├── std_msgs
│   └── sensor_msgs
├── 构建依赖 (build_depend)
│   ├── ament_cmake
│   └── geometry_msgs
└── 测试依赖 (test_depend)
    └── ament_lint_auto
```

## 4.7 增量编译

### 4.7.1 理解增量编译

Colcon 会检测哪些包被修改，只重新编译必要的包：

```bash
# 首次完整编译
colcon build

# 修改某个包的代码后
# 只重新编译该包及其依赖者
colcon build --packages-select <modified_package>
```

### 4.7.2 强制重新编译

```bash
# 清理并重新编译单个包
colcon build --packages-select <package_name> --cmake-force-configure

# 清理整个工作空间重新编译
rm -rf build install log
colcon build
```

### 4.7.3 编译加速技巧

```bash
# 使用更多并行任务
colcon build --parallel-workers 8

# 只编译修改过的包（默认行为）
colcon build

# 使用 symlink-install 减少复制
colcon build --symlink-install

# 使用 ccache 加速 C++ 编译
sudo apt install ccache
export CC="ccache gcc"
export CXX="ccache g++"
```

## 4.8 测试与验证

### 4.8.1 运行单元测试

```bash
# 编译并运行所有测试
colcon test

# 运行特定包的测试
colcon test --packages-select <package_name>

# 显示详细测试输出
colcon test --packages-select <package_name> --event-handlers console_direct+

# 查看测试结果
colcon test-result --all
colcon test-result --verbose
```

### 4.8.2 验证工作空间

```bash
# 验证包是否正确安装
ros2 pkg list | grep <package_name>

# 验证可执行文件是否可用
ros2 run <package_name> <executable_name> --ros-args --remap __node:=test_node

# 使用 ros2 doctor 检查环境
ros2 doctor --report
```

## 4.9 常用工作空间操作

### 4.9.1 清理工作空间

```bash
# 清理编译产物
rm -rf build/ install/ log/

# 或者使用 colcon 的清理功能
colcon clean --all

# 清理特定包
colcon clean --packages-select <package_name>
```

### 4.9.2 克隆包到工作空间

```bash
cd ~/ros2_ws/src

# 从 GitHub 克隆包
git clone https://github.com/username/package.git

# 从另一个工作空间复制包
cp -r ~/other_ws/src/package .

# 返回根目录并编译
cd ..
colcon build --packages-select <package_name>
```

### 4.9.3 查看工作空间状态

```bash
# 查看编译历史
colcon build-summary --all

# 查看包的构建信息
colcon list

# 查看包的依赖图
colcon graph --all
```

## 4.10 最佳实践

### 4.10.1 工作空间组织

```
推荐的目录结构：

~/
├── ros2_ws/                    # 主开发工作空间
│   ├── src/
│   ├── build/
│   ├── install/
│   └── log/
│
├── overlay_ws/                 # 覆盖工作空间（用于修改第三方包）
│   └── src/
│
├── ros2_test_ws/               # 测试/实验工作空间
│   └── src/
│
└── projects/                   # 项目专用工作空间
    ├── project_a_ws/
    └── project_b_ws/
```

### 4.10.2 开发工作流

```
标准开发流程：

1. 创建/打开工作空间
   cd ~/ros2_ws

2. Source ROS 2 环境
   source /opt/ros/humble/setup.bash

3. 编译修改的包
   colcon build --packages-select <pkg> --symlink-install

4. Source 工作空间环境
   source install/setup.bash

5. 运行测试
   ros2 run <pkg> <node>

6. 如果需要，运行单元测试
   colcon test --packages-select <pkg>
```

### 4.10.3 .colcon 隐藏目录

在工作空间根目录创建 `.colcon` 隐藏目录可以存放 colcon 配置：

```bash
# 创建 .colcon 目录
mkdir -p ~/.colcon

# 创建默认配置文件
cat > ~/.colcon/default.yaml << 'EOF'
# Colcon 默认配置
build:
  symlink-install: true
  cmake-args:
    - -DCMAKE_BUILD_TYPE=Debug
    - -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
EOF
```

## 4.11 常见问题

### 4.11.1 找不到包

**问题：** `ros2 run` 或 `ros2 launch` 找不到包

**解决方法：**

```bash
# 1. 确保已 source 环境
source install/setup.bash

# 2. 检查包是否存在
ros2 pkg list | grep <package_name>

# 3. 重新编译
colcon build --packages-select <package_name>

# 4. 检查 package.xml 中的包名是否正确
cat src/<package_name>/package.xml | grep "<name>"
```

### 4.11.2 库找不到

**问题：** 运行节点时报错找不到共享库

**解决方法：**

```bash
# 检查 LD_LIBRARY_PATH
echo $LD_LIBRARY_PATH | tr ':' '\n' | grep install

# 重新 source 环境
source install/setup.bash

# 清理并重新编译
rm -rf build install
colcon build
```

### 4.11.3 Python 模块导入错误

**问题：** Python 节点无法导入自定义模块

**解决方法：**

```bash
# 检查 PYTHONPATH
echo $PYTHONPATH | tr ':' '\n' | grep install

# 确保使用 symlink-install 编译
colcon build --symlink-install

# 手动添加到 PYTHONPATH（临时）
export PYTHONPATH=$PYTHONPATH:~/ros2_ws/install/<pkg>/lib/python3.10/site-packages
```

## 4.12 下一步

完成工作空间学习后，您可以：

1. **[05 Packages](./05-packages.md)** - 学习创建和管理功能包
2. **[06 Nodes](./06-nodes.md)** - 编写第一个节点

