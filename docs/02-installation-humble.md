# 02 安装 Humble (Installation)

## 2.1 安装前准备

### 2.1.1 系统要求

ROS 2 Humble Hawksbill 官方支持的操作系统为 Ubuntu 22.04 (Jammy Jellyfish)。

| 硬件/软件 | 最低要求 | 推荐配置 |
|----------|---------|---------|
| **操作系统** | Ubuntu 22.04 (Jammy) | Ubuntu 22.04 LTS |
| **CPU** | 双核处理器 | 四核或更高 |
| **内存** | 4 GB RAM | 8 GB 或更多 |
| **磁盘空间** | 10 GB 可用空间 | 30 GB 或更多 |
| **网络** | 稳定的互联网连接 | 稳定的互联网连接 |

### 2.1.2 支持的架构

ROS 2 Humble 支持以下处理器架构：

- **amd64** (x86_64) - 桌面和服务器处理器
- **arm64** (aarch64) - ARM 64位处理器（如 NVIDIA Jetson、树莓派 4）

### 2.1.3 语言支持

ROS 2 Humble 默认支持以下编程语言：

| 语言 | 版本要求 | 说明 |
|-----|---------|------|
| **Python** | Python 3.10+ | rclpy 客户端库 |
| **C++** | C++17 标准 | rclcpp 客户端库 |

### 2.1.4 添加语言支持（可选）

如果您需要使用其他语言版本的 ROS 2 消息和接口：

```bash
# 确保系统已安装 locale 支持
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# 验证设置
locale
```

## 2.2 标准安装方法（二进制包）

### 2.2.1 安装步骤概览

```
┌────────────────────────────────────────────────────────┐
│              ROS 2 Humble 安装流程                      │
├────────────────────────────────────────────────────────┤
│                                                        │
│  1. 设置 Locale                                         │
│          │                                             │
│          ▼                                             │
│  2. 添加 ROS 2 APT 仓库                                │
│          │                                             │
│          ▼                                             │
│  3. 更新 apt 索引                                      │
│          │                                             │
│          ▼                                             │
│  4. 安装 ROS 2 软件包                                  │
│          │                                             │
│          ▼                                             │
│  5. 配置环境变量                                        │
│          │                                             │
│          ▼                                             │
│  6. 验证安装                                            │
│                                                        │
└────────────────────────────────────────────────────────┘
```

### 2.2.2 设置 Locale

首先确保系统使用 UTF-8 编码：

```bash
# 设置 locale
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# 验证 locale 设置
locale
```

**预期输出：**
```
LANG=en_US.UTF-8
LANGUAGE=en_US
LC_ALL=en_US.UTF-8
```

### 2.2.3 添加 ROS 2 APT 仓库

添加 Ubuntu Universe 仓库和 ROS 2 GPG 密钥：

```bash
# 确保 Ubuntu Universe 仓库已启用
sudo apt install -y software-properties-common
sudo add-apt-repository universe

# 添加 ROS 2 GPG 密钥
sudo apt update && sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# 添加 ROS 2 仓库到源列表
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

**说明：**
- `$(dpkg --print-architecture)` 自动检测系统架构（如 amd64）
- `$(. /etc/os-release && echo $UBUNTU_CODENAME)` 自动获取 Ubuntu 版本代号（jammy）

### 2.2.4 更新 apt 索引

```bash
# 更新软件包索引
sudo apt update
```

### 2.2.5 安装 ROS 2 软件包

ROS 2 Humble 提供多种安装选项，根据需求选择：

**选项 1：完整桌面版安装（推荐）**

包含 ROS 2、RViz2、演示工具、教程和开发工具：

```bash
sudo apt install -y ros-humble-desktop
```

**选项 2：ROS-Base 基础版（不含 GUI 工具）**

适合服务器或无头系统：

```bash
sudo apt install -y ros-humble-ros-base
```

**选项 3：完整开发环境安装**

包含桌面版内容，加上用于构建 ROS 2 包的开发工具：

```bash
sudo apt install -y ros-humble-desktop
sudo apt install -y ros-dev-tools
```

**软件包组成对比：**

| 软件包 | 包含内容 | 大小 | 适用场景 |
|-------|---------|------|---------|
| `ros-humble-ros-base` | 核心 ROS 2 消息、通信层、工具 | ~500 MB | 服务器、无头系统 |
| `ros-humble-desktop` | ros-base + RViz2 + 通用工具 | ~1.5 GB | 桌面开发、学习 |
| `ros-dev-tools` | colcon、ament_cmake、vcstool | ~100 MB | 包开发 |

### 2.2.6 配置环境变量

在使用 ROS 2 之前，需要将 ROS 2 环境脚本添加到 Shell 配置文件中：

```bash
# 对于 bash shell
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# 对于 zsh shell
echo "source /opt/ros/humble/setup.zsh" >> ~/.zshrc
source ~/.zshrc
```

**环境脚本说明：**

| 脚本文件 | 用途 |
|---------|------|
| `/opt/ros/humble/setup.bash` | bash 环境设置 |
| `/opt/ros/humble/setup.zsh` | zsh 环境设置 |
| `/opt/ros/humble/local_setup.bash` | 仅设置本地环境，不扩展 |

### 2.2.7 安装常用工具（推荐）

安装 ROS 2 开发常用的额外工具：

```bash
# 安装 ROS 2 开发工具
sudo apt install -y \
    python3-argcomplete \
    python3-colcon-common-extensions \
    python3-rosdep

# 安装 rosdep（依赖管理工具）
sudo rosdep init
rosdep update
```

**工具说明：**

| 工具 | 用途 |
|-----|------|
| `python3-argcomplete` | 命令行自动补全 |
| `python3-colcon-common-extensions` | colcon 编译工具扩展 |
| `python3-rosdep` | ROS 依赖管理工具 |

### 2.2.8 安装用于自动补全的 argcomplete

```bash
# 启用 argcomplete
sudo apt install -y python3-argcomplete
activate-global-python-argcomplete
```

## 2.3 验证安装

### 2.3.1 检查 ROS 2 版本

```bash
# 方法 1：使用 print_rmw_vars
print_rmw_vars

# 方法 2：检查环境变量
echo $ROS_DISTRO
echo $ROS_LOCALHOST_ONLY
echo $ROS_PYTHON_VERSION
```

**预期输出：**
```
ROS_DISTRO=humble
ROS_LOCALHOST_ONLY=0
ROS_PYTHON_VERSION=3
```

### 2.3.2 运行示例节点

运行 turtlesim 演示来验证安装：

**终端 1 - 启动 turtlesim 节点：**
```bash
ros2 run turtlesim turtlesim_node
```

**终端 2 - 启动键盘控制节点：**
```bash
ros2 run turtlesim turtle_teleop_key
```

如果安装成功，您将看到一个包含海龟的图形窗口，并且可以使用键盘方向键控制海龟移动。

### 2.3.3 测试常用命令

```bash
# 列出所有节点
ros2 node list

# 列出所有话题
ros2 topic list

# 列出所有服务
ros2 service list

# 列出所有动作
ros2 action list

# 查看节点信息
ros2 node info /turtlesim
```

**预期输出示例：**
```
/ros2_cli_tutorials
/turtlesim
/teleop_turtle
```

### 2.3.4 检查 RMW 实现

```bash
# 检查当前使用的 RMW 实现
echo $RMW_IMPLEMENTATION

# 列出可用的 RMW 实现
ros2 doctor --report | grep rmw
```

**默认 RMW 实现：**
- Humble 默认：`rmw_cyclonedds_cpp`

## 2.4 从源码构建安装

### 2.4.1 何时需要从源码构建

| 场景 | 说明 |
|-----|------|
| **修改 ROS 2 核心代码** | 需要修改或调试 ROS 2 本身的代码 |
| **最新功能** | 需要使用尚未发布的最新功能 |
| **自定义构建选项** | 需要特定的编译选项或优化 |
| **学习目的** | 想要深入了解 ROS 2 构建系统 |

### 2.4.2 准备开发环境

```bash
# 安装开发工具
sudo apt update && sudo apt install -y \
    build-essential \
    cmake \
    git \
    python3-pip \
    python3-rosdep \
    wget

# 安装 ROS 2 构建依赖
sudo apt install -y \
    python3-flake8 \
    python3-pydocstyle \
    python3-pytest-cov \
    python3-rosinstall-generator \
    python3-vcstool

# 安装 Python 依赖
pip install -U \
    flake8 \
    pytest \
    pytest-cov \
    pytest-repeat \
    pytest-runner \
    pytest-rerunfailures
```

### 2.4.3 获取源码

```bash
# 创建工作空间
mkdir -p ~/ros2_humble/src
cd ~/ros2_humble

# 下载 ROS 2 源码
wget https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos
vcs import src < ros2.repos
```

**repos 文件结构：**
```
ros2.repos:
├── ros2/rclcpp         # C++ 客户端库
├── ros2/rclpy          # Python 客户端库
├── ros2/rmw            # ROS 中间件接口
├── ros2/rmw_cyclonedds # CycloneDDS 实现
├── ros2/ros2cli        # 命令行工具
├── ...                 # 其他核心包
```

### 2.4.4 安装依赖

```bash
# 使用 rosdep 安装依赖
sudo apt upgrade -y
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### 2.4.5 编译源码

```bash
# 使用 colcon 编译
cd ~/ros2_humble
colcon build --symlink-install --packages-skip ros2_control

# 或者编译所有包（耗时较长）
colcon build --symlink-install
```

**编译选项说明：**

| 选项 | 说明 |
|-----|------|
| `--symlink-install` | 使用符号链接，便于修改后立即生效 |
| `--packages-skip` | 跳过某些包的编译 |
| `--cmake-args` | 传递额外 CMake 参数 |

### 2.4.6 设置环境

```bash
# 添加到 bashrc
echo "source ~/ros2_humble/install/setup.bash" >> ~/.bashrc
source ~/.bashrc

# 验证
echo $ROS_DISTRO  # 应输出 humble
```

## 2.5 卸载 ROS 2

### 2.5.1 卸载二进制安装

```bash
# 移除所有 ROS 2 包
sudo apt remove -y ~nros-humble-* ~nros-*

# 移除 rosdep
sudo apt remove -y python3-rosdep

# 移除 APT 仓库
sudo rm /etc/apt/sources.list.d/ros2.list
sudo apt update

# 移除环境变量（编辑 ~/.bashrc）
# 删除包含 "source /opt/ros/humble" 的行
```

### 2.5.2 清理源码构建

```bash
# 删除源码工作空间
rm -rf ~/ros2_humble

# 从 ~/.bashrc 中移除环境变量
# 编辑 ~/.bashrc，删除对应行
```

## 2.6 环境配置详解

### 2.6.1 环境变量

ROS 2 重要的环境变量：

| 环境变量 | 默认值 | 说明 |
|---------|-------|------|
| `ROS_DISTRO` | humble | ROS 2 发行版本名称 |
| `ROS_LOCALHOST_ONLY` | 0 | 是否仅限本地通信 |
| `ROS_PYTHON_VERSION` | 3 | Python 版本 |
| `RMW_IMPLEMENTATION` | rmw_cyclonedds_cpp | ROS 中间件实现 |
| `ROS_DOMAIN_ID` | 0 | DDS 域 ID（用于多机器人隔离） |

### 2.6.2 设置 ROS_DOMAIN_ID

在多机器人环境中，需要为每个机器人设置不同的域 ID：

```bash
# 设置域 ID（临时）
export ROS_DOMAIN_ID=42

# 永久设置（添加到 ~/.bashrc）
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
```

**域 ID 使用建议：**

| 场景 | 域 ID | 说明 |
|-----|-------|------|
| 单机器人 | 0 | 默认域 ID |
| 机器人 1 | 1 | 第一台机器人 |
| 机器人 2 | 2 | 第二台机器人 |
| 仿真环境 | 10-20 | 仿真测试使用 |

### 2.6.3 设置 RMW 实现

切换不同的 DDS 实现：

```bash
# 使用 FastDDS
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# 使用 CycloneDDS（默认）
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 永久设置
echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> ~/.bashrc
```

## 2.7 常见问题排查

### 2.7.1 GPG 密钥错误

**问题：** 添加 APT 仓库时出现 GPG 密钥错误

```
The following signatures couldn't be verified because the public key is not available: NO_PUBKEY ...
```

**解决方法：**
```bash
# 重新添加 GPG 密钥
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# 然后重新更新
sudo apt update
```

### 2.7.2 节点发现失败

**问题：** 节点无法互相发现

**解决方法：**

1. **检查防火墙设置：**
```bash
# 允许 DDS 端口
sudo ufw allow 7400-7400/udp
sudo ufw allow 7411/udp
```

2. **检查网络配置：**
```bash
# 设置组播路由
sudo ip route add 224.0.0.0/4 dev eth0
```

3. **禁用 localhost only：**
```bash
export ROS_LOCALHOST_ONLY=0
```

### 2.7.3 colcon 命令未找到

**问题：** colcon 命令不可用

**解决方法：**
```bash
# 安装 colcon
sudo apt install -y python3-colcon-common-extensions

# 重新加载环境
source /opt/ros/humble/setup.bash
```

### 2.7.4 Python 版本问题

**问题：** Python 版本不匹配

**解决方法：**
```bash
# 确保使用 Python 3
python3 --version  # 应该是 3.10+

# 如果默认是 Python 2，更新 alternatives
sudo update-alternatives --install /usr/bin/python python /usr/bin/python3 1
```

### 2.7.5 RViz2 启动失败

**问题：** RViz2 无法启动或崩溃

**解决方法：**
```bash
# 安装缺失的依赖
sudo apt install -y libogre-1.9-dev libqt5core5a

# 检查显卡驱动
glxinfo | grep "OpenGL renderer"
```

## 2.8 安装后配置

### 2.8.1 配置自动补全

启用 ROS 2 命令自动补全：

```bash
# 对于 bash
echo "source /usr/share/colcon_argcomplete/bash/colcon" >> ~/.bashrc

# 对于 zsh
echo "source /usr/share/colcon_argcomplete/zsh/colcon" >> ~/.zshrc
```

### 2.8.2 配置 Git（推荐）

如果计划从源码构建或参与开发：

```bash
# 配置 Git 用户信息
git config --global user.name "Your Name"
git config --global user.email "your.email@example.com"

# 配置 Git 来处理行尾符
git config --global core.autocrlf input
```

### 2.8.3 创建工作空间

创建用于开发的工作空间：

```bash
# 创建工作空间目录
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# 初始化工作空间
# 这里可以添加您自己的包
```

**工作空间结构：**
```
~/ros2_ws/
├── build/           # 编译输出目录
├── install/         # 安装目录
├── log/             # 日志目录
└── src/             # 源码目录
    ├── package_1/
    ├── package_2/
    └── ...
```

## 2.9 下一步

安装完成后，建议按照以下顺序学习：

1. **[03 IDE Setup](./03-ide-setup.md)** - 配置开发环境
2. **[04 Workspace](./04-workspace.md)** - 学习工作空间和编译
3. **[05 Packages](./05-packages.md)** - 创建和管理功能包
4. **[06 Nodes](./06-nodes.md)** - 编写第一个节点

## 2.10 参考资源

- [官方安装文档](https://docs.ros.org/en/humble/Installation.html)
- [从源码构建](https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html)
- [ROS 2 问题追踪](https://github.com/ros2/ros2/issues)

---
**✅ 02 安装 Humble - 已完成**
