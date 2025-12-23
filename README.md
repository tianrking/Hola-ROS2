# Hola-ROS2 👋🤖

> **[ES]** Una guía de estudio de ROS 2 simple y directa.
> **[CN]** 一本简洁直接的 ROS 2 学习指南。

---

## 📚 项目简介 / Project Introduction

**Hola-ROS2** 是一套完整的 ROS 2 Humble 学习教程，包含 26 篇详细的文章，涵盖从基础入门到高级应用的全部内容。每篇文章都包含理论说明、代码示例和实践练习。

---

## 🎯 学习目标 / Learning Goals

- 掌握 ROS 2 核心概念（节点、话题、服务、动作、参数）
- 学会使用 RViz2 和 Gazebo 进行仿真和可视化
- 了解 URDF 机器人建模
- 掌握相机校准和 AR 视觉应用

---

## 📖 文档目录 / Course Contents

### 基础篇 (Basics)

| 章节 | 标题 | 内容 |
|-----|------|------|
| [01](./docs/01-ros2-introduction.md) | ROS 2 简介 | 核心概念、架构、发行版本 |
| [02](./docs/02-installation-humble.md) | 安装 Humble | 系统要求、安装步骤、验证 |
| [03](./docs/03-ide-setup.md) | IDE 配置 | VS Code + ROS 扩展配置 |
| [04](./docs/04-workspace.md) | 工作空间 | Colcon 编译、覆盖机制 |
| [05](./docs/05-packages.md) | 功能包 | 包创建、依赖管理 |
| [06](./docs/06-nodes.md) | 节点 | C++/Python 节点编程 |

### 通信篇 (Communication)

| 章节 | 标题 | 内容 |
|-----|------|------|
| [07](./docs/07-topics.md) | 话题通信 | 发布/订阅、QoS 策略 |
| [08](./docs/08-services.md) | 服务通信 | 客户端/服务端 |
| [09](./docs/09-actions.md) | 动作通信 | 长任务处理、反馈 |
| [10](./docs/10-tf2.md) | TF2 变换 | 坐标系管理 |

### 进阶篇 (Advanced)

| 章节 | 标题 | 内容 |
|-----|------|------|
| [11](./docs/11-custom-interfaces.md) | 自定义接口 | msg/srv/action 定义 |
| [12](./docs/12-parameters.md) | 参数服务器 | 动态参数配置 |
| [13](./docs/13-metapackages.md) | 元包 | 包组织管理 |
| [14](./docs/14-distributed-communication.md) | 分布式通信 | 多机器人通信 |
| [15](./docs/15-dds.md) | DDS 中间件 | RMW、QoS 深入 |
| [16](./docs/16-time-apis.md) | 时间 API | 定时、速率控制 |

### 工具篇 (Tools)

| 章节 | 标题 | 内容 |
|-----|------|------|
| [17](./docs/17-cli-tools.md) | 命令行工具 | ros2 命令详解 |
| [18](./docs/18-rviz2.md) | RViz2 | 3D 可视化 |
| [19](./docs/19-rqt-tools.md) | Rqt 工具 | 图形化工具集 |
| [20](./docs/20-launch.md) | Launch 文件 | 系统启动配置 |
| [21](./docs/21-rosbag2.md) | Rosbag2 | 数据记录回放 |

### 应用篇 (Applications)

| 章节 | 标题 | 内容 |
|-----|------|------|
| [22](./docs/22-urdf.md) | URDF 建模 | 机器人描述 |
| [23](./docs/23-gazebo.md) | Gazebo 仿真 | 物理仿真环境 |
| [24](./docs/24-camera.md) | 摄像头 | 相机驱动配置 |
| [25](./docs/25-camera-calibration.md) | 相机校准 | 内参标定 |
| [26](./docs/26-ar-vision.md) | AR 视觉 | ArUco 标记检测 |

---

## 🚀 快速开始 / Quick Start

### 1. 安装 ROS 2 Humble

```bash
# 添加 ROS 2 APT 仓库
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 安装
sudo apt update
sudo apt install -y ros-humble-desktop

# 配置环境
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2. 验证安装

```bash
# 运行示例
ros2 run turtlesim turtlesim_node
```

---

## 🧰 技术栈 / Tech Stack

| 技术 | 版本 | 说明 |
|-----|------|------|
| **ROS 2** | Humble Hawksbill | LTS 版本，支持至 2027 年 |
| **C++** | C++17 | rclcpp 客户端库 |
| **Python** | 3.10+ | rclpy 客户端库 |
| **Gazebo** | 11 | 物理仿真 |
| **RViz2** | Humble | 3D 可视化 |

---

## 📂 项目结构 / Project Structure

```
Hola-ROS2/
├── docs/                    # 教程文档
│   ├── 01-ros2-introduction.md
│   ├── 02-installation-humble.md
│   ├── ...
│   └── 26-ar-vision.md
├── media/                   # 图片资源
├── README.md                # 项目说明
└── TODO.md                  # 课程大纲
```

---

## 🌟 特性 / Features

- **完整覆盖** - 26 篇文章，涵盖 ROS 2 全部核心内容
- **双语支持** - 中文为主，部分内容包含英文对照
- **代码示例** - C++ 和 Python 双语示例
- **实践导向** - 每个概念都配有实际代码和操作步骤

---

## 📖 学习路径 / Learning Path

```
基础入门 (1-6)
    ↓
通信机制 (7-10)
    ↓
进阶功能 (11-16)
    ↓
开发工具 (17-21)
    ↓
应用实践 (22-26)
```

---

## 🤝 贡献 / Contributing

欢迎提交 Issue 和 Pull Request！

---

## 📄 许可证 / License

MIT License

---

*Less theory, more code. / 少讲理论，多写代码。*
