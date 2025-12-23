# 19 Rqt 工具箱 (Rqt Tools)

## 19.1 Rqt 概述

### 19.1.1 什么是 Rqt

**Rqt** 是 ROS 2 的图形化工具框架，提供了一套可扩展的可视化工具插件。

```
Rqt 插件生态：

┌─────────────────────────────────────────────────┐
│                   Rqt 框架                       │
├─────────────────────────────────────────────────┤
│                                                 │
│  ┌─────────┐  ┌─────────┐  ┌─────────┐       │
│  │rqt_graph│  │rqt_plot │  │rqt_console│      │
│  │(节点图) │  │(数据绘图)│  │(日志查看)│       │
│  └─────────┘  └─────────┘  └─────────┘       │
│                                                 │
│  ┌─────────┐  ┌─────────┐  ┌─────────┐       │
│  │rqt_reconfigure│ │rqt_tf_tree│ │rqt_bag│ │
│  │(参数配置)│  │(TF树)  │  │(数据包) │       │
│  └─────────┘  └─────────┘  └─────────┘       │
│                                                 │
└─────────────────────────────────────────────────┘
```

### 19.1.2 安装 Rqt

```bash
# 安装完整的 rqt 工具集
sudo apt install ros-humble-rqt*
sudo apt install ros-humble-rqt-common-plugins

# 或安装特定工具
sudo apt install ros-humble-rqt-graph
sudo apt install ros-humble-rqt-plot
sudo apt install ros-humble-rqt-console
```

## 19.2 Rqt Graph（节点图）

### 19.2.1 启动

```bash
# 启动 rqt_graph
ros2 run rqt_graph rqt_graph
```

### 19.2.2 功能

| 功能 | 说明 |
|-----|------|
| **节点可视化** | 显示所有活动节点 |
| **连接显示** | 显示话题连接 |
| **节点过滤** | 过滤特定节点 |
| **刷新** | 实时更新节点图 |

### 19.2.3 使用

```
节点图示例：

/talkerr    ──/chatter──▶  /listener
                             ▲
                        /chatter
                             │
                    /another_listener

- 方块: 节点
- 箭头: 话题连接
- 方向: 数据流向
```

### 19.2.4 工具栏

| 工具 | 功能 |
|-----|------|
| 刷新 | 更新节点图 |
| 排列布局 | 自动排列节点 |
| 导出图 | 保存为图片 |
| 帮助 | 显示帮助 |

## 19.3 Rqt Plot（数据绘图）

### 19.3.1 启动

```bash
# 启动 rqt_plot
ros2 run rqt_plot rqt_plot

# 或直接绘图
ros2 run rqt_plot rqt_plot /topic_name/field_name
```

### 19.3.2 添加绘图

```
1. 点击左上角 "+" 按钮
2. 输入话题名称
3. 选择字段
4. 设置显示样式
```

### 19.3.3 绘图配置

| 设置 | 说明 |
|-----|------|
| Topic | 话题名称 |
| Field | 字段路径 |
| Type | 数据类型 |
| Color | 线条颜色 |

### 19.3.4 常用操作

```bash
# 绘制单个话题
rqt_plot /cmd_vel/linear/x

# 绘制多个话题
rqt_plot /cmd_vel/linear/x /cmd_vel/angular/z

# 绘制数组元素
rqt_plot /joint_states/position[0] /joint_states/position[1]
```

## 19.4 Rqt Console（日志查看）

### 19.4.1 启动

```bash
# 启动 rqt_console
ros2 run rqt_console rqt_console
```

### 19.4.2 功能

| 功能 | 说明 |
|-----|------|
| **日志显示** | 显示所有节点日志 |
| **级别过滤** | 过滤日志级别 |
| **节点过滤** | 过滤特定节点 |
| **日志导出** | 导出日志到文件 |

### 19.4.3 日志级别

| 级别 | 说明 |
|-----|------|
| Debug | 调试信息 |
| Info | 一般信息 |
| Warn | 警告 |
| Error | 错误 |
| Fatal | 致命错误 |

### 19.4.4 过滤器

```
日志过滤：

- 节点过滤器: 只显示特定节点的日志
- 严重性过滤器: 只显示特定级别及以上的日志
- 消息过滤器: 搜索包含特定文本的日志
- 正则表达式: 使用正则表达式匹配
- 排除过滤器: 排除匹配的消息
```

## 19.5 Rqt Reconfigure（参数配置）

### 19.5.1 启动

```bash
# 启动 rqt_reconfigure
ros2 run rqt_reconfigure rqt_reconfigure
```

### 19.5.2 功能

| 功能 | 说明 |
|-----|------|
| **参数查看** | 显示所有节点参数 |
| **动态修改** | 运行时修改参数 |
| **参数保存** | 保存参数配置 |

### 19.5.3 使用

```
参数配置界面：

节点列表:
├── /camera_node
│   ├── exposure: [===|===] 1.5
│   ├── gain:      [==|========] 0.8
│   └── enabled:   ☑
│
└── /laser_node
    ├── scan_frequency: [==========|==] 9.5
    └── max_range: 10.0
```

## 19.6 Rqt TF Tree（TF 树）

### 19.6.1 启动

```bash
# 启动 rqt_tf_tree
ros2 run rqt_tf_tree rqt_tf_tree
```

### 19.6.2 功能

| 功能 | 说明 |
|-----|------|
| **TF 树显示** | 可视化坐标变换树 |
| **树结构** | 显示父子关系 |
| **刷新** | 实时更新 |

### 19.6.3 显示

```
TF 树示例:

map
└── odom
    └── base_link
        ├── camera_link
        ├── laser_link
        └── imu_link
```

## 19.7 Rqt Bag（数据包）

### 19.7.1 启动

```bash
# 启动 rqt_bag
ros2 run rqt_bag rqt_bag
```

### 19.7.2 功能

| 功能 | 说明 |
|-----|------|
| **包浏览** | 查看包内容 |
| **时间轴** | 显示时间线 |
| **消息查看** | 查看消息详情 |
| **播放控制** | 播放/暂停/跳转 |

## 19.8 Rqt Image View（图像查看）

### 19.8.1 启动

```bash
# 启动 rqt_image_view
ros2 run rqt_image_view rqt_image_view
```

### 19.8.2 功能

| 功能 | 说明 |
|-----|------|
| **图像显示** | 显示相机图像话题 |
| **多窗口** | 同时显示多个图像 |
| **保存图像** | 保存当前帧 |

## 19.9 Rqt Shell

### 19.9.1 启动

```bash
# 启动 rqt_shell
ros2 run rqt_shell rqt_shell
```

### 19.9.2 功能

在 Rqt 中集成终端，执行 ROS 2 命令。

## 19.10 Rqt Robot Steering

### 19.10.1 启动

```bash
# 启动机器人遥控
ros2 run rqt_robot_steering rqt_robot_steering
```

### 19.10.2 功能

| 功能 | 说明 |
|-----|------|
| **速度控制** | 发布 /cmd_vel |
| **键盘控制** | 方向键控制 |
| **平滑输出** | 可配置的加速度 |

## 19.11 Rqt Action Client

### 19.11.1 启动

```bash
# 启动动作客户端
ros2 run rqt_action rqt_action
```

### 19.11.2 功能

- 查看可用动作
- 发送动作目标
- 查看反馈
- 取消动作

## 19.12 Rqt Publisher

### 19.12.1 启动

```bash
# 启动发布者
ros2 run rqt_publisher rqt_publisher
```

### 19.12.2 功能

- 选择消息类型
- 填充消息字段
- 发布消息
- 设置发布频率

## 19.13 Rqt Service Caller

### 19.13.1 启动

```bash
# 启动服务调用器
ros2 run rqt_service_caller rqt_service_caller
```

### 19.13.2 功能

- 查看可用服务
- 填充请求字段
- 调用服务
- 显示响应

## 19.14 最佳实践

| 场景 | 推荐工具 |
|-----|---------|
| 系统概览 | rqt_graph |
| 数据分析 | rqt_plot |
| 调试日志 | rqt_console |
| 参数调优 | rqt_reconfigure |
| TF 调试 | rqt_tf_tree |
| 图像查看 | rqt_image_view |
| 包分析 | rqt_bag |

## 19.15 下一步

1. **[20 Launch](./20-launch.md)** - 学习 Launch 配置
2. **[21 Rosbag2](./21-rosbag2.md)** - 学习数据包录制

