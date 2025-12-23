# 17 常用命令工具 (CLI Tools)

## 17.1 命令行工具概述

ROS 2 提供了一套完整的命令行工具，用于调试、监控和与 ROS 2 系统交互。

| 工具类别 | 命令 |
|---------|------|
| **节点管理** | `ros2 node` |
| **话题管理** | `ros2 topic` |
| **服务管理** | `ros2 service` |
| **参数管理** | `ros2 param` |
| **动作管理** | `ros2 action` |
| **数据包管理** | `ros2 bag` |
| **接口查看** | `ros2 interface` |
| **包管理** | `ros2 pkg` |
| **daemon** | `ros2 daemon` |

## 17.2 节点命令 (ros2 node)

### 17.2.1 列出节点

```bash
# 列出所有活动节点
ros2 node list

# 列出节点及其类型
ros2 node list -t

# 列出所有节点包括隐藏节点
ros2 node list --all
```

### 17.2.2 查看节点信息

```bash
# 查看节点详细信息
ros2 node info /node_name

# 查看节点的订阅者、发布者、服务等
ros2 node info /turtlesim
```

**输出示例：**
```
/turtlesim
  Subscribers:
    /turtle1/cmd_vel: geometry_msgs/msg/Twist
  Publishers:
    /turtle1/color_sensor: turtlesim/msg/Color
    /turtle1/pose: turtlesim/msg/Pose
  Service Servers:
    /spawn: turtlesim/srv/Spawn
    /teleport_absolute: turtlesim/srv/TeleportAbsolute
```

### 17.2.3 管理节点生命周期

```bash
# 安全终止节点
ros2 node kill /node_name

# 强制终止
ros2 node kill /node_name --force
```

## 17.3 话题命令 (ros2 topic)

### 17.3.1 列出话题

```bash
# 列出所有话题
ros2 topic list

# 列出话题及其类型
ros2 topic list -t

# 列出话题详细信息
ros2 topic list -v
```

### 17.3.2 查看话题信息

```bash
# 查看话题类型
ros2 topic type /topic_name

# 查看话题详细信息（包括 QoS）
ros2 topic info /topic_name

# 查看话题详细信息（包括 QoS 策略）
ros2 topic info /topic_name -v
```

### 17.3.3 话题数据回显

```bash
# 回显话题消息
ros2 topic echo /topic_name

# 回显特定字段
ros2 topic echo /topic_name --field header.frame_id

# 回显带时间戳
ros2 topic echo /topic_name --field header.stamp

# 回显 JSON 格式
ros2 topic echo /topic_name --json-output

# 只回显 N 条消息
ros2 topic echo /topic_name --once
ros2 topic echo /topic_name --times 10
```

### 17.3.4 发布消息到话题

```bash
# 发布消息
ros2 topic pub /topic_name std_msgs/msg/String "{data: 'Hello'}"

# 持续发布（指定频率）
ros2 topic pub --rate 10 /topic_name std_msgs/msg/String "{data: 'Hello'}"

# 发布一次
ros2 topic pub --once /topic_name std_msgs/msg/String "{data: 'Hello'}"

# 从文件发布
ros2 topic pub /topic_name std_msgs/msg/String --from-file message.json
```

### 17.3.5 话题频率与带宽

```bash
# 计算话题发布频率
ros2 topic hz /topic_name

# 指定窗口大小
ros2 topic hz /topic_name --window 50

# 查看话题带宽
ros2 topic bw /topic_name

# 持续监控
ros2 topic hz /chatter
```

## 17.4 服务命令 (ros2 service)

### 17.4.1 列出服务

```bash
# 列出所有服务
ros2 service list

# 列出服务及其类型
ros2 service list -t

# 过滤服务
ros2 service list | grep /spawn
```

### 17.4.2 查看服务信息

```bash
# 查看服务类型
ros2 service type /service_name

# 查看服务定义
ros2 service interface /service_name

# 查找提供服务的节点
ros2 service find /service_name
```

### 17.4.3 调用服务

```bash
# 调用服务
ros2 service call /service_name std_srvs/srv/Empty

# 调用带参数的服务
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 5, b: 3}"

# 调用并等待响应
ros2 service call /spawn turtlesim/srv/Spawn "{x: 5.0, y: 5.0, theta: 0.0, name: 'turtle2'}"
```

## 17.5 参数命令 (ros2 param)

### 17.5.1 列出参数

```bash
# 列出节点的所有参数
ros2 param list /node_name

# 列出所有节点的参数
ros2 param list

# 查看参数值
ros2 param get /node_name parameter_name

# 描述参数
ros2 param describe /node_name parameter_name
```

### 17.5.2 设置参数

```bash
# 设置参数
ros2 param set /node_name parameter_name value

# 设置数值参数
ros2 param set /camera_node exposure 1.5

# 设置字符串参数
ros2 param set /camera_node frame_id "camera_link"

# 设置数组参数
ros2 param set /camera_node size "[640, 480]"
```

### 17.5.3 导出/导入参数

```bash
# 导出参数到文件
ros2 param dump /node_name parameters.yaml

# 导出所有节点参数
ros2 param dump --all

# 从文件加载参数
ros2 param load /node_name parameters.yaml

# 删除参数
ros2 param delete /node_name parameter_name
```

## 17.6 动作命令 (ros2 action)

### 17.6.1 列出动作

```bash
# 列出所有动作
ros2 action list

# 列出动作及其类型
ros2 action list -t
```

### 17.6.2 查看动作信息

```bash
# 查看动作类型
ros2 action type /action_name

# 查看动作定义
ros2 action interface example_interfaces/action/Fibonacci
```

### 17.6.3 发送动作目标

```bash
# 发送动作目标
ros2 action send_goal /fibonacci example_interfaces/action/Fibonacci "{order: 10}"

# 带反馈的动作调用
ros2 action send_goal /fibonacci example_interfaces/action/Fibonacci "{order: 10}" --feedback

# 取消动作
# Ctrl+C 或发送取消请求
```

## 17.7 包命令 (ros2 pkg)

### 17.7.1 包信息

```bash
# 列出所有包
ros2 pkg list

# 查找包
ros2 pkg list | grep nav2

# 查看包前缀（安装路径）
ros2 pkg prefix package_name

# 查看包 XML
ros2 pkg xml package_name

# 导出包列表
ros2 pkg list > packages.txt
```

### 17.7.2 包依赖

```bash
# 列出依赖
ros2 pkg dependencies package_name

# 递归列出所有依赖
ros2 pkg dependencies package_name --all

# 列出依赖于此包的包
ros2 pkg dependents package_name
```

### 17.7.3 包创建

```bash
# 创建 C++ 包
ros2 pkg create --build-type ament_cmake --dependencies rclcpp my_package

# 创建 Python 包
ros2 pkg create --build-type ament_python --dependencies rclpy my_package

# 创建包并添加节点模板
ros2 pkg create --build-type ament_cmake --node-name my_node --dependencies rclcpp my_package
```

## 17.8 接口命令 (ros2 interface)

### 17.8.1 列出接口

```bash
# 列出所有接口
ros2 interface list

# 列出消息接口
ros2 interface list -m

# 列出服务接口
ros2 interface list -s

# 列出动作接口
ros2 interface list -a
```

### 17.8.2 查看接口定义

```bash
# 查看消息定义
ros2 interface show std_msgs/msg/String

# 查看服务定义
ros2 interface show example_interfaces/srv/AddTwoInts

# 查看动作定义
ros2 interface show example_interfaces/action/Fibonacci

# 搜索接口
ros2 interface list | grep geometry_msgs
```

## 17.9 Daemon 命令

```bash
# 启动 daemon
ros2 daemon start

# 停止 daemon
ros2 daemon stop

# 重启 daemon
ros2 daemon restart

# 查看状态
ros2 daemon status
```

## 17.10 实用技巧

### 17.10.1 命令组合

```bash
# 查看特定节点的话题
ros2 topic list | grep /node_name

# 监控话题频率
watch -n 1 "ros2 topic hz /chatter"

# 查找特定类型的消息
ros2 topic list -t | grep std_msgs

# 导出系统状态
ros2 topic list > topics.txt
ros2 node list > nodes.txt
ros2 service list > services.txt
```

### 17.10.2 管道和过滤

```bash
# 过滤话题
ros2 topic list | grep /cmd

# 统计话题数量
ros2 topic list | wc -l

# 查找特定节点的信息
ros2 node list | xargs -I {} ros2 node info {}
```

## 17.11 调试工作流

```bash
# 1. 检查节点
ros2 node list

# 2. 检查话题
ros2 topic list -t

# 3. 查看话题信息
ros2 topic info /topic_name -v

# 4. 回显消息
ros2 topic echo /topic_name

# 5. 测试发布
ros2 topic pub /topic_name std_msgs/msg/String "{data: 'test'}"

# 6. 检查参数
ros2 param list /node_name

# 7. 使用诊断
ros2 doctor --report
```

## 17.12 下一步

1. **[18 RViz2](./18-rviz2.md)** - 学习 RViz2 可视化
2. **[19 Rqt Tools](./19-rqt-tools.md)** - 学习 Rqt 工具

