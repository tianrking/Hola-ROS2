# 14 分布式通讯 (Distributed Communication)

## 14.1 分布式通信概述

### 14.1.1 什么是分布式通信

ROS 2 的分布式通信允许多个计算机上的节点互相通信，无需中心服务器。这是通过 DDS (Data Distribution Service) 实现的。

```
分布式通信架构：

        网络交换机/路由器
              │
    ┌─────────┼─────────┐
    │         │         │
┌─────┐  ┌─────┐  ┌─────┐
│ PC1 │  │ PC2 │  │ PC3 │
│     │  │     │  │     │
│传感器│  │控制 │  │可视化│
│ 节点 │  │节点 │  │ 节点 │
└─────┘  └─────┘  └─────┘
```

### 14.1.2 分布式通信的特点

| 特点 | 说明 |
|-----|------|
| **无中心化** | 不需要 ROS Master |
| **自动发现** | 节点自动发现网络上的其他节点 |
| **跨平台** | 不同操作系统间通信 |
| **可靠传输** | 支持多种 QoS 策略 |

## 14.2 ROS_DOMAIN_ID

### 14.2.1 域 ID 概念

`ROS_DOMAIN_ID` 用于隔离不同的 ROS 2 网络。同一域 ID 的节点可以互相通信，不同域 ID 的节点彼此隔离。

```
域 ID 隔离示意图：

ROS_DOMAIN_ID=0          ROS_DOMAIN_ID=1
┌──────────────┐        ┌──────────────┐
│  机器人A      │        │  机器人B      │
│              │        │              │
│  传感器节点   │        │  传感器节点   │
│  控制节点     │        │  控制节点     │
└──────────────┘        └──────────────┘

互不干扰，独立运行
```

### 14.2.2 设置域 ID

**临时设置 (当前终端):**
```bash
export ROS_DOMAIN_ID=10
ros2 run <package> <node>
```

**永久设置 (添加到 ~/.bashrc):**
```bash
echo "export ROS_DOMAIN_ID=10" >> ~/.bashrc
source ~/.bashrc
```

**Launch 文件中设置:**
```python
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable(name='ROS_DOMAIN_ID', value='10'),
        # ... 其他 launch 配置
    ])
```

### 14.2.3 域 ID 分配建议

| 场景 | 推荐域 ID |
|-----|----------|
| 默认单机器人 | 0 |
| 机器人 1 | 1 |
| 机器人 2 | 2 |
| 仿真环境 | 10-20 |
| 测试环境 | 100+ |

## 14.3 网络配置

### 14.3.1 检查网络连接

```bash
# 检查网络接口
ip addr show

# 检查连接状态
ping <other_pc_ip>

# 检查端口 (DDS 使用)
netstat -an | grep 7400
```

### 14.3.2 配置 DDS 发现

**方法 1: 使用环境变量**

```bash
# 指定发现使用的网络接口
export ROS_LOCALHOST_ONLY=0  # 允许非本地通信

# FastDDS 配置
export FASTRTPS_DEFAULT_PROFILES_FILE=<config_file>

# CycloneDDS 配置
export CYCLONEDDS_URI=<config_file>
```

**方法 2: 创建 XML 配置文件**

`CycloneDDS 配置 (cyclonedds.xml):`
```xml
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
    <Domain>
        <General>
            <NetworkInterfaceAddress>eth0</NetworkInterfaceAddress>
            <AllowMulticast>true</AllowMulticast>
        </General>
    </Domain>
</CycloneDDS>
```

`FastDDS 配置 (fastdds.xml):`
```xml
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <participant profile_name="participant_profile" is_default_profile="true">
        <rtps>
            <builtin>
                <discovery_config>
                    <discoveryProtocol>SIMPLE</discoveryProtocol>
                    <simpleEDP>
                        <PUBWRITER_SUBREADER>true</PUBWRITER_SUBREADER>
                        <PUBREADER_SUBWRITER>true</PUBREADER_SUBWRITER>
                    </simpleEDP>
                </discovery_config>
            </builtin>
        </rtps>
    </participant>
</profiles>
```

### 14.3.3 使用配置文件

```bash
# CycloneDDS
export CYCLONEDDS_URI=file:///path/to/cyclonedds.xml

# FastDDS
export FASTRTPS_DEFAULT_PROFILES_FILE=/path/to/fastdds.xml
```

## 14.4 多机器人配置

### 14.4.1 配置方案 A: 不同域 ID

```bash
# 机器人 1 (PC1)
export ROS_DOMAIN_ID=1
ros2 launch robot_bringup robot_launch.py

# 机器人 2 (PC2)
export ROS_DOMAIN_ID=2
ros2 launch robot_bringup robot_launch.py
```

### 14.4.2 配置方案 B: 命名空间

```python
# 机器人 1 Launch
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_driver',
            executable='driver_node',
            namespace='robot1',
            # ... 其他配置
        )
    ])

# 机器人 2 Launch
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_driver',
            executable='driver_node',
            namespace='robot2',
            # ... 其他配置
        )
    ])
```

### 14.4.3 混合配置

```bash
# 同一域 ID，使用命名空间区分
export ROS_DOMAIN_ID=10

# 机器人 1
ros2 launch robot_bringup robot_launch.py namespace:=robot1

# 机器人 2
ros2 launch robot_bringup robot_launch.py namespace:=robot2
```

## 14.5 跨子网通信

### 14.5.1 配置静态发现

当节点不在同一子网时，需要配置静态发现：

`cyclonedds.xml` (静态配置):
```xml
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config">
    <Domain>
        <General>
            <NetworkInterfaceAddress>eth0</NetworkInterfaceAddress>
            <AllowMulticast>false</AllowMulticast>
        </General>
        <Discovery>
            <ParticipantIndex>auto</ParticipantIndex>
            <Peers>
                <Peer address="192.168.1.100"/>
                <Peer address="192.168.1.101"/>
            </Peers>
        </Discovery>
    </Domain>
</CycloneDDS>
```

### 14.5.2 使用 SIP（Server Info Protocol）

某些 DDS 实现支持 SIP 进行跨子网发现：

```bash
# 设置发现服务器
export ROS_DISCOVERY_SERVER=192.168.1.1:11811
```

## 14.6 防火墙配置

### 14.6.1 DDS 端口

DDS 使用以下端口范围：

| 用途 | 端口范围 |
|-----|---------|
| 发现 | 7400-7400 |
| 数据传输 | 动态分配 |
| 组播 | 动态分配 |

### 14.6.2 UFW 配置

```bash
# 允许 ROS 2 通信
sudo ufw allow 7400/tcp
sudo ufw allow 7400/udp

# 允许组播
sudo ufw allow from 224.0.0.0/4

# 允许特定端口范围
sudo ufw allow 7400:7420/tcp
sudo ufw allow 7400:7420/udp
```

### 14.6.3 firewalld 配置

```bash
# 添加 ROS 2 服务
sudo firewall-cmd --permanent --add-port=7400/tcp
sudo firewall-cmd --permanent --add-port=7400/udp

# 重载防火墙
sudo firewall-cmd --reload
```

## 14.7 故障排查

### 14.7.1 节点无法互相发现

```bash
# 检查域 ID
echo $ROS_DOMAIN_ID

# 检查节点列表
ros2 node list

# 检查网络连接
ping <other_pc>

# 检查 DDS 配置
ros2 doctor --report

# 使用详细模式
RCUTILS_LOGGING_SEVERITY=DEBUG ros2 run <pkg> <node>
```

### 14.7.2 通信延迟

```bash
# 检查网络延迟
ping -c 100 <other_pc>

# 检查丢包
ros2 topic hz /topic_name

# 使用网络诊断工具
ros2 run tf2_ros tf2_monitor
```

### 14.7.3 组播问题

```bash
# 检查组播支持
ip maddr show

# 禁用组播（某些网络环境）
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
```

## 14.8 最佳实践

### 14.8.1 网络设计

| 原则 | 说明 |
|-----|------|
| **专用网络** | 机器人通信使用独立网络 |
| **有线连接** | 关键节点使用有线连接 |
| **域 ID 规划** | 预先规划域 ID 分配 |
| **防火墙配置** | 正确配置防火墙规则 |

### 14.8.2 命名策略

```
推荐的多机器人命名：

/robot1/camera/image_raw
/robot1/lidar/scan
/robot1/cmd_vel

/robot2/camera/image_raw
/robot2/lidar/scan
/robot2/cmd_vel
```

## 14.9 下一步

1. **[15 DDS](./15-dds.md)** - 深入学习 DDS 中间件
2. **[16 Time APIs](./16-time-apis.md)** - 学习时间 API

