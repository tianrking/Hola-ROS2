# 15 DDS (Data Distribution Service)

## 15.1 DDS 概述

### 15.1.1 什么是 DDS

**DDS (Data Distribution Service)** 是以数据为中心的发布-订阅中间件标准，ROS 2 使用 DDS 实现底层通信。

```
ROS 2 与 DDS 的关系：

┌─────────────────────────────────────────────────┐
│                 ROS 2 应用层                    │
│  (节点、话题、服务、动作)                        │
└─────────────────────────────────────────────────┘
                        ▲
                        │
┌─────────────────────────────────────────────────┐
│              RMW (ROS Middleware)                │
│           统一接口层                             │
└─────────────────────────────────────────────────┘
                        ▲
                        │
        ┌───────────────┼───────────────┐
        ▼               ▼               ▼
┌──────────────┐ ┌──────────────┐ ┌──────────────┐
│CycloneDDS    │ │  FastDDS     │ │RTI Connext   │
│              │ │              │ │              │
└──────────────┘ └──────────────┘ └──────────────┘
```

### 15.1.2 DDS 的核心功能

| 功能 | 说明 |
|-----|------|
| **发现机制** | 自动发现网络上的 DDS 参与者 |
| **发布/订阅** | 解耦的数据传输模式 |
| **QoS 策略** | 可配置的服务质量 |
| **类型系统** | 强类型数据定义 |
| **零拷贝** | 高效的数据传输 |

## 15.2 RMW 实现

### 15.2.1 可用的 RMW 实现

| RMW 实现 | DDS 后端 | 许可证 | 特点 |
|---------|---------|-------|------|
| `rmw_cyclonedds_cpp` | CycloneDDS | 开源 | 默认，轻量 |
| `rmw_fastrtps_cpp` | FastDDS | 开源 | 功能丰富 |
| `rmw_connext_cpp` | RTI Connext | 商业 | 工业级支持 |
| `rmw_gurumdds` | GurumDDS | 商业/免费 | 嵌入式优化 |

### 15.2.2 查看当前 RMW

```bash
# 查看当前 RMW 实现
echo $RMW_IMPLEMENTATION

# 查看所有可用的 RMW
ros2 doctor --report | grep rmw
```

### 15.2.3 切换 RMW 实现

**安装 FastDDS:**
```bash
sudo apt install ros-humble-rmw-fastrtps-cpp
```

**切换到 FastDDS:**
```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

**永久设置 (~/.bashrc):**
```bash
echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> ~/.bashrc
```

## 15.3 CycloneDDS

### 15.3.1 CycloneDDS 简介

CycloneDDS 是 ROS 2 Humble 的默认 DDS 实现：
- 高性能、低延迟
- 内存占用小
- 良好的标准兼容性
- 适合嵌入式系统

### 15.3.2 配置文件

`cyclonedds.xml` 配置示例：

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance">
    <Domain>
        <General>
            <!-- 网络接口 -->
            <NetworkInterfaceAddress>auto</NetworkInterfaceAddress>
            <!-- 允许组播 -->
            <AllowMulticast>true</AllowMulticast>
            <!-- 最大消息大小 (字节) -->
            <MaxMessageSize>65536B</MaxMessageSize>
            <!-- 分片大小 -->
            <FragmentSize>8192B</FragmentSize>
        </General>

        <!-- 发现配置 -->
        <Discovery>
            <ParticipantIndex>auto</ParticipantIndex>
            <!-- 发现超时 (毫秒) -->
            <ParticipantIndex>auto</ParticipantIndex>
            <DSDiscovery>
                <!-- 发现对等点 -->
                <ParticipantIndex>auto</ParticipantIndex>
            </DSDiscovery>
        </Discovery>

        <!-- 传输配置 -->
        <Internal>
            <!-- 缓冲区大小 -->
            <SocketRecvBufferSize>2097152</SocketRecvBufferSize>
            <SocketSendBufferSize>2097152</SocketSendBufferSize>
        </Internal>
    </Domain>
</CycloneDDS>
```

### 15.3.3 使用配置文件

```bash
export CYCLONEDDS_URI=file:///path/to/cyclonedds.xml
```

## 15.4 FastDDS

### 15.4.1 FastDDS 简介

FastDDS (前身为 FastRTPS) 是 Epic Games 维护的开源 DDS 实现：
- 功能最丰富
- 性能优秀
- 活跃的开发和维护
- 支持共享内存传输

### 15.4.2 FastDDS 配置

`fastdds.xml` 配置示例：

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <participant profile_name="participant_profile" is_default_profile="true">
        <rtps>
            <!-- 发现配置 -->
            <builtin>
                <discovery_config>
                    <discoveryProtocol>SIMPLE</discoveryProtocol>
                    <initialPeersList>
                        <locator>
                            <udpv4>
                                <address>192.168.1.100</address>
                            </udpv4>
                        </locator>
                    </initialPeersList>
                </discovery_config>
            </builtin>

            <!-- 用户传输 -->
            <userTransports>
                <transportType>UDPv4</transportType>
            </userTransports>

            <!-- 共享内存 -->
            <useBuiltinTransports>false</useBuiltinTransports>
        </rtps>
    </participant>
</profiles>
```

### 15.4.3 共享内存传输

```xml
<participant profile_name="shm_profile" is_default_profile="true">
    <rtps>
        <userTransports>
            <transportType>SHM</transportType>
        </userTransports>
        <useBuiltinTransports>false</useBuiltinTransports>
    </rtps>
</participant>
```

## 15.5 QoS (Quality of Service)

### 15.5.1 QoS 策略

| QoS 策略 | 说明 | 值 |
|---------|------|-----|
| **Reliability** | 可靠性 | RELIABLE, BEST_EFFORT |
| **Durability** | 持久性 | VOLATILE, TRANSIENT_LOCAL |
| **History** | 历史记录 | KEEP_LAST, KEEP_ALL |
| **Depth** | 队列深度 | 整数值 |
| **Deadline** | 截止期限 | Duration |
| **Lifespan** | 生命周期 | Duration |
| **Liveliness** | 活跃性 | AUTOMATIC, MANUAL_BY_TOPIC |

### 15.5.2 QoS 兼容性

```
QoS 兼容性规则：

发布者 QoS          订阅者 QoS         结果
─────────          ──────────         ────
RELIABLE     +     RELIABLE      =   兼容
RELIABLE     +     BEST_EFFORT   =   不兼容
BEST_EFFORT  +     BEST_EFFORT   =   兼容
```

### 15.5.3 在代码中使用 QoS

**C++:**
```cpp
// 传感器数据 QoS
auto qos = rclcpp::SensorDataQoS();
publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
    "scan", qos);

// 自定义 QoS
auto custom_qos = rclcpp::QoS(rclcpp::KeepLast(10))
    .reliable()
    .durability_volatile();

publisher_ = this->create_publisher<std_msgs::msg::String>(
    "chatter", custom_qos);
```

**Python:**
```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# 自定义 QoS
custom_qos = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE
)

self.publisher = self.create_publisher(String, 'chatter', custom_qos)
```

## 15.6 DDS 调试工具

### 15.6.1 监控 DDS 流量

```bash
# 使用 Wireshark
wireshark

# 过滤 DDS 流量
dds || udp.port == 7400
```

### 15.6.2 XML 调试

```bash
# 验证 XML 配置
xmllint --noout cyclonedds.xml
```

### 15.6.3 FastDDS 监控工具

```bash
# FastDDS 提供的监控工具
ros2 run fastdds fastdds-monitor
```

## 15.7 性能优化

### 15.7.1 减少延迟

```xml
<!-- CycloneDDS 低延迟配置 -->
<General>
    <!-- 减少缓冲区 -->
    <FragmentSize>1400</FragmentSize>
    <!-- 禁用 Nagle 算法 -->
    <Transport>udp</Transport>
</General>
```

### 15.7.2 增加吞吐量

```xml
<!-- FastDDS 高吞吐量配置 -->
<rtps>
    <sendBufferSize>8388608</sendBufferSize>
    <receiveBufferSize>8388608</receiveBufferSize>
</rtps>
```

### 15.7.3 使用共享内存

```python
# 本地通信使用共享内存（低延迟）
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=shm_config.xml
```

## 15.8 最佳实践

### 15.8.1 RMW 选择

| 场景 | 推荐 RMW |
|-----|---------|
| 通用应用 | `rmw_cyclonedds_cpp` |
| 需要共享内存 | `rmw_fastrtps_cpp` |
| 工业部署 | `rmw_connext_cpp` |
| 嵌入式系统 | `rmw_cyclonedxs_cpp` |

### 15.8.2 QoS 建议

| 场景 | QoS 配置 |
|-----|---------|
| 传感器数据 | `BEST_EFFORT`, `VOLATILE` |
| 控制指令 | `RELIABLE`, `VOLATILE` |
| 配置参数 | `RELIABLE`, `TRANSIENT_LOCAL` |
| 实时图像 | `BEST_EFFORT`, `VOLATILE` |

## 15.9 下一步

1. **[16 Time APIs](./16-time-apis.md)** - 学习时间 API
2. **[17 CLI Tools](./17-cli-tools.md)** - 学习命令行工具

