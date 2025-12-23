# 07 话题通讯 (Topics)

## 7.1 话题通信概述

### 7.1.1 什么是话题通信

**话题 (Topic)** 是 ROS 2 中节点间进行异步流式通信的机制，采用发布/订阅（Pub/Sub）模式。发布者节点向话题发布消息，订阅者节点从话题接收消息，两者无需知道对方的存在。

```
话题通信模型：

                    ┌─────────────────────┐
                    │     话题: /cmd_vel   │
                    │   消息类型: Twist    │
                    └─────────────────────┘
                              │
        ┌─────────────────────┼─────────────────────┐
        │                     │                     │
   [发布者1]             [发布者2]             [订阅者1]
  navigation            teleop               robot_base
        │                     │                     │
        └─────────────────────┴─────────────────────┘
                              │
                        [订阅者2]
                     velocity_monitor
```

### 7.1.2 话题通信特点

| 特点 | 描述 | 适用场景 |
|-----|------|---------|
| **异步通信** | 发布者不等待订阅者响应 | 传感器数据流 |
| **多对多** | 多个发布者和订阅者 | 数据广播 |
| **弱耦合** | 发布者与订阅者独立 | 模块化设计 |
| **流式传输** | 持续的数据流 | 持续监控 |

### 7.1.3 话题命名规则

| 规则 | 说明 |
|-----|------|
| 必须以 `/` 开头（全局命名空间）或相对名称 |
| 使用小写字母、数字和下划线 |
| 使用 `/` 分隔命名空间层级 |
| 避免使用保留名称 |

**命名示例：**

| 话题名 | 评价 |
|-------|------|
| `/cmd_vel` | 标准，推荐 |
| `/camera/image_raw` | 层级清晰，推荐 |
| `/sensor/front_camera/image` | 带命名空间，推荐 |
| `/MyTopic` | 不推荐（大写） |
| `cmd_vel` | 相对名称（会被加上节点命名空间） |

## 7.2 创建发布者 (Publisher)

### 7.2.1 C++ 发布者

```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class PublisherNode : public rclcpp::Node {
public:
    PublisherNode() : Node("publisher_node") {
        // 创建发布者
        // 参数1: 消息类型
        // 参数2: 话题名称
        // 参数3: QoS 队列大小
        publisher_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);

        // 创建定时器，定期发布消息
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&PublisherNode::timer_callback, this)
        );

        RCLCPP_INFO(this->get_logger(), "Publisher node initialized");
    }

private:
    void timer_callback() {
        auto message = std_msgs::msg::String();
        message.data = "Hello, ROS 2! Count: " + std::to_string(count_++);

        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());

        // 发布消息
        publisher_->publish(message);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_ = 0;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

### 7.2.2 Python 发布者

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')

        # 创建发布者
        self.publisher_ = self.create_publisher(String, 'chatter', 10)

        # 创建定时器
        timer_period = 0.5  # 秒
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.count = 0

        self.get_logger().info('Publisher node initialized')

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello, ROS 2! Count: {self.count}'

        self.get_logger().info(f"Publishing: '{msg.data}'")

        # 发布消息
        self.publisher_.publish(msg)

        self.count += 1


def main(args=None):
    rclpy.init(args=args)
    node = PublisherNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## 7.3 创建订阅者 (Subscriber)

### 7.3.1 C++ 订阅者

```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class SubscriberNode : public rclcpp::Node {
public:
    SubscriberNode() : Node("subscriber_node") {
        // 创建订阅者
        // 参数1: 话题名称
        // 参数2: QoS 队列大小
        // 参数3: 回调函数
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "chatter",
            10,
            std::bind(&SubscriberNode::topic_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Subscriber node initialized");
    }

private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const {
        RCLCPP_INFO(this->get_logger(), "Received: '%s'", msg->data.c_str());
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SubscriberNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

### 7.3.2 Python 订阅者

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber_node')

        # 创建订阅者
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.topic_callback,
            10
        )

        self.get_logger().info('Subscriber node initialized')

    def topic_callback(self, msg):
        self.get_logger().info(f"Received: '{msg.data}'")


def main(args=None):
    rclpy.init(args=args)
    node = SubscriberNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## 7.4 常用消息类型

### 7.4.1 标准消息

**String 消息：**

```cpp
#include "std_msgs/msg/string.hpp"

std_msgs::msg::String msg;
msg.data = "Hello ROS 2";
```

**Bool 消息：**

```cpp
#include "std_msgs/msg/bool.hpp"

std_msgs::msg::Bool msg;
msg.data = true;
```

**Int/Float 消息：**

```cpp
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float64.hpp"

std_msgs::msg::Int32 int_msg;
int_msg.data = 42;

std_msgs::msg::Float64 float_msg;
float_msg.data = 3.14159;
```

### 7.4.2 几何消息

**Twist (速度指令)：**

```cpp
#include "geometry_msgs/msg/twist.hpp"

geometry_msgs::msg::Twist twist;
twist.linear.x = 0.5;    // 前进速度 (m/s)
twist.linear.y = 0.0;
twist.linear.z = 0.0;
twist.angular.x = 0.0;
twist.angular.y = 0.0;
twist.angular.z = 0.3;   // 旋转速度 (rad/s)
```

**Pose (位姿)：**

```cpp
#include "geometry_msgs/msg/pose.hpp"

geometry_msgs::msg::Pose pose;
pose.position.x = 1.0;
pose.position.y = 2.0;
pose.position.z = 0.0;

pose.orientation.x = 0.0;
pose.orientation.y = 0.0;
pose.orientation.z = 0.0;
pose.orientation.w = 1.0;  // 四元数
```

### 7.4.3 传感器消息

**LaserScan (激光雷达)：**

```cpp
#include "sensor_msgs/msg/laser_scan.hpp"

sensor_msgs::msg::LaserScan scan;
scan.header.stamp = this->get_clock()->now();
scan.header.frame_id = "laser_frame";
scan.angle_min = -M_PI;
scan.angle_max = M_PI;
scan.angle_increment = M_PI / 180.0;
scan.range_min = 0.1;
scan.range_max = 10.0;
scan.ranges.resize(360);  // 360 个点
```

**Image (图像)：**

```cpp
#include "sensor_msgs/msg/image.hpp"

sensor_msgs::msg::Image image;
image.header.stamp = this->get_clock()->now();
image.header.frame_id = "camera_frame";
image.height = 480;
image.width = 640;
image.encoding = "bgr8";
image.is_bigendian = false;
image.step = 640 * 3;  // width * channels
image.data.resize(480 * 640 * 3);  // height * width * channels
```

## 7.5 QoS (Quality of Service) 策略

### 7.5.1 QoS 概述

QoS 控制话题通信的质量，包括可靠性、持久性、历史记录等。

```
QoS 匹配规则：

发布者 QoS ──匹配──► 订阅者 QoS
                   │
                   │ 必须兼容
                   ▼
              通信成功
```

### 7.5.2 QoS 策略详解

**Reliability (可靠性)：**

| 策略 | 说明 | 适用场景 |
|-----|------|---------|
| `RELIABLE` | 保证消息传递 | 控制指令 |
| `BEST_EFFORT` | 尽力传递 | 传感器数据流 |

```cpp
// C++ QoS 设置
auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

auto best_effort_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
```

**Durability (持久性)：**

| 策略 | 说明 |
|-----|------|
| `VOLATILE` | 不保留历史消息 |
| `TRANSIENT_LOCAL` | 保留最后一条消息 |

```cpp
// 新订阅者能收到最后一条消息
auto transient_local_qos = rclcpp::QoS(rclcpp::KeepLast(1))
    .reliable()
    .transient_local();
```

**History (历史记录)：**

| 策略 | 说明 |
|-----|------|
| `KEEP_LAST` | 保留最近 N 条消息 |
| `KEEP_ALL` | 保留所有消息 |

```cpp
auto qos = rclcpp::QoS(rclcpp::KeepLast(10));  // 保留最近 10 条
auto qos_all = rclcpp::QoS(rclcpp::KeepAll());  // 保留所有
```

**Depth (深度)：**

队列大小，与 `KEEP_LAST` 配合使用：

```cpp
auto qos = rclcpp::QoS(rclcpp::KeepLast(10));  // 队列深度为 10
```

### 7.5.3 预定义 QoS 配置

ROS 2 提供预定义的 QoS 配置：

| 配置 | 说明 |
|-----|------|
| `rmw_qos_profile_sensor_data` | 传感器数据（BEST_EFFORT） |
| `rmw_qos_profile_default` | 默认配置（RELIABLE） |
| `rmw_qos_profile_services_default` | 服务默认配置 |
| `rmw_qos_profile_parameters` | 参数配置 |

```cpp
// 使用预定义 QoS
auto qos = rclcpp::SensorDataQoS();  // 传感器数据
auto qos = rclcpp::ParametersQoS();  // 参数数据
auto qos = rclcpp::ServicesDefaultQoS();  // 服务数据
```

### 7.5.4 Python QoS 设置

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.qos import HistoryPolicy

# 传感器数据 QoS
sensor_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)

# 创建订阅者时使用
self.subscription = self.create_subscription(
    LaserScan,
    '/scan',
    self.scan_callback,
    sensor_qos
)

# 使用预定义 QoS
from rclpy.qos import sensor_data_qos
self.subscription = self.create_subscription(
    LaserScan,
    '/scan',
    self.scan_callback,
    sensor_data_qos
)
```

## 7.6 话题命令行工具

### 7.6.1 列出话题

```bash
# 列出所有话题
ros2 topic list

# 列出话题及其类型
ros2 topic list -t

# 仅列出特定话题
ros2 topic list | grep /cmd_vel
```

### 7.6.2 查看话题信息

```bash
# 查看话题详细信息
ros2 topic info /cmd_vel

# 查看话题详细信息（包括 QoS）
ros2 topic info /cmd_vel -v
```

**输出示例：**
```
Topic: /cmd_vel
Type: geometry_msgs/msg/Twist
Publisher count: 2
Subscription count: 1
```

### 7.6.3 回显话题数据

```bash
# 回显话题消息
ros2 topic echo /chatter

# 回显带时间戳
ros2 topic echo /chatter --field header.stamp

# 回显指定字段
ros2 topic echo /cmd_vel --field linear
```

### 7.6.4 发布消息到话题

```bash
# 发布简单消息
ros2 topic pub /chatter std_msgs/msg/String "{data: 'Hello'}"

# 持续发布
ros2 topic pub --rate 10 /chatter std_msgs/msg/String "{data: 'Hello'}"

# 发布速度指令
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.3}}"

# 发布一次
ros2 topic pub --once /chatter std_msgs/msg/String "{data: 'Once'}"
```

### 7.6.5 查看话题频率

```bash
# 计算话题发布频率
ros2 topic hz /chatter

# 窗口大小
ros2 topic hz /chatter --window 50
```

### 7.6.6 查看话题带宽

```bash
# 查看话题带宽使用
ros2 topic bw /camera/image_raw
```

## 7.7 最佳实践

### 7.7.1 话题设计原则

| 原则 | 说明 |
|-----|------|
| **语义化命名** | 话题名应清晰描述其内容 |
| **合适的数据率** | 不要过度发布数据 |
| **选择正确的 QoS** | 根据应用场景选择 |
| **消息大小适中** | 避免过大的消息 |

### 7.7.2 命名规范

```
推荐的话题命名：

/cmd_vel                    # 速度控制
/odom                        # 里程计
/scan                        # 激光雷达数据
/camera/image_raw            # 原始图像
/camera/image/compressed     # 压缩图像
/sensor/imu                  # IMU 数据
/joint_states                # 关节状态
/tf                          # TF 变换
/tf_static                   # 静态 TF
```

### 7.7.3 发布者模式

```cpp
// 发布者应该使用定时器控制发布频率
class TimedPublisher : public rclcpp::Node {
public:
    TimedPublisher() : Node("timed_publisher") {
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);

        // 10 Hz 发布
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            [this]() {
                auto msg = std_msgs::msg::String();
                msg.data = "Regular message";
                publisher_->publish(msg);
            }
        );
    }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};
```

## 7.8 常见问题

### 7.8.1 订阅者收不到消息

**问题：** 订阅者启动但无法接收消息

**排查步骤：**

```bash
# 1. 检查话题是否匹配
ros2 topic list

# 2. 检查话题类型
ros2 topic info /topic_name -v

# 3. 检查 QoS 兼容性
ros2 topic info /topic_name -v

# 4. 验证消息是否在发布
ros2 topic echo /topic_name

# 5. 检查节点连接
ros2 node info /publisher_node
ros2 node info /subscriber_node
```

### 7.8.2 QoS 不匹配

**问题：** QoS 配置不兼容导致通信失败

**解决方法：**

```cpp
// 确保发布者和订阅者的 QoS 兼容
// 发布者
auto pub_qos = rclcpp::SensorDataQoS();
publisher_ = this->create_publisher<MsgType>("topic", pub_qos);

// 订阅者
auto sub_qos = rclcpp::SensorDataQoS();
subscription_ = this->create_subscription<MsgType>(
    "topic", sub_qos, callback);
```

### 7.8.3 消息延迟

**问题：** 消息接收延迟过大

**排查与解决：**

```bash
# 检查话题频率
ros2 topic hz /topic_name

# 检查网络延迟
ros2 doctor --report

# 解决方案：
# 1. 使用 BEST_EFFORT QoS
# 2. 减少队列深度
# 3. 优化回调函数处理时间
```

## 7.9 下一步

学习话题通信后，您可以：

1. **[08 Services](./08-services.md)** - 学习服务通信
2. **[09 Actions](./09-actions.md)** - 学习动作通信

