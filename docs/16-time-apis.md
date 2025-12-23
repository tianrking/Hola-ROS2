# 16 时间相关 API (Time APIs)

## 16.1 时间概念

### 16.1.1 ROS 时间类型

| 类型 | 说明 | 用途 |
|-----|------|------|
| `RCL_SYSTEM_TIME` | 系统时钟时间 | 一般时间计算 |
| `RCL_ROS_TIME` | ROS 模拟时间 | 仿真环境 |
| `RCL_STEADY_TIME` | 单调递增时间 | 定时器、超时 |

### 16.1.2 时间表示

ROS 2 使用两种时间表示：

**C++ (builtin_interfaces/msg/Time):**
```cpp
int32 sec;    // 秒
uint32 nanosec;  // 纳秒
```

**Python:**
```python
# 时间通常是 float 类型（秒）或 Time 对象
time = node.get_clock().now()
seconds = time.nanoseconds / 1e9
```

## 16.2 获取当前时间

### 16.2.1 C++ 获取时间

```cpp
#include "rclcpp/rclcpp.hpp"

class TimeNode : public rclcpp::Node {
public:
    TimeNode() : Node("time_node") {
        // 获取当前时间
        auto now = this->get_clock()->now();
        RCLCPP_INFO(this->get_logger(), "Current time: %d.%09d",
                    now.seconds(), now.nanoseconds());

        // 获取系统时间
        auto system_now = this->now();
        auto steady_now = this->get_clock()->steady_time();

        // 使用 Wall 时钟（系统时间，不受模拟时间影响）
        auto wall_now = this->get_clock()->now();
    }
};
```

### 16.2.2 Python 获取时间

```python
import rclpy
from rclpy.node import Node


class TimeNode(Node):
    def __init__(self):
        super().__init__('time_node')

        # 获取当前时间
        now = self.get_clock().now()
        self.get_logger().info(f'Current time: {now.nanoseconds}')

        # 转换为秒
        seconds = now.nanoseconds / 1e9
        self.get_logger().info(f'Seconds: {seconds}')


def main():
    rclpy.init()
    node = TimeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## 16.3 Duration (时间间隔)

### 16.3.1 C++ Duration

```cpp
#include "rclcpp/rclcpp.hpp"

void duration_examples() {
    // 创建 Duration
    rclcpp::Duration d1(5, 0);  // 5 秒
    rclcpp::Duration d2(0, 1000000000);  // 1 秒 (纳秒)
    rclcpp::Duration d3(1.5);  // 1.5 秒

    // Duration 运算
    auto sum = d1 + d2;
    auto diff = d1 - d2;
    auto scaled = d1 * 2.0;

    // 比较
    if (d1 > d2) {
        // d1 更长
    }

    // 转换
    double seconds = d1.seconds();
    auto nanoseconds = d1.nanoseconds();
}
```

### 16.3.2 Python Duration

```python
from rclpy.duration import Duration

# 创建 Duration
d1 = Duration(seconds=5.0)
d2 = Duration(nanoseconds=1_000_000_000)  # 1 秒

# 运算
sum_duration = d1 + d2
diff_duration = d1 - d2
scaled = d1 * 2.0

# 比较
if d1 > d2:
    pass  # d1 更长

# 转换
seconds = d1.nanoseconds / 1e9
```

## 16.4 Rate (频率控制)

### 16.4.1 C++ Rate

```cpp
#include "rclcpp/rclcpp.hpp"

void rate_example() {
    rclcpp::init(0, nullptr);
    auto node = std::make_shared<rclcpp::Node>("rate_node");

    // 创建 10 Hz 的 Rate
    rclcpp::Rate rate(10.0);  // 10 Hz

    int count = 0;
    while (rclcpp::ok()) {
        RCLCPP_INFO(node->get_logger(), "Count: %d", count++);

        // 按频率睡眠
        rate.sleep();
    }

    rclcpp::shutdown();
}
```

### 16.4.2 Python Rate

```python
import rclpy
from rclpy.node import Node


class RateNode(Node):
    def __init__(self):
        super().__init__('rate_node')
        self.count = 0

        # 创建定时器 (推荐方式)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz

    def timer_callback(self):
        self.get_logger().info(f'Count: {self.count}')
        self.count += 1


def main():
    rclpy.init()
    node = RateNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## 16.5 Timer (定时器)

### 16.5.1 C++ Timer

```cpp
#include "rclcpp/rclcpp.hpp"

class TimerNode : public rclcpp::Node {
public:
    TimerNode() : Node("timer_node"), count_(0) {
        // 创建定时器
        // 参数: 周期, 回调函数
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),  // 100ms
            std::bind(&TimerNode::timer_callback, this)
        );

        // 使用 steady 时间
        steady_timer_ = this->create_timer(
            std::chrono::seconds(1),
            std::bind(&TimerNode::steady_callback, this)
        );
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr steady_timer_;
    int count_;

    void timer_callback() {
        RCLCPP_INFO(this->get_logger(), "Timer callback: %d", count_++);
    }

    void steady_callback() {
        RCLCPP_INFO(this->get_logger(), "Steady timer");
    }
};
```

### 16.5.2 Python Timer

```python
import rclpy
from rclpy.node import Node


class TimerNode(Node):
    def __init__(self):
        super().__init__('timer_node')
        self.count = 0

        # 创建定时器 (秒)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.steady_timer = self.create_timer(1.0, self.steady_callback)

    def timer_callback(self):
        self.get_logger().info(f'Timer callback: {self.count}')
        self.count += 1

    def steady_callback(self):
        self.get_logger().info('Steady timer')


def main():
    rclpy.init()
    node = TimerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## 16.6 时间戳

### 16.6.1 消息时间戳

```cpp
#include "std_msgs/msg/header.hpp"

void timestamp_example() {
    std_msgs::msg::Header header;

    // 设置时间戳为当前时间
    header.stamp = rclcpp::Clock().now();

    // 设置帧 ID
    header.frame_id = "base_link";
}
```

```python
from std_msgs.msg import Header

header = Header()
header.stamp = self.get_clock().now().to_msg()
header.frame_id = 'base_link'
```

### 16.6.2 消息中的时间戳

```cpp
#include "sensor_msgs/msg/image.hpp"

void image_with_timestamp(rclcpp::Node* node) {
    auto msg = sensor_msgs::msg::Image();

    // 设置时间戳
    msg.header.stamp = node->get_clock()->now();
    msg.header.frame_id = "camera_link";

    // 设置图像数据
    msg.width = 640;
    msg.height = 480;
    msg.encoding = "bgr8";
}
```

## 16.7 时间转换

### 16.7.1 时间单位转换

```cpp
// 秒转 Duration
rclcpp::Duration from_seconds(double seconds) {
    return rclcpp::Duration::from_seconds(seconds);
}

// 纳秒转 Duration
rclcpp::Duration from_nanoseconds(int64_t nanos) {
    return rclcpp::Duration(nanos / 1000000000, nanos % 1000000000);
}

// Duration 转秒
double to_seconds(rclcpp::Duration dur) {
    return dur.seconds();
}
```

```python
# Python 中的时间转换
nanoseconds = 1_500_000_000  # 1.5 秒
seconds = nanoseconds / 1e9

from rclpy.duration import Duration
duration = Duration(seconds=1.5)
nanos = duration.nanoseconds
```

## 16.8 模拟时间

### 16.8.1 使用模拟时间

```bash
# 启动使用模拟时间的节点
ros2 run my_package my_node --ros-args -p use_sim_time:=true
```

### 16.8.2 发布模拟时间

```cpp
#include "rosgraph_msgs/msg/clock.hpp"

class ClockPublisher : public rclcpp::Node {
public:
    ClockPublisher() : Node("clock_publisher") {
        clock_pub_ = this->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            [this]() {
                rosgraph_msgs::msg::Clock clock_msg;
                // 模拟时间加速运行
                clock_msg.clock.sec = simulated_time_;
                clock_pub_->publish(clock_msg);
                simulated_time_++;
            }
        );

        rclcpp::Time simulated_time_ = 0;
    }

private:
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};
```

## 16.9 时间实用工具

### 16.9.1 时间差计算

```cpp
#include "rclcpp/rclcpp.hpp"

void time_difference() {
    auto start = rclcpp::Clock().now();

    // 执行某些操作
    rclcpp::sleep_for(std::chrono::milliseconds(100));

    auto end = rclcpp::Clock().now();

    // 计算时间差
    auto duration = end - start;
    RCLCPP_INFO(rclcpp::get_logger("example"), "Duration: %f seconds",
                duration.seconds());
}
```

### 16.9.2 超时检测

```cpp
#include "rclcpp/rclcpp.hpp"

class TimeoutNode : public rclcpp::Node {
public:
    TimeoutNode() : Node("timeout_node") {
        last_message_time_ = this->get_clock()->now();

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            [this]() {
                auto now = this->get_clock()->now();
                auto elapsed = now - last_message_time_;

                if (elapsed.seconds() > 1.0) {
                    RCLCPP_WARN(this->get_logger(), "Timeout detected!");
                }
            }
        );
    }

private:
    rclcpp::Time last_message_time_;
    rclcpp::TimerBase::SharedPtr timer_;
};
```

## 16.10 最佳实践

| 场景 | 推荐 API |
|-----|---------|
| 周期性任务 | `create_timer()` |
| 控制循环频率 | `create_timer()` |
| 测量时间差 | `Clock::now()` |
| 消息时间戳 | `msg.header.stamp` |
| 超时检测 | 比较 `Clock::now()` |

## 16.11 下一步

1. **[17 CLI Tools](./17-cli-tools.md)** - 学习命令行工具
2. **[18 RViz2](./18-rviz2.md)** - 学习可视化

---
**✅ 16 时间相关 API - 已完成**
