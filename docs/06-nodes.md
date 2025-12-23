# 06 节点 (Nodes)

## 6.1 节点概述

### 6.1.1 什么是节点

**节点 (Node)** 是 ROS 2 中最基本的计算单元。一个节点是一个使用 ROS 2 API 与其他节点通信的进程。每个节点通常负责特定的功能，如读取传感器数据、处理数据、控制执行器等。

```
ROS 2 节点架构图：

┌────────────────────────────────────────────────────────────┐
│                      ROS 2 系统                             │
│                                                            │
│   ┌──────────┐         ┌──────────┐         ┌──────────┐  │
│   │ 传感器   │         │  处理器   │         │ 执行器   │  │
│   │ 节点     │         │ 节点      │         │ 节点      │  │
│   │          │         │          │         │          │  │
│   │ • 读取   │─数据───▶│ • 分析   │─控制───▶│ • 执行   │  │
│   │   数据   │         │ • 决策   │         │   动作   │  │
│   └──────────┘         └──────────┘         └──────────┘  │
│                                                            │
│   ┌──────────┐         ┌──────────┐                       │
│   │ 日志     │         │ 可视化   │                       │
│   │ 节点     │         │ 节点      │                       │
│   └──────────┘         └──────────┘                       │
│                                                            │
└────────────────────────────────────────────────────────────┘
```

### 6.1.2 节点的特点

| 特点 | 说明 |
|-----|------|
| **轻量级** | 一个可执行文件可以包含多个节点 |
| **分布式** | 节点可以运行在不同机器上 |
| **解耦** | 节点间通过接口通信，不直接依赖 |
| **可组合** | 多个节点组合实现复杂功能 |
| **独立生命周期** | 每个节点独立启动和关闭 |

### 6.1.3 节点命名规则

- 节点名必须唯一（同一命名空间内）
- 只能包含字母、数字和下划线
- 区分大小写
- 推荐使用描述性名称

**命名示例：**

| 节点名 | 评价 |
|-------|------|
| `camera_driver` | 优秀 |
| `path_planner` | 优秀 |
| `node1` | 不推荐（不描述性） |
| `my-node` | 无效（含连字符） |

## 6.2 C++ 节点编程

### 6.2.1 最小节点示例

```cpp
// minimal_node.cpp
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
    // 初始化 ROS 2
    rclcpp::init(argc, argv);

    // 创建节点
    auto node = rclcpp::Node::make_shared("minimal_node");

    // 输出日志
    RCLCPP_INFO(node->get_logger(), "Minimal node has been started.");

    // 运行节点
    rclcpp::spin(node);

    // 清理
    rclcpp::shutdown();
    return 0;
}
```

### 6.2.2 面向对象节点

推荐使用面向对象方式创建节点：

```cpp
// my_node.hpp
#ifndef MY_NODE_HPP
#define MY_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class MyNode : public rclcpp::Node {
public:
    MyNode();

private:
    void timer_callback();
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

#endif  // MY_NODE_HPP
```

```cpp
// my_node.cpp
#include "my_node.hpp"

MyNode::MyNode() : Node("my_node"), count_(0) {
    // 创建发布者
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);

    // 创建定时器
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&MyNode::timer_callback, this)
    );

    RCLCPP_INFO(this->get_logger(), "MyNode has been initialized.");
}

void MyNode::timer_callback() {
    auto message = std_msgs::msg::String();
    message.data = "Hello, ROS 2! Count: " + std::to_string(count_++);

    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
}
```

```cpp
// main.cpp
#include "my_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

### 6.2.3 节点组件化（Component）

创建组件节点（用于节点容器）：

```cpp
#include <rclcpp/rclcpp.hpp>
#include <memory>

class MyComponent : public rclcpp::Node {
public:
    MyComponent(const rclcpp::NodeOptions & options)
        : Node("my_component", options) {

        RCLCPP_INFO(this->get_logger(), "MyComponent initialized");

        // 创建发布者、订阅者等
        publisher_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);
    }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(MyComponent)
```

**CMakeLists.txt 配置：**

```cmake
# 组件节点需要特殊配置
add_library(my_component SHARED src/my_component.cpp)
target_compile_definitions(my_component PRIVATE "MY_COMPONENT_BUILDING_DLL")

ament_target_dependencies(my_component
    rclcpp
    rclcpp_components
    std_msgs
)

rclcpp_components_register_nodes(my_component "MyComponent")

install(TARGETS my_component
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)
```

## 6.3 Python 节点编程

### 6.3.1 最小节点示例

```python
#!/usr/bin/env python3
# minimal_node.py

import rclpy
from rclpy.node import Node

def main(args=None):
    rclpy.init(args=args)

    node = Node('minimal_node')
    node.get_logger().info('Minimal node has been started.')

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 6.3.2 面向对象节点

```python
#!/usr/bin/env python3
# my_node.py

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String


class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')

        # 创建发布者
        self.publisher_ = self.create_publisher(String, 'topic', 10)

        # 创建定时器
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.count = 0

        self.get_logger().info('MyNode has been initialized.')

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello, ROS 2! Count: {self.count}'

        self.get_logger().info(f"Publishing: '{msg.data}'")
        self.publisher_.publish(msg)

        self.count += 1


def main(args=None):
    rclpy.init(args=args)

    node = MyNode()

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

### 6.3.3 多节点执行器

在一个进程中运行多个节点：

```python
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor


class Node1(Node):
    def __init__(self):
        super().__init__('node1')
        self.get_logger().info('Node1 initialized')


class Node2(Node):
    def __init__(self):
        super().__init__('node2')
        self.get_logger().info('Node2 initialized')


def main():
    rclpy.init()

    node1 = Node1()
    node2 = Node2()

    # 使用多线程执行器
    executor = MultiThreadedExecutor()
    executor.add_node(node1)
    executor.add_node(node2)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node1.destroy_node()
        node2.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## 6.4 节点生命周期管理

### 6.4.1 节点生命周期状态

```
┌─────────────────────────────────────────────────────────┐
│              节点生命周期状态机                            │
├─────────────────────────────────────────────────────────┤
│                                                         │
│   ┌──────────┐    configure    ┌──────────┐           │
│   │ Unconfigured │ ────────────▶│Inactive  │           │
│   └──────────┘                 └──────────┘           │
│        ▲                            │                  │
│        │ cleanup                   │ activate          │
│        │                            ▼                  │
│   ┌──────────┟───────────────▶  ┌──────────┐          │
│   │Finalized│                   │  Active  │          │
│   └──────────┘                   └──────────┘          │
│                                     │                  │
│                                     │ deactivate       │
│                                     └─────────────────┘│
│                                                         │
└─────────────────────────────────────────────────────────┘
```

### 6.4.2 生命周期节点 (C++)

```cpp
#include <rclcpp/rclcpp.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

class MyLifecycleNode : public rclcpp_lifecycle::LifecycleNode {
public:
    explicit MyLifecycleNode(const std::string & node_name)
        : rclcpp_lifecycle::LifecycleNode(node_name) {

        // 配置回调
        this->register_on_configure(
            std::bind(&MyLifecycleNode::on_configure, this, std::placeholders::_1));

        // 激活回调
        this->register_on_activate(
            std::bind(&MyLifecycleNode::on_activate, this, std::placeholders::_1));

        // 停用回调
        this->register_on_deactivate(
            std::bind(&MyLifecycleNode::on_deactivate, this, std::placeholders::_1));

        // 清理回调
        this->register_on_cleanup(
            std::bind(&MyLifecycleNode::on_cleanup, this, std::placeholders::_1));
    }

private:
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State &) {
        RCLCPP_INFO(get_logger(), "Configuring node...");
        // 初始化资源
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State &) {
        RCLCPP_INFO(get_logger(), "Activating node...");
        // 激活资源
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State &) {
        RCLCPP_INFO(get_logger(), "Deactivating node...");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_cleanup(const rclcpp_lifecycle::State &) {
        RCLCPP_INFO(get_logger(), "Cleaning up node...");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }
};
```

## 6.5 节点配置与参数

### 6.5.1 声明参数

**C++ 参数声明：**

```cpp
class ParameterNode : public rclcpp::Node {
public:
    ParameterNode() : Node("parameter_node") {
        // 声明参数
        this->declare_parameter("frequency", 10.0);
        this->declare_parameter("sensor_name", "camera");
        this->declare_parameter("enabled", true);

        // 获取参数值
        double frequency = this->get_parameter("frequency").as_double();
        std::string sensor_name = this->get_parameter("sensor_name").as_string();
        bool enabled = this->get_parameter("enabled").as_bool();

        // 参数回调
        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&ParameterNode::parameters_callback, this, std::placeholders::_1));
    }

private:
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
        param_callback_handle_;

    rcl_interfaces::msg::SetParametersResult parameters_callback(
        const std::vector<rclcpp::Parameter> & parameters) {

        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto & param : parameters) {
            if (param.get_name() == "frequency") {
                if (param.as_double() <= 0.0) {
                    result.successful = false;
                    result.reason = "Frequency must be positive";
                }
            }
        }

        return result;
    }
};
```

**Python 参数声明：**

```python
class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')

        # 声明参数
        self.declare_parameter('frequency', 10.0)
        self.declare_parameter('sensor_name', 'camera')
        self.declare_parameter('enabled', True)

        # 获取参数值
        frequency = self.get_parameter('frequency').value
        sensor_name = self.get_parameter('sensor_name').value
        enabled = self.get_parameter('enabled').value

        # 添加参数回调
        self.add_on_set_parameters_callback(self.parameter_callback)

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'frequency':
                if param.value <= 0.0:
                    return rcl_interfaces.msg.SetParametersResult(
                        successful=False, reason='Frequency must be positive')
        return rcl_interfaces.msg.SetParametersResult(successful=True)
```

### 6.5.2 从 YAML 文件加载参数

**创建参数文件 `config/params.yaml`：**

```yaml
parameter_node:
  ros__parameters:
    frequency: 20.0
    sensor_name: "lidar"
    enabled: true
    camera_settings:
      width: 640
      height: 480
```

**从 Launch 文件加载：**

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('my_package'),
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=params_file,
            description='Path to parameters file'
        ),
        Node(
            package='my_package',
            executable='parameter_node',
            name='parameter_node',
            parameters=[LaunchConfiguration('params_file')],
            output='screen'
        )
    ])
```

## 6.6 节点调试

### 6.6.1 日志系统

**C++ 日志级别：**

```cpp
RCLCPP_DEBUG(node->get_logger(), "Debug message: %d", value);
RCLCPP_INFO(node->get_logger(), "Info message");
RCLCPP_WARN(node->get_logger(), "Warning message");
RCLCPP_ERROR(node->get_logger(), "Error message");
RCLCPP_FATAL(node->get_logger(), "Fatal message");

// 条件日志
RCLCPP_INFO_ONCE(node->get_logger(), "Printed only once");
RCLCPP_INFO_THROTTLE(node->get_logger(), *clock, 1000, "Printed every 1 second");
```

**Python 日志级别：**

```python
self.get_logger().debug("Debug message")
self.get_logger().info("Info message")
self.get_logger().warning("Warning message")
self.get_logger().error("Error message")
self.get_logger().fatal("Fatal message")
```

### 6.6.2 控制日志级别

```bash
# 设置所有节点的日志级别
ros2 run my_package my_node --ros-args --log-level debug

# 设置特定节点的日志级别
ros2 run my_package my_node --ros-args --log-level my_node:=debug

# 使用外部日志配置
export RCUTILS_LOGGING_SEVERITY=DEBUG
ros2 run my_package my_node
```

### 6.6.3 使用 GDB 调试

```bash
# 使用 GDB 启动节点
ros2 run --prefix 'gdb -ex run --args' my_package my_node

# 或直接使用 GDB
gdb --args /home/user/ros2_ws/install/my_package/lib/my_package/my_node
(gdb) run
(gdb) backtrace  # 查看调用栈
(gdb) print variable_name
```

## 6.7 节点命名空间与重映射

### 6.7.1 命名空间

```bash
# 启动节点时指定命名空间
ros2 run my_package my_node --ros-args -r __node:=my_node -r __ns:=/my_namespace

# 结果节点名为 /my_namespace/my_node
```

**在 Launch 文件中设置：**

```python
Node(
    package='my_package',
    executable='my_node',
    namespace='my_namespace',
    name='my_node'
)
# 结果节点名: /my_namespace/my_node
```

### 6.7.2 重映射

```bash
# 重映射话题
ros2 run my_package talker --ros-args -r /chatter:=/new_topic

# 重映射节点名
ros2 run my_package my_node --ros-args -r __node:=new_node_name

# 重映射命名空间
ros2 run my_package my_node --ros-args -r __ns:=/new_namespace
```

## 6.8 最佳实践

### 6.8.1 节点设计原则

| 原则 | 说明 | 示例 |
|-----|------|------|
| **单一职责** | 每个节点做一件事 | 传感器节点只负责读取数据 |
| **低耦合** | 最小化节点间依赖 | 使用消息接口通信 |
| **可配置** | 使用参数而非硬编码 | 通过参数配置更新频率 |
| **健壮性** | 处理异常和错误 | 检查返回值和异常 |

### 6.8.2 节点结构模式

**发布者-订阅者节点：**

```cpp
class PubSubNode : public rclcpp::Node {
public:
    PubSubNode() : Node("pubsub_node") {
        // 发布者
        publisher_ = this->create_publisher<MsgType>("output", 10);

        // 订阅者
        subscription_ = this->create_subscription<MsgType>(
            "input", 10,
            std::bind(&PubSubNode::topic_callback, this, std::placeholders::_1)
        );
    }

private:
    void topic_callback(const MsgType::SharedPtr msg) {
        // 处理数据
        auto processed_msg = process(msg);

        // 发布结果
        publisher_->publish(processed_msg);
    }

    rclcpp::Publisher<MsgType>::SharedPtr publisher_;
    rclcpp::Subscription<MsgType>::SharedPtr subscription_;
};
```

## 6.9 常见问题

### 6.9.1 节点无法启动

**问题：** 运行节点时找不到可执行文件

**解决方法：**

```bash
# 1. 确保已编译
cd ~/ros2_ws
colcon build --packages-select my_package

# 2. Source 环境
source install/setup.bash

# 3. 检查可执行文件
ros2 pkg prefix my_package
ls ~/ros2_ws/install/my_package/lib/my_package/
```

### 6.9.2 节点无法通信

**问题：** 节点启动但无法接收/发送消息

**解决方法：**

```bash
# 1. 检查话题是否匹配
ros2 topic list
ros2 topic info /topic_name

# 2. 检查 QoS 配置
ros2 topic info /topic_name -v

# 3. 检查节点信息
ros2 node info /node_name

# 4. 使用 ros2 doctor 诊断
ros2 doctor --report
```

## 6.10 下一步

学习节点后，您可以：

1. **[07 Topics](./07-topics.md)** - 深入学习话题通信
2. **[08 Services](./08-services.md)** - 学习服务通信

