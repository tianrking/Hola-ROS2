# 08 服务通讯 (Services)

## 8.1 服务通信概述

### 8.1.1 什么是服务通信

**服务 (Service)** 是 ROS 2 中节点间进行同步通信的机制，采用客户端/服务器（Client/Server）模式。客户端发送请求，服务端处理并返回响应。

```
服务通信模型：

客户端A                    服务端                      客户端B
ClientA                   Server                     ClientB
  │                         │                           │
  │ ────请求(Request)──────►│                           │
  │                         │◄───请求(Request)───────────│
  │                         │                           │
  │ ◄───响应(Response)──────┘                           │
  │                                                     │
  │                         │ ────响应(Response)───────►│
  │                         │                           │
```

### 8.1.2 服务 vs 话题

| 特性 | 服务 (Services) | 话题 (Topics) |
|-----|----------------|--------------|
| **通信模式** | 同步（请求-响应） | 异步（发布-订阅） |
| **连接方式** | 一对一 | 多对多 |
| **适用场景** | 短暂操作、查询 | 持续数据流 |
| **阻塞** | 客户端阻塞等待 | 不阻塞 |
| **返回值** | 必须返回响应 | 无响应 |

### 8.1.3 服务类型定义

服务类型使用 `.srv` 文件定义，包含请求和响应两部分：

```srv
# 数据类型定义
# 文件: example_interfaces/srv/AddTwoInts.srv

# 请求部分（--- 上方）
int64 a
int64 b

# 响应部分（--- 下方）
int64 sum
```

## 8.2 创建服务端 (Server)

### 8.2.1 C++ 服务端

```cpp
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"
#include <memory>

class MinimalService : public rclcpp::Node {
public:
    MinimalService() : Node("minimal_service") {
        // 创建服务
        // 参数1: 服务名称
        // 参数2: 服务类型
        // 参数3: 回调函数
        service_ = this->create_service<example_interfaces::srv::AddTwoInts>(
            "add_two_ints",
            std::bind(&MinimalService::handle_add_two_ints, this,
                      std::placeholders::_1, std::placeholders::_2)
        );

        RCLCPP_INFO(this->get_logger(), "Service Ready");
    }

private:
    void handle_add_two_ints(
        const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
        std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Incoming request: %ld + %ld",
                    request->a, request->b);

        response->sum = request->a + request->b;
        RCLCPP_INFO(this->get_logger(), "Sending back response: %ld",
                    (long int)response->sum);
    }

    rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MinimalService>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

### 8.2.2 Python 服务端

```python
#!/usr/bin/env python3
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node


class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        # 创建服务
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_two_ints_callback
        )
        self.get_logger().info('Service Ready')

    def add_two_ints_callback(self, request, response):
        self.get_logger().info(
            f'Incoming request: {request.a} + {request.b}')

        response.sum = request.a + request.b
        self.get_logger().info(f'Sending back response: {response.sum}')
        return response


def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()

    try:
        rclpy.spin(minimal_service)
    except KeyboardInterrupt:
        pass

    minimal_service.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## 8.3 创建客户端 (Client)

### 8.3.1 C++ 客户端（同步调用）

```cpp
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"
#include <chrono>

using namespace std::chrono_literals;

class MinimalClient : public rclcpp::Node {
public:
    MinimalClient() : Node("minimal_client") {
        // 创建客户端
        client_ = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");

        // 定时器用于发送请求
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            [this]() {
                this->send_request();
            }
        );
    }

    void send_request() {
        // 等待服务可用
        while (!client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Service not available, waiting...");
        }

        // 创建请求
        auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
        request->a = 2;
        request->b = 3;

        // 同步调用
        auto future = client_->async_send_request(request);

        // 等待响应
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "Result: %ld + %ld = %ld",
                        request->a, request->b, response->sum);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Service call failed");
        }
    }

private:
    rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MinimalClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

### 8.3.2 C++ 客户端（异步调用）

```cpp
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

class AsyncClientNode : public rclcpp::Node {
public:
    AsyncClientNode() : Node("async_client_node") {
        client_ = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");

        // 发送多个异步请求
        for (int i = 0; i < 3; i++) {
            send_request_async(i, i + 1);
        }
    }

private:
    void send_request_async(int a, int b) {
        // 等待服务可用
        if (!client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_ERROR(this->get_logger(), "Service not available");
            return;
        }

        auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
        request->a = a;
        request->b = b;

        // 异步发送请求，并注册回调
        auto future = client_->async_send_request(
            request,
            [this](rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture future) {
                auto response = future.get();
                RCLCPP_INFO(this->get_logger(), "Response: %ld", response->sum);
            }
        );
    }

    rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client_;
};
```

### 8.3.3 Python 客户端

```python
#!/usr/bin/env python3
import sys
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        # 创建客户端
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')

        # 等待服务可用
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        # 异步调用
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request(int(sys.argv[1]), int(sys.argv[2]))
    minimal_client.get_logger().info(
        f'Result of add_two_ints: {minimal_client.req.a} + {minimal_client.req.b} = {response.sum}')

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## 8.4 常用服务类型

### 8.4.1 标准服务

**Empty 服务（无参数）：**

```srv
# std_srvs/srv/Empty.srv
# 请求和响应都为空
```

```cpp
// 使用示例
auto request = std::make_shared<std_srvs::srv::Empty::Request>();
auto future = client_->async_send_request(request);
```

**SetBool 服务：**

```srv
# std_srvs/srv/SetBool.srv
bool data   # 请求: 设置的值
bool success   # 响应: 是否成功
string message   # 响应: 状态信息
```

**Trigger 服务：**

```srv
# std_srvs/srv/Trigger.srv
bool success
string message
```

### 8.4.2 turtlesim 服务

**Spawn 服务（生成海龟）：**

```srv
float32 x
float32 y
float32 theta
string name
string name
```

**使用示例：**

```bash
# 使用命令行调用服务
ros2 service call /spawn turtlesim/srv/Spawn "{x: 2.0, y: 2.0, theta: 0.0, name: 'turtle2'}"
```

## 8.5 服务命令行工具

### 8.5.1 列出服务

```bash
# 列出所有服务
ros2 service list

# 列出特定节点的服务
ros2 service list | grep /turtlesim

# 列出服务及其类型
ros2 service list -t
```

### 8.5.2 查看服务信息

```bash
# 查看服务类型
ros2 service type /spawn

# 查看服务定义
ros2 service interface turtlesim/srv/Spawn

# 查找提供服务的节点
ros2 service find /spawn
```

### 8.5.3 调用服务

```bash
# 调用服务（基本语法）
ros2 service call <service_name> <service_type> <request>

# 调用 Empty 服务
ros2 service call /reset std_srvs/srv/Empty

# 调用带参数的服务
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 5, b: 3}"

# 调用 spawn 服务
ros2 service call /spawn turtlesim/srv/Spawn "{x: 5.0, y: 5.0, theta: 0.0, name: 'my_turtle'}"
```

### 8.5.4 回显服务请求/响应

```bash
# 监控服务调用
ros2 service echo /spawn
```

## 8.6 服务最佳实践

### 8.6.1 服务设计原则

| 原则 | 说明 |
|-----|------|
| **快速响应** | 服务应在合理时间内返回 |
| **幂等性** | 相同请求应产生相同结果 |
| **错误处理** | 明确的错误返回值 |
| **文档化** | 清晰的服务说明 |

### 8.6.2 命名规范

```
推荐的服务命名：

/reset                      # 重置服务
/spawn                      # 生成服务
/teleport_absolute          # 绝对移动
/set_camera_info            # 设置相机信息
/save_map                   # 保存地图
/start_navigation           # 开始导航
```

### 8.6.3 超时处理

```cpp
// 设置超时
auto future = client_->async_send_request(request);
auto status = future.wait_for(std::chrono::seconds(5));

if (status == std::future_status::timeout) {
    RCLCPP_ERROR(this->get_logger(), "Service call timeout");
    return;
}
```

```python
# Python 超时处理
try:
    future = self.cli.call_async(request)
    rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
    if future.done():
        response = future.result()
    else:
        self.get_logger().warn('Service call timeout')
except Exception as e:
    self.get_logger().error(f'Service call failed: {e}')
```

## 8.7 常见问题

### 8.7.1 服务调用失败

**问题：** 客户端无法连接到服务

**排查步骤：**

```bash
# 1. 检查服务是否存在
ros2 service list

# 2. 检查服务端节点是否运行
ros2 node list

# 3. 查看服务类型
ros2 service type /service_name

# 4. 测试服务调用
ros2 service call /service_name <type> "{data: value}"
```

### 8.7.2 请求参数错误

**问题：** 服务返回错误或不正确的结果

**解决方法：**

```bash
# 查看服务定义
ros2 interface show <service_type>

# 验证请求格式
ros2 service call /service_name <service_type> "{param: value}"
```

## 8.8 下一步

学习服务通信后，您可以：

1. **[09 Actions](./09-actions.md)** - 学习动作通信（长任务）
2. **[11 Custom Interfaces](./11-custom-interfaces.md)** - 创建自定义服务类型

