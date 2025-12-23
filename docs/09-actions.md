# 09 动作通讯 (Actions)

## 9.1 动作通信概述

### 9.1.1 什么是动作通信

**动作 (Action)** 是 ROS 2 中用于处理长时间任务的通信机制。与服务类似，动作也是客户端-服务端模式，但支持：
- 任务执行过程中的**实时反馈**
- 客户端可以**取消正在执行的任务**
- 适合处理**可能耗时数秒到数分钟**的操作

```
动作通信时序图：

客户端                              动作服务端
  │                                    │
  │ ────发送目标(Goal)─────────────────►│
  │                                    │
  │ ◄───反馈(Feedback)──────────────────┤ (持续发送)
  │ ◄───反馈(Feedback)──────────────────┤
  │ ◄───反馈(Feedback)──────────────────┤
  │                                    │
  │ ────取消请求(Cancel)────────────────►│ (可选)
  │                                    │
  │ ◄─────────结果(Result)──────────────┤ (任务完成)
```

### 9.1.2 动作 vs 服务

| 特性 | 服务 (Service) | 动作 (Action) |
|-----|---------------|--------------|
| **适用时长** | 短暂操作（毫秒-秒） | 长任务（秒-分钟） |
| **反馈** | 无实时反馈 | 持续发送反馈 |
| **取消** | 不支持 | 可中途取消 |
| **阻塞** | 客户端阻塞 | 可异步执行 |
| **应用场景** | 查询、简单操作 | 导航、抓取 |

### 9.1.3 动作类型定义

动作使用 `.action` 文件定义，包含三部分：目标、结果、反馈

```action
# Fibonacci.action
# 目标 (Goal) - 客户端发送的任务
int32 order
# 结果 (Result) - 任务完成后的返回值
int32[] sequence
bool success
# 反馈 (Feedback) - 任务执行过程中的状态更新
int32[] partial_sequence
```

## 9.2 创建动作服务端

### 9.2.1 C++ 动作服务端

```cpp
#include <functional>
#include <memory>
#include <thread>
#include <string>

#include "action_msgs/msg/goal_status_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "example_interfaces/action/fibonacci.hpp"

class FibonacciActionServer : public rclcpp::Node {
public:
    using Fibonacci = example_interfaces::action::Fibonacci;
    using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;

    explicit FibonacciActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
        : Node("fibonacci_action_server", options) {

        // 创建动作服务端
        action_server_ = rclcpp_action::create_server<Fibonacci>(
            this,
            "fibonacci",
            std::bind(&FibonacciActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&FibonacciActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&FibonacciActionServer::handle_accepted, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Fibonacci Action Server Ready");
    }

private:
    rclcpp_action::Server<Fibonacci>::SharedPtr action_server_;

    // 处理新目标
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const Fibonacci::Goal> goal) {

        RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->order);

        // 验证目标
        if (goal->order > 10000 || goal->order < 0) {
            return rclcpp_action::GoalResponse::REJECT;
        }

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    // 处理取消请求
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleFibonacci> goal_handle) {

        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    // 处理已接受的目标
    void handle_accepted(const std::shared_ptr<GoalHandleFibonacci> goal_handle) {
        // 在新线程中执行任务，避免阻塞
        std::thread{std::bind(&FibonacciActionServer::execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    // 执行动作
    void execute(const std::shared_ptr<GoalHandleFibonacci> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Executing goal");

        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<Fibonacci::Feedback>();
        auto & sequence = feedback->partial_sequence;

        sequence.push_back(0);
        sequence.push_back(1);

        auto result = std::make_shared<Fibonacci::Result>();

        // 执行计算
        for (int i = 1; (i < goal->order) && rclcpp::ok(); ++i) {
            // 检查是否被取消
            if (goal_handle->is_canceling()) {
                result->sequence = sequence;
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal canceled");
                return;
            }

            // 更新序列
            sequence.push_back(sequence[i] + sequence[i - 1]);

            // 发送反馈
            goal_handle->publish_feedback(feedback);
            RCLCPP_INFO(this->get_logger(), "Publish feedback");

            // 模拟工作
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }

        // 检查是否成功
        if (rclcpp::ok()) {
            result->sequence = sequence;
            result->success = true;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FibonacciActionServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

### 9.2.2 Python 动作服务端

```python
#!/usr/bin/env python3
import time
from example_interfaces.action import Fibonacci
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node


class FibonacciActionServer(Node):

    def __init__(self):
        super().__init__('fibonacci_action_server')

        # 创建动作服务端
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback
        )
        self.get_logger().info('Fibonacci Action Server Ready')

    def execute_callback(self, goal_handle):
        """执行动作"""
        self.get_logger().info('Executing goal...')

        feedback_msg = Fibonacci.Feedback()
        feedback_msg.partial_sequence = [0, 1]

        # 执行计算
        for i in range(1, goal_handle.request.order):
            # 检查是否被取消
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Fibonacci.Result()

            # 更新序列
            feedback_msg.partial_sequence.append(
                feedback_msg.partial_sequence[i] + feedback_msg.partial_sequence[i - 1]
            )

            # 发送反馈
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'Feedback: {feedback_msg.partial_sequence}')

            # 模拟工作
            time.sleep(0.2)

        # 返回结果
        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.partial_sequence
        result.success = True

        self.get_logger().info('Goal succeeded')
        return result


def main(args=None):
    rclpy.init(args=args)
    fibonacci_action_server = FibonacciActionServer()

    try:
        rclpy.spin(fibonacci_action_server)
    except KeyboardInterrupt:
        pass

    fibonacci_action_server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## 9.3 创建动作客户端

### 9.3.1 C++ 动作客户端

```cpp
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "example_interfaces/action/fibonacci.hpp"

using namespace std::chrono_literals;
using Fibonacci = example_interfaces::action::Fibonacci;

class FibonacciActionClient : public rclcpp::Node {
public:
    FibonacciActionClient() : Node("fibonacci_action_client") {
        // 创建动作客户端
        action_client_ = rclcpp_action::create_client<Fibonacci>(this, "fibonacci");

        // 定时器触发发送目标
        timer_ = this->create_wall_timer(
            500ms,
            [this]() {
                if (!this->action_client_->wait_for_action_server(1s)) {
                    RCLCPP_ERROR(this->get_logger(), "Action server not available");
                    return;
                }
                this->send_goal(10);  // 发送目标，计算前10个斐波那契数
                timer_->cancel();
            }
        );
    }

private:
    rclcpp_action::Client<Fibonacci>::SharedPtr action_client_;
    rclcpp::TimerBase::SharedPtr timer_;

    void send_goal(int order) {
        // 创建目标
        auto goal_msg = Fibonacci::Goal();
        goal_msg.order = order;

        RCLCPP_INFO(this->get_logger(), "Sending goal");

        // 配置选项
        auto goal_options = rclcpp_action::Client<Fibonacci>::SendGoalOptions();
        goal_options.goal_response_callback =
            std::bind(&FibonacciActionClient::goal_response_callback, this, std::placeholders::_1);
        goal_options.feedback_callback =
            std::bind(&FibonacciActionClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        goal_options.result_callback =
            std::bind(&FibonacciActionClient::result_callback, this, std::placeholders::_1);

        // 异步发送目标
        action_client_->async_send_goal(goal_msg, goal_options);
    }

    // 目标响应回调
    void goal_response_callback(std::shared_future<rclcpp_action::ClientGoalHandle<Fibonacci>::SharedPtr> future) {
        auto goal_handle = future.get();
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server");
    }

    // 反馈回调
    void feedback_callback(
        rclcpp_action::ClientGoalHandle<Fibonacci>::SharedPtr,
        const std::shared_ptr<const Fibonacci::Feedback> feedback) {
        RCLCPP_INFO(this->get_logger(), "Received feedback: %s",
                    feedback->partial_sequence.back());
    }

    // 结果回调
    void result_callback(const rclcpp_action::ClientGoalHandle<Fibonacci>::WrappedResult & result) {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Goal succeeded");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                return;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                return;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                return;
        }

        RCLCPP_INFO(this->get_logger(), "Result received");
        for (auto number : result.result->sequence) {
            RCLCPP_INFO(this->get_logger(), "%d", number);
        }
        rclcpp::shutdown();
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FibonacciActionClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

### 9.3.2 Python 动作客户端

```python
#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from example_interfaces.action import Fibonacci


class FibonacciActionClient(Node):

    def __init__(self):
        super().__init__('fibonacci_action_client')
        # 创建动作客户端
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')

    def send_goal(self, order):
        """发送目标"""
        self.get_logger().info('Waiting for action server...')
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available')
            return False

        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self.get_logger().info(f'Sending goal: {order}')

        # 异步发送目标
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        self._send_goal_future.add_done_callback(self.goal_response_callback)
        return True

    def goal_response_callback(self, future):
        """目标响应回调"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return

        self.get_logger().info('Goal accepted')

        # 获取结果
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        """反馈回调"""
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Feedback: {feedback.partial_sequence[-1]}')

    def get_result_callback(self, future):
        """结果回调"""
        result = future.result().result
        self.get_logger().info(f'Result: {result.sequence}')
        self.get_logger().info(f'Success: {result.success}')

        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    action_client = FibonacciActionClient()

    action_client.send_goal(10)

    try:
        rclpy.spin(action_client)
    except KeyboardInterrupt:
        pass

    action_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## 9.4 动作命令行工具

### 9.4.1 列出动作

```bash
# 列出所有动作
ros2 action list

# 列出动作及其类型
ros2 action list -t
```

### 9.4.2 查看动作信息

```bash
# 查看动作类型
ros2 action type /fibonacci

# 查看动作定义
ros2 interface show example_interfaces/action/Fibonacci
```

### 9.4.3 发送动作目标

```bash
# 发送动作目标
ros2 action send_goal /fibonacci example_interfaces/action/Fibonacci "{order: 10}"

# 带反馈的目标发送
ros2 action send_goal /fibonacci example_interfaces/action/Fibonacci "{order: 10}" --feedback

# 取消动作
ros2 action send_goal /fibonacci example_interfaces/action/Fibonacci "{order: 10}" &
# Ctrl+C 取消
```

## 9.5 导航动作示例

导航是动作通信的典型应用场景：

```action
# NavigateToPose.action
geometry_msgs/PoseStamped pose
string behavior_tree
std_msgs/Empty result
geometry_msgs/PoseStamped current_pose
```

```python
# 使用导航动作客户端
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped

navigator = BasicNavigator()

# 设置目标位姿
goal_pose = PoseStamped()
goal_pose.header.frame_id = 'map'
goal_pose.pose.position.x = 2.0
goal_pose.pose.position.y = 1.0
goal_pose.pose.orientation.w = 1.0

# 发送导航目标
navigator.goToPose(goal_pose)

# 等待完成
navigator.waitUntilNavTaskComplete()
```

## 9.6 动作最佳实践

### 9.6.1 何时使用动作

| 场景 | 推荐通信方式 |
|-----|------------|
| 简单查询 | 服务 |
| 短暂操作 | 服务 |
| 持续数据流 | 话题 |
| **长任务（>1秒）** | **动作** |
| **需要反馈** | **动作** |
| **需要取消** | **动作** |

### 9.6.2 动作设计原则

| 原则 | 说明 |
|-----|------|
| **明确的目标** | 目标应清晰描述任务 |
| **有意义的反馈** | 反馈应反映任务进度 |
| **合理的结果** | 结果应包含任务最终状态 |
| **可取消性** | 长任务应支持取消 |

## 9.7 下一步

学习动作通信后，您可以：

1. **[10 TF2](./10-tf2.md)** - 学习坐标变换
2. **[11 Custom Interfaces](./11-custom-interfaces.md)** - 创建自定义动作类型

