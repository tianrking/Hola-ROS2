# 12 参数服务 (Parameters)

## 12.1 参数概述

### 12.1.1 什么是参数

**参数 (Parameters)** 是节点的配置值，可以在节点启动时设置，也可以在运行时动态修改。参数适合存储配置信息，如端口号、更新频率、传感器参数等。

```
参数概念图：

节点: camera_node
├── 参数: frame_id = "camera_link"
├── 参数: width = 640
├── 参数: height = 480
├── 参数: fps = 30
└── 参数: exposure_mode = "auto"

动态修改:
$ ros2 param set camera_node exposure_mode "manual"
参数已更新: exposure_mode = "manual"
```

### 12.1.2 参数的类型

| 类型 | 示例值 | 说明 |
|-----|-------|------|
| `bool` | `true`, `false` | 布尔值 |
| `int` | `42`, `-10` | 整数 |
| `float` | `3.14`, `0.001` | 浮点数 |
| `double` | `3.14159265` | 双精度浮点 |
| `string` | `"hello"` | 字符串 |
| `byte_array` | `[0x01, 0x02]` | 字节数组 |
| `bool_array` | `[true, false]` | 布尔数组 |
| `int_array` | `[1, 2, 3]` | 整数数组 |
| `float_array` | `[1.0, 2.0]` | 浮点数组 |
| `string_array` | `["a", "b"]` | 字符串数组 |

## 12.2 声明和使用参数

### 12.2.1 C++ 参数声明

```cpp
#include "rclcpp/rclcpp.hpp"

class ParameterNode : public rclcpp::Node {
public:
    ParameterNode() : Node("parameter_node") {
        // 声明参数 (名称, 默认值)
        this->declare_parameter("frequency", 10.0);
        this->declare_parameter("sensor_name", "camera");
        this->declare_parameter("enabled", true);
        this->declare_parameter("threshold", std::vector<int>{50, 100, 150});

        // 获取参数值
        double frequency = this->get_parameter("frequency").as_double();
        std::string sensor_name = this->get_parameter("sensor_name").as_string();
        bool enabled = this->get_parameter("enabled").as_bool();
        auto threshold = this->get_parameter("threshold").as_int_array();

        RCLCPP_INFO(this->get_logger(), "Frequency: %.2f", frequency);
        RCLCPP_INFO(this->get_logger(), "Sensor: %s", sensor_name.c_str());
        RCLCPP_INFO(this->get_logger(), "Enabled: %s", enabled ? "true" : "false");

        // 添加参数回调
        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&ParameterNode::parameters_callback, this, std::placeholders::_1)
        );

        // 创建定时器使用参数
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / frequency)),
            std::bind(&ParameterNode::timer_callback, this)
        );
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    void timer_callback() {
        bool enabled = this->get_parameter("enabled").as_bool();
        if (enabled) {
            RCLCPP_INFO(this->get_logger(), "Node is running");
        }
    }

    // 参数变更回调
    rcl_interfaces::msg::SetParametersResult parameters_callback(
        const std::vector<rclcpp::Parameter> & parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto & param : parameters) {
            if (param.get_name() == "frequency") {
                double freq = param.as_double();
                if (freq <= 0.0) {
                    result.successful = false;
                    result.reason = "Frequency must be positive";
                } else {
                    // 更新定时器
                    timer_->cancel();
                    timer_ = this->create_wall_timer(
                        std::chrono::milliseconds(static_cast<int>(1000.0 / freq)),
                        std::bind(&ParameterNode::timer_callback, this)
                    );
                    RCLCPP_INFO(this->get_logger(), "Frequency updated to %.2f Hz", freq);
                }
            }
        }

        return result;
    }
};
```

### 12.2.2 Python 参数声明

```python
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult


class ParameterNode(Node):

    def __init__(self):
        super().__init__('parameter_node')

        # 声明参数 (名称, 默认值)
        self.declare_parameter('frequency', 10.0)
        self.declare_parameter('sensor_name', 'camera')
        self.declare_parameter('enabled', True)
        self.declare_parameter('threshold', [50, 100, 150])

        # 获取参数值
        frequency = self.get_parameter('frequency').value
        sensor_name = self.get_parameter('sensor_name').value
        enabled = self.get_parameter('enabled').value
        threshold = self.get_parameter('threshold').value

        self.get_logger().info(f'Frequency: {frequency}')
        self.get_logger().info(f'Sensor: {sensor_name}')
        self.get_logger().info(f'Enabled: {enabled}')

        # 添加参数回调
        self.add_on_set_parameters_callback(self.parameter_callback)

    def parameter_callback(self, params):
        """参数变更回调"""
        for param in params:
            if param.name == 'frequency':
                freq = param.value
                if freq <= 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason='Frequency must be positive'
                    )
                self.get_logger().info(f'Frequency updated to {freq} Hz')

        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)
    node = ParameterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## 12.3 YAML 参数文件

### 12.3.1 创建参数文件

**config/params.yaml:**
```yaml
parameter_node:
  ros__parameters:
    frequency: 20.0
    sensor_name: "lidar"
    enabled: true
    threshold: [30, 60, 90]
    camera_settings:
      width: 640
      height: 480
```

### 12.3.2 从 Launch 文件加载

**Python Launch:**
```python
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('my_package'),
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        Node(
            package='my_package',
            executable='parameter_node',
            name='parameter_node',
            parameters=[config_file],
            output='screen'
        )
    ])
```

**C++ Launch:**
```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value='',
        description='Path to parameters file'
    )

    node = Node(
        package='my_package',
        executable='parameter_node',
        parameters=[LaunchConfiguration('params_file')],
    )

    return LaunchDescription([params_arg, node])
```

### 12.3.3 从命令行加载

```bash
# 从 YAML 文件加载参数
ros2 run my_package parameter_node --ros-params config/params.yaml

# 加载特定节点的参数
ros2 run my_package parameter_node --ros-params config/params.yaml --ros-args -r __node:=my_node
```

## 12.4 参数命令行工具

### 12.4.1 列出参数

```bash
# 列出节点所有参数
ros2 param list /parameter_node

# 列出所有节点的参数
ros2 param list

# 查看参数值
ros2 param get /parameter_node frequency

# 描述参数
ros2 param describe /parameter_node frequency
```

### 12.4.2 设置参数

```bash
# 设置参数
ros2 param set /parameter_node frequency 15.0

# 设置字符串参数
ros2 param set /parameter_node sensor_name "camera2"

# 设置数组参数
ros2 param set /parameter_node threshold "[40, 80, 120]"
```

### 12.4.3 删除参数

```bash
# 删除参数（某些实现支持）
ros2 param delete /parameter_node temp_param
```

### 12.4.4 导出参数

```bash
# 导出参数到 YAML 文件
ros2 param dump /parameter_node parameter_node.yaml

# 导出所有节点的参数
ros2 param dump --all
```

### 12.4.5 从文件加载参数

```bash
# 从文件加载参数
ros2 param load /parameter_node parameter_node.yaml
```

## 12.5 动态参数

### 12.5.1 动态参数配置

```cpp
// 使参数在运行时可修改
auto parameter_descriptor = rclcpp::ParameterDescriptor{};
parameter_descriptor.set__description("Sensor update frequency in Hz");
parameter_descriptor.set__read_only(false);  // 可修改

this->declare_parameter("frequency", 10.0, parameter_descriptor);
```

### 12.5.2 只读参数

```cpp
auto param_desc = rclcpp::ParameterDescriptor{};
param_desc.set__read_only(true);  // 设置为只读

this->declare_parameter("serial_number", "12345", param_desc);
```

## 12.6 参数最佳实践

### 12.6.1 参数命名规范

| 规范 | 示例 |
|-----|------|
| 使用小写 | `camera_fps` |
| 使用下划线 | `max_range` |
| 描述性命名 | `initial_position_x` |
| 避免缩写 | `threshold` 而非 `thr` |

### 12.6.2 参数组织

```yaml
# 按功能组织参数
my_robot:
  ros__parameters:
    # 传感器配置
    sensors:
      camera:
        width: 640
        height: 480
        fps: 30
      lidar:
        max_range: 10.0
        angular_resolution: 0.01

    # 控制配置
    control:
      max_velocity: 1.0
      max_acceleration: 0.5
```

### 12.6.3 参数验证

```cpp
rcl_interfaces::msg::SetParametersResult validate_parameters(
    const std::vector<rclcpp::Parameter> & parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto & param : parameters) {
        if (param.get_name() == "max_velocity") {
            double vel = param.as_double();
            if (vel < 0.0 || vel > 10.0) {
                result.successful = false;
                result.reason = "Velocity must be between 0 and 10";
                return result;  // 立即返回
            }
        }
    }

    return result;
}
```

## 12.7 常见问题

### 12.7.1 参数未声明

**问题:** 尝试获取未声明的参数

**解决:**
```cpp
// 方法1: 先声明再获取
this->declare_parameter("my_param", default_value);
auto value = this->get_parameter("my_param").as_double();

// 方法2: 使用 get_parameter_or (自动声明)
double value = this->get_parameter_or("my_param", 10.0);
```

### 12.7.2 类型不匹配

**问题:** 参数类型不匹配

**解决:**
```python
# 检查参数类型
param = self.get_parameter('my_param')
if param.type == rclpy.Parameter.Type.DOUBLE:
    value = param.value
else:
    self.get_logger().warn('Parameter type mismatch')
```

## 12.8 下一步

1. **[13 Metapackages](./13-metapackages.md)** - 学习元功能包
2. **[14 Distributed Communication](./14-distributed.md)** - 学习分布式通信

---
**✅ 12 参数服务 - 已完成**
