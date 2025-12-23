# 20 Launch 配置 (Launch Files)

## 20.1 Launch 概述

### 20.1.1 什么是 Launch

**Launch** 文件用于同时启动多个节点、设置参数、配置系统。ROS 2 使用 Python 编写 Launch 文件。

```
Launch 文件功能：

┌─────────────────────────────────────────────────┐
│              Launch 文件                          │
├─────────────────────────────────────────────────┤
│                                                 │
│  ┌─────────┐  ┌─────────┐  ┌─────────┐       │
│  │ 节点1   │  │ 节点2   │  │ 节点3   │       │
│  └─────────┘  └─────────┘  └─────────┘       │
│                                                 │
│  ┌─────────┐  ┌─────────┐  ┌─────────┐       │
│  │参数加载 │  │命名空间 │  │重映射   │       │
│  └─────────┘  └─────────┘  └─────────┘       │
│                                                 │
└─────────────────────────────────────────────────┘
```

### 20.1.2 Python Launch 文件结构

```
my_package/
├── launch/
│   ├── my_launch.py
│   └── another_launch.py
├── CMakeLists.txt
└── package.xml
```

## 20.2 基础 Launch 文件

### 20.2.1 最简单的 Launch

```python
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_package',
            executable='my_node',
            name='my_node',
            output='screen'
        )
    ])
```

### 20.2.2 启动多个节点

```python
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # 节点 1
        Node(
            package='my_package',
            executable='talker',
            name='talker',
            output='screen'
        ),
        # 节点 2
        Node(
            package='my_package',
            executable='listener',
            name='listener',
            output='screen'
        ),
    ])
```

## 20.3 Node 配置

### 20.3.1 基本参数

```python
Node(
    package='my_package',           # 包名
    executable='my_node',           # 可执行文件
    name='my_node',                 # 节点名
    namespace='my_namespace',       # 命名空间
    output='screen',                # 输出到屏幕
    emulate_tty=True,               # 彩色输出
    parameters=[{'param': value}],  # 参数
    arguments=['--ros-args', '-r', '__node:=new_name']  # 额外参数
)
```

### 20.3.2 命名空间

```python
Node(
    package='my_package',
    executable='my_node',
    namespace='robot1',  # 节点名: /robot1/my_node
    output='screen'
)
```

### 20.3.3 重映射

```python
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

Node(
    package='my_package',
    executable='my_node',
    remappings=[
        ('/input', '/new_input'),
        ('/output', '/new_output'),
    ],
    output='screen'
)
```

## 20.4 参数配置

### 20.4.1 加载 YAML 参数

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 获取参数文件路径
    config_file = os.path.join(
        get_package_share_directory('my_package'),
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        Node(
            package='my_package',
            executable='my_node',
            parameters=[config_file],
            output='screen'
        )
    ])
```

### 20.4.2 内联参数

```python
Node(
    package='my_package',
    executable='my_node',
    parameters=[
        {'frequency': 10.0},
        {'sensor_name': 'camera'},
        {'enabled': True}
    ],
    output='screen'
)
```

### 20.4.3 参数覆盖

```python
# YAML 文件中的参数可以被命令行参数覆盖
Node(
    package='my_package',
    parameters=[
        config_file,
        {'override_param': new_value}  # 覆盖 YAML 中的值
    ]
)
```

## 20.5 Launch 参数

### 20.5.1 声明参数

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 声明启动参数
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    frequency_arg = DeclareLaunchArgument(
        'frequency',
        default_value='10.0',
        description='Update frequency'
    )

    return LaunchDescription([
        use_sim_time_arg,
        frequency_arg,
        Node(
            package='my_package',
            executable='my_node',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'frequency': LaunchConfiguration('frequency'),
            }],
            output='screen'
        )
    ])
```

### 20.5.2 使用参数

```bash
# 使用默认参数
ros2 launch my_package my_launch.py

# 覆盖参数
ros2 launch my_package my_launch.py use_sim_time:=true frequency:=20.0
```

## 20.6 Include 其他 Launch

### 20.6.1 Include Launch 文件

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 包含另一个 launch 文件
    included_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('another_package'),
                'launch',
                'another_launch.py'
            )
        ),
        launch_arguments={'param': 'value'}.items()
    )

    return LaunchDescription([
        included_launch,
        # 添加其他节点
        Node(...)
    ])
```

## 20.7 条件和逻辑

### 20.7.1 条件启动

```python
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    enable_viz_arg = DeclareLaunchArgument(
        'enable_viz',
        default_value='true'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        condition=IfCondition(LaunchConfiguration('enable_viz')),
        arguments=['-d', get_package_share_directory('my_package') + '/rviz/config.rviz']
    )

    return LaunchDescription([
        enable_viz_arg,
        rviz_node
    ])
```

### 20.7.2 多个条件

```python
from launch.conditions import IfCondition, UnlessCondition

# 如果参数为真则启动
node1 = Node(
    package='pkg1',
    executable='node1',
    condition=IfCondition(LaunchConfiguration('enable_node1'))
)

# 如果参数为假则启动
node2 = Node(
    package='pkg2',
    executable='node2',
    condition=UnlessCondition(LaunchConfiguration('use_sim'))
)
```

## 20.8 事件处理

### 20.8.1 退出时关闭节点

```python
from launch.actions import ExecuteProcess
from launch.event_handlers import OnProcessExit


def generate_launch_description():
    recorder = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-a'],
        output='screen'
    )

    # 当主节点退出时，停止录制
    return LaunchDescription([
        main_node,
        recorder,
        # 注册事件处理器
        RegisterEventHandler(
            OnProcessExit(
                target_action=main_node,
                on_exit=[Emitter(name='recorder_stop', action=recorder)]
            )
        )
    ])
```

### 20.8.2 进程间通信

```python
from launch.actions import EmitEvent, RegisterEventHandler
from launch.events import matches_action


def generate_launch_description():
    # 节点 A
    node_a = Node(
        package='my_package',
        executable='node_a'
    )

    # 节点 B
    node_b = Node(
        package='my_package',
        executable='node_b',
        # 在节点 A 启动后才启动
        condition=...
    )
```

## 20.9 复杂示例

### 20.9.1 机器人启动文件

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 声明参数
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='robot1',
        description='Robot namespace'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    enable_viz_arg = DeclareLaunchArgument(
        'enable_viz',
        default_value='false',
        description='Enable visualization'
    )

    # URDF 文件
    urdf_file = os.path.join(
        get_package_share_directory('my_robot'),
        'urdf',
        'my_robot.urdf'
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': open(urdf_file).read(),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        output='screen'
    )

    # Joint State Publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )

    # RViz2 (条件启动)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(
            get_package_share_directory('my_robot'),
            'rviz',
            'config.rviz'
        )],
        condition=IfCondition(LaunchConfiguration('enable_viz')),
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )

    return LaunchDescription([
        namespace_arg,
        use_sim_time_arg,
        enable_viz_arg,
        GroupAction([
            PushRosNamespace(LaunchConfiguration('namespace')),
            robot_state_publisher,
            joint_state_publisher,
            rviz_node
        ])
    ])
```

## 20.10 Launch 文件安装

### 20.10.1 CMakeLists.txt 配置

```cmake
# 安装 Launch 文件
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch/
)
```

### 20.10.2 setup.py 配置

```python
data_files=[
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/launch', ['launch/my_launch.py']),
],
```

## 20.11 调试 Launch

### 20.11.1 检查语法

```bash
# 验证 launch 文件语法
python3 -m py_compile launch/my_launch.py
```

### 20.11.2 详细输出

```bash
# 显示详细输出
ros2 launch my_package my_launch.py --show-args

# 显示完整的 launch 配置
ros2 launch my_package my_launch.py --debug
```

## 20.12 最佳实践

| 建议 | 说明 |
|-----|------|
| **参数化** | 使用参数提高灵活性 |
| **模块化** | 将复杂 Launch 拆分成多个文件 |
| **文档化** | 添加注释说明功能 |
| **错误处理** | 使用条件处理异常情况 |
| **测试** | 测试不同参数组合 |

## 20.13 下一步

1. **[21 Rosbag2](./21-rosbag2.md)** - 学习数据包录制
2. **[22 URDF](./22-urdf.md)** - 学习机器人建模

---
**✅ 20 Launch 配置 - 已完成**
