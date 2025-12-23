# 23 Gazebo 仿真 (Gazebo Simulation)

## 23.1 Gazebo 概述

### 23.1.1 什么是 Gazebo

**Gazebo** 是 ROS 2 的 3D 物理仿真环境，支持机器人动力学模拟、传感器仿真和环境建模。

```
Gazebo 仿真架构：

┌─────────────────────────────────────────────────┐
│                   Gazebo Server                 │
│  ┌─────────┐  ┌─────────┐  ┌─────────┐       │
│  │ 物理引擎 │  │ 渲染引擎 │  │ 传感器  │       │
│  │(ODE/Bullet)│ │(OGRE)  │  │ 插件    │       │
│  └─────────┘  └─────────┘  └─────────┘       │
├─────────────────────────────────────────────────┤
│                   ROS 2 接口                    │
│  ┌─────────┐  ┌─────────┐  ┌─────────┐       │
│  │/cmd_vel │  │/odom    │  │/scan    │       │
│  └─────────┘  └─────────┘  └─────────┘       │
└─────────────────────────────────────────────────┘
```

### 23.1.2 安装 Gazebo

```bash
# 安装 Gazebo 11 (Humble 默认)
sudo apt install gazebo11 libgazebo11-dev

# 安装 ROS 2 Gazebo 包
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-ros-gz-sim

# 安装常用插件
sudo apt install ros-humble-gazebo-plugins
sudo apt install ros-humble-gazebo-ros2-control
```

## 23.2 启动 Gazebo

### 23.2.1 空白世界

```bash
# 启动空白世界
gazebo

# 或使用 ros_gz_sim
ros2 launch ros_gz_sim gz.launch.py gz_args:=empty.sdf
```

### 23.2.2 从 Launch 启动

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Gazebo 启动
    gazebo = Node(
        package='ros_gz_sim',
        executable='gz sim',
        arguments=['-r', 'empty.sdf'],
        output='screen'
    )

    # Spawn 机器人
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'my_robot',
                  '-topic', '/robot_description'],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        spawn_entity
    ])
```

## 23.3 URDF/Gazebo 集成

### 23.3.1 在 Gazebo 中显示 URDF

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 启动 Gazebo
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so'],
        output='screen'
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': open('my_robot.urdf').read()}]
    )

    # Spawn 机器人
    spawn_urdf = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'my_robot',
                  '-file', 'my_robot.urdf'],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_urdf
    ])
```

### 23.3.2 Gazebo 插件

```xml
<!-- 差速驱动控制器 -->
<gazebo>
  <plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
    <update_rate>50</update_rate>
    <left_joint>left_wheel_joint</left_joint>
    <right_joint>right_wheel_joint</right_joint>
    <wheel_separation>0.5</wheel_separation>
    <wheel_diameter>0.2</wheel_diameter>
    <max_wheel_torque>20</max_wheel_torque>
    <command_topic>cmd_vel</command_topic>
    <odometry_topic>odom</odometry_topic>
    <odometry_frame>odom</odometry_frame>
    <robot_base_frame>base_link</robot_base_frame>
    <publish_odom>true</publish_odom>
    <publish_odom_tf>true</publish_odom_tf>
    <publish_wheel_tf>true</publish_wheel_tf>
  </plugin>
</gazebo>

<!-- IMU 插件 -->
<gazebo reference="imu_link">
  <sensor type="imu" name="imu_sensor">
    <plugin filename="libgazebo_ros_imu_sensor.so" name="imu_plugin">
      <topic_name>imu</topic_name>
      <bodyName>imu_link</bodyName>
    </plugin>
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <visualize>true</visualize>
  </sensor>
</gazebo>

<!-- GPS 插件 -->
<gazebo reference="gps_link">
  <sensor type="gps" name="gps_sensor">
    <plugin filename="libgazebo_ros_gps.so" name="gps_plugin">
      <topic_name>gps</topic_name>
      <referenceLatitude>48.858294</referenceLatitude>
      <referenceLongitude>2.294503</referenceLongitude>
    </plugin>
    <update_rate>10</update_rate>
  </sensor>
</gazebo>
```

## 23.4 传感器仿真

### 23.4.1 激光雷达

```xml
<gazebo reference="laser_link">
  <sensor type="ray" name="laser_sensor">
    <pose>0 0 0 0 0 0</pose>
    <visualize>true</visualize>
    <update_rate>10</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>360</samples>
          <resolution>1.0</resolution>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.1</min>
        <max>10.0</max>
        <resolution>0.01</resolution>
      </range>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.01</stddev>
      </noise>
    </ray>
    <plugin name="gazebo_ros_laser" filename="libgazebo_ros_laser.so">
      <topicName>/scan</topicName>
      <frameName>laser_link</frameName>
    </plugin>
  </sensor>
</gazebo>
```

### 23.4.2 相机

```xml
<gazebo reference="camera_link">
  <sensor type="camera" name="camera_sensor">
    <update_rate>30</update_rate>
    <camera name="camera">
      <horizontal_fov>1.047</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>100.0</far>
      </clip>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <alwaysOn>true</alwaysOn>
      <updateRate>0.0</updateRate>
      <cameraName>camera</cameraName>
      <imageTopicName>image_raw</imageTopicName>
      <cameraInfoTopicName>camera_info</cameraInfoTopicName>
    </plugin>
  </sensor>
</gazebo>
```

### 23.4.3 深度相机

```xml
<gazebo reference="depth_camera_link">
  <sensor type="depth" name="depth_camera">
    <update_rate>30</update_rate>
    <camera>
      <horizontal_fov>1.047</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>10.0</far>
      </clip>
    </camera>
    <plugin name="depth_camera_controller" filename="libgazebo_ros_openni_kinect.so">
      <baseline>0.1</baseline>
    </plugin>
  </sensor>
</gazebo>
```

## 23.5 环境建模

### 23.5.1 SDF 世界文件

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="my_world">
    <!-- 物理引擎 -->
    <physics name='default_physics' default='true' type='ode'>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
    </physics>

    <!-- 场景 -->
    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>true</shadows>
    </scene>

    <!-- 太阳光 -->
    <light name='sun' type='directional'>
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <!-- 地面 -->
    <model name='ground_plane'>
      <static>true</static>
      <link name='link'>
        <collision name='collision'>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>100</mu>
                <mu2>50</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
      </link>
    </model>
  </world>
</sdf>
```

### 23.5.2 添加模型

```bash
# 从命令行添加模型
gazebo my_world.sdf

# 在 Gazebo GUI 中添加
Edit -> Insert Model -> 选择模型
```

## 23.6 插件开发

### 23.6.1 自定义插件

```cpp
#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/gazebo.hh>

namespace gazebo {
class MyPlugin : public WorldPlugin {
public:
    void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf) {
        // 创建节点
        this->node = transport::NodePtr(new transport::Node());
        this->node->Init();

        // 订阅话题
        this->sub = this->node->Subscribe("~/my_topic", &MyPlugin::OnMsg, this);

        // 发布者
        this->pub = this->node->Advertise<msgs::Vector3d>("~/my_topic");
    }

    void OnMsg(ConstVector3dPtr &_msg) {
        // 处理消息
        double x = _msg->x();
        double y = _msg->y();
        double z = _msg->z();
    }

private:
    transport::NodePtr node;
    transport::SubscriberPtr sub;
    transport::PublisherPtr pub;
};

GZ_REGISTER_WORLD_PLUGIN(MyPlugin)
}
```

### 23.6.2 编译插件

```cmake
# CMakeLists.txt
add_library(my_plugin SHARED my_plugin.cpp)
target_link_libraries(my_plugin ${GAZEBO_LIBRARIES})
```

## 23.7 常用命令

```bash
# 启动 Gazebo
gazebo
gazebo world.sdf

# 列出模型
gz model -m

# 列出世界
gz world -l

# 暂停/继续
gz pause
gz play -d 1

# 重置
gz reset -1  # 重置世界
gz reset -a  # 重置所有
```

## 23.8 最佳实践

| 建议 | 说明 |
|-----|------|
| **物理设置** | 根据需求选择物理引擎 |
| **传感器频率** | 平衡性能和精度 |
| **插件使用** | 优先使用现成插件 |
| **模型简化** | 简化模型提高性能 |

## 23.9 下一步

1. **[24 Camera](./24-camera.md)** - 学习摄像头配置
2. **[25 Calibration](./25-camera-calibration.md)** - 学习相机校准

---
**✅ 23 Gazebo 仿真 - 已完成**
