# 22 URDF 模型 (URDF Modeling)

## 22.1 URDF 概述

### 22.1.1 什么是 URDF

**URDF (Unified Robot Description Format)** 是 XML 格式的机器人描述文件，用于定义机器人的几何形状、关节、惯性等属性。

```
URDF 文件结构：

robot.urdf
├── <robot>                 # 根元素
│   ├── <link>              # 连杆定义
│   │   ├── <visual>        # 可视化
│   │   ├── <collision>     # 碰撞体
│   │   └── <inertial>      # 惯性
│   └── <joint>             # 关节定义
│       ├── <parent>        # 父连杆
│       ├── <child>         # 子连杆
│       └── <origin>        # 变换
```

### 22.1.2 Xacro

**Xacro** 是 URDF 的宏语言，简化 URDF 编写：

```xml
<!-- 使用 Xacro -->
<xacro:macro name="wheel" params="prefix">
  <link name="${prefix}_wheel_link" />
  <joint name="${prefix}_wheel_joint" />
</xacro:macro>

<xacro:wheel prefix="front_left" />
<xacro:wheel prefix="front_right" />
```

## 22.2 Link（连杆）

### 22.2.1 Link 基本结构

```xml
<link name="base_link">
  <!-- 惯性属性 -->
  <inertial>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <mass value="5.0"/>
    <inertia ixx="0.1" ixy="0.0" ixz="0.0"
             iyy="0.1" iyz="0.0"
             izz="0.1"/>
  </inertial>

  <!-- 可视化 -->
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="1.0 0.5 0.2"/>
    </geometry>
    <material name="blue">
      <color rgba="0.0 0.0 1.0 1.0"/>
    </material>
  </visual>

  <!-- 碰撞体 -->
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="1.0 0.5 0.2"/>
    </geometry>
  </collision>
</link>
```

### 22.2.2 惯性计算

```xml
<inertial>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <mass value="10.0"/>
  <!-- 惯性矩阵 (kg·m²) -->
  <inertia ixx="0.01" ixy="0.0" ixz="0.0"
           iyy="0.01" iyz="0.0"
           izz="0.01"/>
</inertial>
```

### 22.2.3 几何形状

```xml
<!-- 长方体 -->
<geometry>
  <box size="1.0 0.5 0.2"/>  <!-- length width height -->
</geometry>

<!-- 圆柱体 -->
<geometry>
  <cylinder radius="0.1" length="0.5"/>
</geometry>

<!-- 球体 -->
<geometry>
  <sphere radius="0.15"/>
</geometry>

<!-- 网格模型 -->
<geometry>
  <mesh filename="package://my_package/meshes/part.stl" scale="0.001 0.001 0.001"/>
</geometry>
```

## 22.3 Joint（关节）

### 22.3.1 关节类型

| 类型 | 说明 | 示例 |
|-----|------|------|
| `revolute` | 旋转关节（单轴） | 机械臂关节 |
| `prismatic` | 移动关节（单轴） | 线性执行器 |
| `continuous` | 连续旋转关节 | 车轮 |
| `fixed` | 固定连接 | 传感器安装 |
| `floating` | 浮动基座 | 移动机器人基座 |
| `planar` | 平面运动 | 2D 导航 |

### 22.3.2 关节定义

```xml
<!-- 旋转关节 -->
<joint name="joint1" type="revolute">
  <parent link="link1"/>
  <child link="link2"/>
  <origin xyz="0 0 0.2" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>  <!-- 绕 Z 轴旋转 -->
  <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
</joint>

<!-- 连续旋转关节 -->
<joint name="wheel_joint" type="continuous">
  <parent link="base_link"/>
  <child link="wheel_link"/>
  <origin xyz="0.2 0 0" rpy="-1.57 0 0"/>
  <axis xyz="0 0 1"/>
  <limit effort="100" velocity="10.0"/>
</joint>

<!-- 固定关节 -->
<joint name="camera_mount" type="fixed">
  <parent link="base_link"/>
  <child link="camera_link"/>
  <origin xyz="0.1 0 0.3" rpy="0 0 0"/>
</joint>

<!-- 移动关节 -->
<joint name="linear_joint" type="prismatic">
  <parent link="base"/>
  <child link="slider"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="0" upper="0.5" effort="50" velocity="0.5"/>
</joint>
```

### 22.3.3 关节限位

```xml
<limit
  lower="-1.57"      <!-- 最小角度 (rad) -->
  upper="1.57"       <!-- 最大角度 (rad) -->
  effort="100"       <!-- 最大扭矩 (N·m) -->
  velocity="1.0"     <!-- 最大速度 (rad/s) -->
/>
```

## 22.4 Material（材质）

### 22.4.1 颜色定义

```xml
<material name="blue">
  <color rgba="0.0 0.0 1.0 1.0"/>  <!-- R G B Alpha -->
</material>

<material name="red">
  <color rgba="1.0 0.0 0.0 1.0"/>
</material>

<material name="translucent_green">
  <color rgba="0.0 1.0 0.0 0.5"/>  <!-- 50% 透明 -->
</material>
```

### 22.4.2 纹理

```xml
<material name="wood">
  <texture filename="package://my_package/textures/wood.png"/>
</material>
```

## 22.5 Gazebo 标签

### 22.5.1 Gazebo 材质

```xml
<gazebo reference="base_link">
  <material>Gazebo/Blue</material>
</gazebo>

<gazebo reference="link1">
  <material>Gazebo/Grey</material>
</gazebo>
```

### 22.5.2 Gazebo 插件

```xml
<gazebo>
  <plugin name="diff_drive_controller" filename="libgazebo_ros_diff_drive.so">
    <update_rate>50</update_rate>
    <left_joint>left_wheel_joint</left_joint>
    <right_joint>right_wheel_joint</right_joint>
    <wheel_separation>0.5</wheel_separation>
    <wheel_diameter>0.2</wheel_diameter>
  </plugin>
</gazebo>
```

### 22.5.3 传感器

```xml
<!-- 激光雷达 -->
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
    </ray>
    <plugin name="gazebo_ros_laser" filename="libgazebo_ros_laser.so">
      <topic_name>/scan</topic_name>
    </plugin>
  </sensor>
</gazebo>

<!-- 相机 -->
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
    </plugin>
  </sensor>
</gazebo>
```

## 22.6 Xacro 宏

### 22.22.1 定义宏

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="my_robot">

  <!-- 定义轮子宏 -->
  <xacro:macro name="wheel" params="prefix x_reflect">
    <link name="${prefix}_wheel_link">
      <visual>
        <geometry>
          <cylinder radius="0.1" length="0.05"/>
        </geometry>
      </visual>
    </link>

    <joint name="${prefix}_wheel_joint" type="continuous">
      <parent link="base_link"/>
      <child link="${prefix}_wheel_link"/>
      <origin xyz="${x_reflect * 0.2} 0 0" rpy="-1.57 0 0"/>
      <axis xyz="0 0 1"/>
    </joint>
  </xacro:macro>

  <!-- 使用宏 -->
  <xacro:wheel prefix="front_left" x_reflect="1"/>
  <xacro:wheel prefix="front_right" x_reflect="-1"/>
  <xacro:wheel prefix="back_left" x_reflect="1"/>
  <xacro:wheel prefix="back_right" x_reflect="-1"/>

</robot>
```

### 22.6.2 属性和数学

```xml
<xacro:property name="wheel_radius" value="0.1"/>
<xacro:property name="wheel_separation" value="0.5"/>

<xacro:property name="base_x" value="${wheel_separation/2}"/>

<origin xyz="${base_x} 0 0" ... />
```

### 22.6.3 包含文件

```xml
<!-- 包含其他 xacro 文件 -->
<xacro:include filename="$(find my_package)/urdf/macros/wheel.xacro"/>
<xacro:include filename="$(find my_package)/urdf/macros/sensor.xacro"/>
```

## 22.7 完整示例

```xml
<?xml version="1.0"?>
<robot name="my_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- 基座 -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.6 0.4 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0.0 0.0 1.0 1.0"/>
      </material>
    </visual>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.1" iyy="0.1" izz="0.1"/>
    </inertial>
  </link>

  <!-- 车轮 -->
  <xacro:wheel prefix="front_left" x_reflect="1"/>
  <xacro:wheel prefix="front_right" x_reflect="-1"/>
  <xacro:wheel prefix="back_left" x_reflect="1"/>
  <xacro:wheel prefix="back_right" x_reflect="-1"/>

  <!-- 相机 -->
  <link name="camera_link">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.05"/>
      </geometry>
    </visual>
  </link>

  <joint name="camera_joint" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="0.2 0 0.15" rpy="0 0 0"/>
  </joint>

</robot>
```

## 22.8 验证和检查

```bash
# 检查 URDF 语法
check_urdf my_robot.urdf

# 可视化 URDF
urdf_to_graphiz my_robot.urdf
evince robot.pdf

# 使用 xacro 处理
xacro my_robot.xacro > processed.urdf
```

## 22.9 下一步

1. **[23 Gazebo](./23-gazebo.md)** - 学习物理仿真
2. **[24 Camera](./24-camera.md)** - 学习摄像头配置

---
**✅ 22 URDF 模型 - 已完成**
