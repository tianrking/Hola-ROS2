# 24 摄像头预览 (Camera Preview)

## 24.1 摄像头驱动概述

### 24.1.1 USB 摄像头支持

ROS 2 支持多种 USB 摄像头驱动：

| 驱动包 | 说明 |
|-------|------|
| `usb_cam` | 通用 USB 摄像头驱动 |
| `libcamera` | 树莓派相机驱动 |
| `openni2` | Kinect/Astra 深度相机 |
| `realsense2` | Intel RealSense 相机 |

### 24.1.2 安装摄像头驱动

```bash
# USB 摄像头
sudo apt install ros-humble-usb-cam

# V4L 驱动
sudo apt install ros-humble-v4l2-camera

# RealSense
sudo apt install ros-humble-realsense2-camera

# libcamera
sudo apt install ros-humble-libcamera
```

## 24.2 USB 摄像头

### 24.2.1 查看摄像头设备

```bash
# 列出视频设备
ls /dev/video*

# 查看摄像头信息
v4l2-ctl --list-devices

# 查看支持的格式
v4l2-ctl -d /dev/video0 --list-formats-ext
```

### 24.2.2 启动 USB 摄像头

```bash
# 使用 usb_cam
ros2 run usb_cam usb_cam_node_exe --ros-args -p pixel_format:=mjpeg2rgb -p image_width:=640 -p image_height:=480

# 使用 v4l2_camera
ros2 run v4l2_camera v4l2_camera_node
```

### 24.2.3 Launch 文件

```python
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam_node',
            parameters=[{
                'video_device': '/dev/video0',
                'pixel_format': 'mjpeg2rgb',
                'image_width': 640,
                'image_height': 480,
                'camera_frame_id': 'camera_link',
                'camera_name': 'my_camera'
            }],
            output='screen'
        )
    ])
```

## 24.3 Image Transport

### 24.3.1 压缩图像传输

```bash
# 安装 image_transport 插件
sudo apt install ros-humble-image-transport*
sudo apt install ros-humble-compressed-image-transport
sudo apt install ros-humble-theora-image-transport
```

### 24.3.2 发布压缩图像

```python
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # 原始相机驱动
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam'
        ),
        # 压缩发布
        Node(
            package='image_transport',
            executable='republish',
            name='republish',
            parameters=[{
                'uncompressed_ns': 'raw',
                'compressed_ns': 'compressed',
                'use_inotify': True
            }],
            arguments=['raw', 'compressed'],
            remappings=[
                ('in', '/usb_cam/image_raw'),
                ('out', '/camera/image_raw')
            ]
        )
    ])
```

### 24.3.3 订阅压缩图像

```bash
# 显示原始图像
ros2 run image_tools showimage --ros-args -r image_raw:=/camera/image_raw

# 显示压缩图像
ros2 run image_tools showimage --ros-args -r image:=/camera/image_raw/compressed
```

## 24.4 相机校准文件

### 24.4.1 加载校准文件

```python
camera_node = Node(
    package='usb_cam',
    executable='usb_cam_node_exe',
    parameters=[{
        'camera_info_url': 'file:///path/to/calibration.yaml',
        'camera_frame_id': 'camera_link'
    }]
)
```

### 24.4.2 校准文件格式

```yaml
image_width: 640
image_height: 480
camera_name: my_camera
camera_matrix:
  rows: 3
  cols: 3
  data: [640.0, 0.0, 320.0,
         0.0, 640.0, 240.0,
         0.0, 0.0, 1.0]
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [0.0, 0.0, 0.0, 0.0, 0.0]
rectification_matrix:
  rows: 3
  cols: 3
  data: [1.0, 0.0, 0.0,
         0.0, 1.0, 0.0,
         0.0, 0.0, 1.0]
projection_matrix:
  rows: 3
  cols: 4
  data: [640.0, 0.0, 320.0, 0.0,
         0.0, 640.0, 240.0, 0.0,
         0.0, 0.0, 1.0, 0.0]
```

## 24.5 多摄像头

### 24.5.1 配置多个摄像头

```python
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # 摄像头 1
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            namespace='camera1',
            name='usb_cam1',
            parameters=[{
                'video_device': '/dev/video0',
                'camera_frame_id': 'camera1_link'
            }]
        ),
        # 摄像头 2
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            namespace='camera2',
            name='usb_cam2',
            parameters=[{
                'video_device': '/dev/video1',
                'camera_frame_id': 'camera2_link'
            }]
        )
    ])
```

### 24.5.2 同步多摄像头

```bash
# 使用 image_transport 同步
ros2 run image_transport sync_sub
```

## 24.6 深度相机

### 24.6.1 RealSense

```bash
# 安装
sudo apt install ros-humble-realsense2-camera

# 启动
ros2 launch realsense2_camera rs_launch.py
```

### 24.6.2 OpenNI

```bash
# 安装
sudo apt install ros-humble-openni2-camera

# 启动
ros2 launch openni2_camera openni2_camera.launch.py
```

## 24.7 可视化

### 24.7.1 RViz2 显示

```
1. 启动 RViz2
2. Add -> Camera
3. 设置 Image Topic: /camera/image_raw
4. 设置 Camera Info: /camera/camera_info
```

### 24.7.2 图像查看工具

```bash
# 显示图像
ros2 run image_tools showimage --ros-args -r image:=/camera/image_raw

# 使用 rqt_image_view
ros2 run rqt_image_view rqt_image_view
```

## 24.8 常见问题

| 问题 | 解决 |
|-----|------|
| 权限错误 | `sudo usermod -a -G video $USER` |
| 设备忙 | 关闭其他使用摄像头的程序 |
| 格式不支持 | 使用 MJPEG 或 YUYV |
| 低帧率 | 检查 USB 带宽 |

## 24.9 下一步

1. **[25 Calibration](./25-camera-calibration.md)** - 学习相机校准
2. **[26 AR Vision](./26-ar-vision.md)** - 学习 AR 视觉

