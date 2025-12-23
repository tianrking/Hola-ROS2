# 25 摄像头校准 (Camera Calibration)

## 25.1 相机校准概述

### 25.1.1 什么是相机校准

**相机校准** 是确定相机内参（焦距、主点、畸变系数）和外参（位置、姿态）的过程。

```
相机参数:

内参 (Intrinsic):
├── 焦距 (focal length)
├── 主点 (principal point)
└── 畸变系数 (distortion)

外参 (Extrinsic):
├── 旋转矩阵 (rotation)
└── 平移向量 (translation)
```

### 25.1.2 为什么要校准

| 用途 | 说明 |
|-----|------|
| **视觉测距** | 将像素距离转换为实际距离 |
| **3D 重建** | 精确的 3D 信息恢复 |
| **相机拼接** | 多相机图像拼接 |
| **机器人导航** | 准确的环境感知 |

## 25.2 安装校准工具

```bash
# 安装 camera_calibration
sudo apt install ros-humble-camera-calibration

# 安装相关依赖
sudo apt install python3-opencv python3-numpy
```

## 25.3 准备校准板

### 25.3.1 打印校准板

```bash
# 生成 8x6 棋盘格，每个格子 30mm
# 使用标定板生成工具
ros2 run camera_calibration cameracalibrator.py --print 8x6 0.03

# 或从网上下载标定板图像并打印
# https://github.com/ros-perception/image_pipeline/blob/noetic/calibration/doc/chessboard_8x6.pdf
```

### 25.3.2 校准板规格

| 类型 | 说明 |
|-----|------|
| **棋盘格** | 黑白相间方格 |
| **对称圆点** | 对称排列的圆点 |
| **不对称圆点** | 不对称排列的圆点 |

## 25.4 校准流程

### 25.4.1 启动相机

```bash
# 启动 USB 摄像头
ros2 run usb_cam usb_cam_node_exe --ros-args -p pixel_format:=yuyv -p image_width:=640 -p image_height:=480

# 或启动 v4l2_camera
ros2 run v4l2_camera v4l2_camera_node
```

### 25.4.2 启动校准程序

```bash
# 基本格式
ros2 run camera_calibration cameracalibrator \
  --size 8x6 \
  --square 0.03 \
  image:=/usb_cam/image_raw \
  camera:=/usb_cam

# 参数说明:
# --size: 棋盘格内角点数量 (行x列)
# --square: 每个格子的实际尺寸 (米)
# image: 相机图像话题
# camera: 相机命名空间
```

### 25.4.3 校准步骤

```
1. 调整相机位置
   - 使标定板尽量填满画面
   - 改变距离和角度

2. 捕捉图像
   - X (左右) 移动标定板
   - Y (上下) 移动标定板
   - Size (大小) 改变距离
   - Skew (倾斜) 改变角度

3. 查看进度
   - 左侧显示当前已捕捉的姿态
   - 达到要求后 CALIBRATE 按钮变亮

4. 校准
   - 点击 CALIBRATE 按钮开始校准
   - 等待计算完成

5. 保存
   - 点击 SAVE 保存校准文件
   - 点击 COMMIT 保存到磁盘
```

## 25.5 校准文件

### 25.5.1 校准文件位置

```bash
# 校准文件默认保存在 /tmp 目录
ls /tmp/calibrationdata*

# 复制到你的包
sudo cp /tmp/calibrationdata.tar.gz ~/ros2_ws/src/my_package/config/
cd ~/ros2_ws/src/my_package/config/
sudo tar -xvf calibrationdata.tar.gz
```

### 25.5.2 校准文件格式

```yaml
# ost.yaml (OpenCV 格式)
image_width: 640
image_height: 480
camera_name: narrow_stereo
camera_matrix:
  rows: 3
  cols: 3
  data: [570.0, 0.0, 320.0,
         0.0, 570.0, 240.0,
         0.0, 0.0, 1.0]
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [-0.02, 0.001, 0.0, 0.0, 0.0]
rectification_matrix:
  rows: 3
  cols: 3
  data: [1.0, 0.0, 0.0,
         0.0, 1.0, 0.0,
         0.0, 0.0, 1.0]
projection_matrix:
  rows: 3
  cols: 4
  data: [570.0, 0.0, 320.0, 0.0,
         0.0, 570.0, 240.0, 0.0,
         0.0, 0.0, 1.0, 0.0]
```

## 25.6 立体相机校准

### 25.6.1 校准立体相机

```bash
# 启动两个相机
ros2 run camera_calibration stereo_calibrator \
  --size 8x6 \
  --square 0.03 \
  --left-camera:=/left_camera \
  --right-camera:=/right_camera \
  --left-image:=/left_camera/image_raw \
  --right-image:=/right_camera/image_raw
```

### 25.6.2 立体校准文件

```yaml
left:
  camera_matrix: [...]
  distortion_model: plumb_bob
  distortion_coefficients: [...]

right:
  camera_matrix: [...]
  distortion_model: plumb_bob
  distortion_coefficients: [...]

stereo:
  rotation_matrix: [...]
  translation_vector: [...]
```

## 25.7 使用校准结果

### 25.7.1 加载校准文件

```python
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            parameters=[{
                'video_device': '/dev/video0',
                'camera_info_url': 'file:///path/to/ost.yaml',
                'camera_frame_id': 'camera_link'
            }]
        )
    ])
```

### 25.7.2 图像去畸变

```cpp
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>

void undistort_image(const sensor_msgs::msg::Image::SharedPtr msg) {
    // 相机矩阵
    cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << 570.0, 0.0, 320.0,
                                                       0.0, 570.0, 240.0,
                                                       0.0, 0.0, 1.0);

    // 畸变系数
    cv::Mat dist_coeffs = (cv::Mat_<double>(1, 5) << -0.02, 0.001, 0.0, 0.0, 0.0);

    // 转换 ROS 消息
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg);

    // 去畸变
    cv::Mat undistorted;
    cv::undistort(cv_ptr->image, undistorted, camera_matrix, dist_coeffs);

    // 显示结果
    cv::imshow("Original", cv_ptr->image);
    cv::imshow("Undistorted", undistorted);
    cv::waitKey(1);
}
```

## 25.8 校准质量评估

### 25.8.1 重投影误差

```bash
# 校准完成后会显示重投影误差
# 通常要求 < 0.5 像素
```

| 误差范围 | 评价 |
|---------|------|
| < 0.1 | 优秀 |
| 0.1 - 0.3 | 良好 |
| 0.3 - 0.5 | 可接受 |
| > 0.5 | 需要重新校准 |

### 25.8.2 验证校准

```bash
# 使用校准后的相机拍摄直线物体
# 检查是否出现弯曲
```

## 25.9 高级技巧

### 25.9.1 使用多个校准板

```bash
# 使用多种尺寸的校准板
# 提高不同距离的校准精度
```

### 25.9.2 自动校准

```python
# 使用相机校准 API 进行自动校准
import cv2
import numpy as np

# 准备标定点
objp = np.zeros((6*8, 3), np.float32)
objp[:, :2] = np.mgrid[0:8, 0:6].T.reshape(-1, 2)

# 存储所有标定点和角点
objpoints = []  # 3D 点
imgpoints = []  # 2D 点

# 处理每张标定图像
for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, (8, 6), None)

    if ret:
        objpoints.append(objp)
        imgpoints.append(corners)

# 执行标定
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, gray.shape[::-1], None, None
)
```

## 25.10 常见问题

| 问题 | 解决 |
|-----|------|
| 无法检测到标定板 | 检查光照和对比度 |
| 校准精度差 | 使用更多姿态、覆盖画面各区域 |
| 重投影误差大 | 确认标定板尺寸正确 |
| 校准失败 | 检查标定板是否为标准图案 |

## 25.11 下一步

1. **[26 AR Vision](./26-ar-vision.md)** - 学习 AR 视觉
2. 回顾 **[17 CLI Tools](./17-cli-tools.md)** - 复习命令行工具

