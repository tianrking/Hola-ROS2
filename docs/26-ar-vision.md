# 26 AR 视觉 (AR Vision)

## 26.1 AR 视觉概述

### 26.1.1 什么是 AR 视觉

**AR (增强现实) 视觉** 是使用 ArUco 标记进行位姿估计的技术。ArUco 是一种二元方形标记，可以用于相机定位和姿态估计。

```
ArUco 标记应用：

┌─────────────────────────────────────────────────┐
│                                                 │
│   ┌─────┐         ┌─────┐         ┌─────┐     │
│   │ ID:0│         │ ID:1│         │ ID:2│     │
│   │     │         │     │         │     │     │
│   └─────┘         └─────┘         └─────┘     │
│                                                 │
│   摄像头检测 ArUco 标记 → 6D 位姿估计         │
│   (位置 + 旋转)                               │
│                                                 │
└─────────────────────────────────────────────────┘
```

### 26.1.2 ArUco 的用途

| 用途 | 说明 |
|-----|------|
| **机器人定位** | 室内机器人精确定位 |
| **物体抓取** | 确定目标物体的位姿 |
| **AR 显示** | 在标记位置叠加虚拟内容 |
| **相机标定** | 辅助相机参数标定 |

## 26.2 安装依赖

```bash
# 安装 vision_opencv
sudo apt install ros-humble-vision-opencv

# 安装 ArUco 相关包
sudo apt install ros-humble-aruco-opencv
sudo apt install ros-humble-aruco_ros

# 安装 OpenCV (带 contrib 模块)
pip3 install opencv-contrib-python
```

## 26.3 生成 ArUco 标记

### 26.3.1 使用 Python 生成

```python
import cv2
import numpy as np

# 定义 ArUco 字典
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)

# 生成标记
marker_size = 200  # 像素
marker_id = 0       # 标记 ID
marker_image = cv2.aruco.generateImageMarker(aruco_dict, marker_id, marker_size)

# 保存
cv2.imwrite('marker_0.png', marker_image)
```

### 26.3.2 打印标记

```python
import cv2

# 生成多个标记
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)

for id in range(10):
    marker = cv2.aruco.generateImageMarker(aruco_dict, id, 200)
    cv2.imwrite(f'marker_{id}.png', marker)
```

## 26.4 ArUco 检测

### 26.4.1 单标记检测

```cpp
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

void detect_aruco_markers() {
    cv::Mat image = cv::imread("aruco_image.jpg");

    // ArUco 字典
    cv::Ptr<cv::aruco::Dictionary> dictionary =
        cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

    // 检测参数
    cv::Ptr<cv::aruco::DetectorParameters> parameters =
        cv::aruco::DetectorParameters::create();

    // 检测标记
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(image, dictionary, corners, ids, parameters);

    // 绘制检测到的标记
    if (!ids.empty()) {
        cv::aruco::drawDetectedMarkers(image, corners, ids);
    }

    cv::imshow("ArUco Detection", image);
    cv::waitKey(0);
}
```

### 26.4.2 Python 检测

```python
import cv2
import numpy as np

# 加载图像
image = cv2.imread('aruco_image.jpg')

# ArUco 字典
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)

# 检测参数
parameters = cv2.aruco.DetectorParameters()

# 检测标记
corners, ids, rejected = cv2.aruco.detectMarkers(image, aruco_dict, parameters=parameters)

# 绘制
if ids is not None:
    cv2.aruco.drawDetectedMarkers(image, corners, ids)

cv2.imshow('ArUco Detection', image)
cv2.waitKey(0)
```

## 26.5 位姿估计

### 26.5.1 准备相机参数

```cpp
// 相机内参矩阵
cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << 570.0, 0.0, 320.0,
                                                       0.0, 570.0, 240.0,
                                                       0.0, 0.0, 1.0);

// 畸变系数
cv::Mat dist_coeffs = (cv::Mat_<double>(1, 5) << 0.0, 0.0, 0.0, 0.0, 0.0);

// 标记实际尺寸 (米)
float marker_length = 0.05;  // 5cm
```

### 26.5.2 估计位姿

```cpp
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>

// 检测标记
std::vector<int> ids;
std::vector<std::vector<cv::Point2f>> corners;
cv::aruco::detectMarkers(image, dictionary, corners, ids);

// 估计位姿
std::vector<cv::Vec3d> rvecs, tvecs;
cv::aruco::estimatePoseSingleMarkers(corners, marker_length,
                                   camera_matrix, dist_coeffs, rvecs, tvecs);

// 绘制坐标轴
for (size_t i = 0; i < ids.size(); i++) {
    cv::aruco::drawAxis(image, camera_matrix, dist_coeffs,
                       rvecs[i], tvecs[i], 0.1);
}
```

### 26.5.3 Python 位姿估计

```python
import cv2
import numpy as np

# 相机参数
camera_matrix = np.array([[570.0, 0.0, 320.0],
                          [0.0, 570.0, 240.0],
                          [0.0, 0.0, 1.0]])
dist_coeffs = np.array([0.0, 0.0, 0.0, 0.0, 0.0])

# 标记尺寸 (米)
marker_length = 0.05

# 检测并估计位姿
corners, ids, rejected = cv2.aruco.detectMarkers(image, aruco_dict)
rvecs, tvecs, _objPoints = cv2.aruco.estimatePoseSingleMarkers(
    corners, marker_length, camera_matrix, dist_coeffs
)

# 绘制坐标轴
if ids is not None:
    for i in range(len(ids)):
        cv2.aruco.drawAxis(image, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], 0.1)

cv2.imshow('Pose Estimation', image)
cv2.waitKey(0)
```

## 26.6 ROS 2 集成

### 26.6.1 aruco_ros 节点

```bash
# 安装
sudo apt install ros-humble-aruco-ros

# 启动单标记检测
ros2 launch aruco_ros aruco_node.launch.py

# 参数
camera_name:=camera
image_topic:=/camera/image_raw
camera_info_topic:=/camera/camera_info
marker_size:=0.05
marker_id:=0
```

### 26.6.2 发布位姿

```bash
# 订阅位姿话题
ros2 topic echo /aruco_single/pose

# 输出格式: geometry_msgs/msg/PoseStamped
```

## 26.7 实时 AR 显示

### 26.7.1 虚拟内容叠加

```python
import cv2
import numpy as np
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node


class ARDisplay(Node):
    def __init__(self):
        super().__init__('ar_display')

        # 订阅位姿
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/aruco_single/pose',
            self.pose_callback,
            10
        )

        # 加载虚拟内容
        self.virtual_object = cv2.imread('virtual_object.png', cv2.IMREAD_UNCHANGED)

    def pose_callback(self, msg):
        # 将 3D 点投影到 2D
        # 这里简化处理，实际需要完整的投影函数

        # 叠加虚拟内容
        overlay = self.overlay_virtual_object(image, pose)

        cv2.imshow('AR View', overlay)
        cv2.waitKey(1)

    def overlay_virtual_object(self, image, pose):
        # 实现虚拟内容叠加
        result = image.copy()

        # 使用 pose 变换虚拟对象位置
        # 然后绘制到图像上

        return result
```

### 26.7.2 ROS 2 节点实现

```cpp
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <opencv2/opencv.hpp>

class ARNode : public rclcpp::Node {
public:
    ARNode() : Node("ar_node") {
        // 位姿订阅者
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/aruco_single/pose",
            10,
            std::bind(&ARNode::pose_callback, this, std::placeholders::_1)
        );
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    cv::Mat virtual_object_;

    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        // 获取位姿
        double x = msg->pose.position.x;
        double y = msg->pose.position.y;
        double z = msg->pose.position.z;

        // 获取四元数
        auto q = msg->pose.orientation;

        // 处理 AR 显示...
        RCLCPP_INFO(this->get_logger(), "Marker position: x=%.2f, y=%.2f, z=%.2f",
                    x, y, z);
    }
};
```

## 26.8 应用示例

### 26.8.1 机器人抓取

```python
def compute_grasp_pose(marker_pose):
    """从标记位姿计算抓取位姿"""
    # 标记坐标系 → 物体坐标系 → 抓取点

    # 平移变换
    grasp_pose = marker_pose
    grasp_pose.position.z -= 0.1  # 上方 10cm

    return grasp_pose
```

### 26.8.2 导航定位

```python
def localize_robot(marker_id, marker_pose):
    """使用标记定位机器人"""
    # 已知标记在地图中的位置
    marker_map_positions = {
        0: [1.0, 0.0, 0.0],
        1: [0.0, 1.0, 0.0],
        2: [1.0, 1.0, 0.0]
    }

    if marker_id in marker_map_positions:
        # 计算机器人相对于地图的位置
        robot_pose = compute_robot_pose(marker_pose, marker_map_positions[marker_id])
        return robot_pose
```

## 26.9 最佳实践

| 建议 | 说明 |
|-----|------|
| **标记尺寸** | 精确测量并设置实际尺寸 |
| **光照** | 确保良好的光照条件 |
| **相机标定** | 使用标定好的相机参数 |
| **标记选择** | 根据应用选择合适的字典 |
| **遮挡处理** | 考虑部分遮挡的情况 |

## 26.10 进阶话题

### 26.10.1 标记板 (Board)

```cpp
// 创建标记板
cv::Ptr<cv::aruco::GridBoard> board =
    cv::aruco::GridBoard::create(5, 7, 0.04, 0.01, dictionary);

// 检测并估计标记板位姿
cv::Vec3d rvec, tvec;
int valid = cv::aruco::estimatePoseBoard(image, board, camera_matrix,
                                        dist_coeffs, rvec, tvec);
```

### 26.10.2 Diamond 标记

```python
# Diamond 标记 (4 个标记组合)
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
charuco_board = cv2.aruco.CharucoBoard_create(5, 7, 0.04, 0.02, dictionary)
```

## 26.11 下一步

恭喜！您已完成所有 26 篇 ROS 2 Humble 课程的学习。

**继续学习建议：**

1. 实践项目：构建一个完整的机器人系统
2. 研究更高级的话题：导航、MoveIt、SLAM
3. 参与开源项目贡献

**回顾系列：**

| 章节 | 内容 |
|-----|------|
| [01 ROS2 简介](./01-ros2-introduction.md) | 基础概念 |
| [02 安装](./02-installation-humble.md) | 环境搭建 |
| [06 节点](./06-nodes.md) | 节点编程 |
| [07 话题](./07-topics.md) | 通信机制 |
| [22 URDF](./22-urdf.md) | 机器人建模 |
