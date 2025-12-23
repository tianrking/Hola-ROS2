# 21 录制回放 (Rosbag2)

## 21.1 Rosbag2 概述

### 21.1.1 什么是 Rosbag2

**Rosbag2** 是 ROS 2 的数据记录和回放工具，可以记录话题消息并在之后回放。

```
Rosbag2 工作流程：

┌─────────────┐     录制      ┌─────────────┐
│  运行节点   │ ────────────► │  数据包     │
│ (传感器数据) │               │  (.db3)     │
└─────────────┘               └─────────────┘
                                      │
                                      │ 回放
                                      ▼
                             ┌─────────────┐
                             │  测试节点   │
                             │ (算法验证)  │
                             └─────────────┘
```

### 21.1.2 Rosbag2 特点

| 特性 | 说明 |
|-----|------|
| **SQLite3** | 使用 SQLite3 数据库存储 |
| **元数据** | 包含时间戳、类型等元信息 |
| **压缩** | 支持数据压缩 |
| **多话题** | 同时录制多个话题 |
| **过滤** | 支持话题过滤 |

## 21.2 安装

```bash
# 安装 Rosbag2
sudo apt install ros-humble-rosbag2
sudo apt install ros-humble-ros2bag

# 安装压缩插件
sudo apt install ros-humble-rosbag2-compression-zstd

# 安装常用工具
sudo apt install ros-humble-rosbag2-transport
```

## 21.3 录制数据

### 21.3.1 录制所有话题

```bash
# 录制所有话题
ros2 bag record -a

# 指定输出目录
ros2 bag record -a -o my_bag

# 指定输出前缀
ros2 bag record -a --output-prefix test
```

### 21.3.2 录制特定话题

```bash
# 录制单个话题
ros2 bag record /chatter

# 录制多个话题
ros2 bag record /chatter /cmd_vel /odom

# 只录制一次
ros2 bag record -a --max-duration 10
```

### 21.3.3 录制选项

| 选项 | 说明 |
|-----|------|
| `-a` / `--all` | 录制所有话题 |
| `-o` / `--output` | 输出目录名 |
| `--max-duration` | 最长录制时间 |
| `--max-size` | 最大文件大小 |
| `--compression-mode` | 压缩模式 |
| `-e` / `--exclude` | 排除话题 |

```bash
# 限制录制时间
ros2 bag record -a --max-duration 30

# 限制文件大小 (MB)
ros2 bag record -a --max-size 1024

# 使用压缩
ros2 bag record -a --compression-mode zstd

# 排除某些话题
ros2 bag record -a -e /tf -e /tf_static
```

### 21.3.4 后台录制

```bash
# 后台录制
ros2 bag record -a -o background_bag &
BAG_PID=$!

# 运行测试
ros2 run my_package my_test

# 停止录制
kill $BAG_PID
```

## 21.4 回放数据

### 21.4.1 基本回放

```bash
# 回放数据包
ros2 bag play my_bag

# 回放指定包
ros2 bag play my_bag/record_2023_01_01-12_00_00

# 查看包内容
ros2 bag info my_bag
```

### 21.4.2 回放选项

| 选项 | 说明 |
|-----|------|
| `--rate` | 回放速率 |
| `--loop` | 循环回放 |
| `--delay` | 回放前延迟 |
| `--start-offset` | 从偏移位置开始 |

```bash
# 慢速回放 (0.5x)
ros2 bag play my_bag --rate 0.5

# 快速回放 (2.0x)
ros2 bag play my_bag --rate 2.0

# 循环回放
ros2 bag play my_bag --loop

# 延迟 5 秒后开始回放
ros2 bag play my_bag --delay 5

# 从 10 秒位置开始
ros2 bag play my_bag --start-offset 10
```

### 21.4.3 选择性回放

```bash
# 只回放特定话题
ros2 bag play my_bag --topics /chatter /cmd_vel

# 排除特定话题
ros2 bag play my_bag --exclude /tf /tf_static
```

## 21.5 数据包信息

### 21.5.1 查看包信息

```bash
# 查看基本信息
ros2 bag info my_bag

# 详细信息
ros2 bag info my_bag -v

# 查看元数据
ros2 bag info my_bag --metadata
```

**输出示例：**
```
Files:             my_bag_0.db3
Bag size:          256.0 KiB
Storage id:        sqlite3
Duration:          10.5 seconds
Start:             Jan  1 12:00:00.000 (2023)
End:               Jan  1 12:00:10.500 (2023)
Messages:          105
Topic information:  Topic: /chatter | Type: std_msgs/msg/String | Count: 105 | Serialization Format: cdr
```

### 21.5.2 查看话题

```bash
# 列出包中的话题
ros2 bag info my_bag | grep "Topic:"

# 查看特定话题的消息数
ros2 bag info my_bag -v
```

## 21.6 数据转换

### 21.6.1 转换格式

```bash
# 转换为旧格式 (ROS 1)
ros2 bag convert my_bag --src-bag-format sqlite3 --dst-bag-format cdr

# 压缩数据包
ros2 bag compress my_bag --compression-mode zstd

# 解压数据包
ros2 bag decompress my_bag
```

### 21.6.2 导出数据

```bash
# 导出为 CSV
ros2 bag export my_bag --output-dir exported_data

# 导出特定话题
ros2 bag export my_bag --topics /chatter --output-dir chatter_data
```

## 21.7 记录（Recording）

### 21.7.1 记录接口

Rosbag2 支持记录插件，可以实现自定义记录逻辑：

```bash
# 安装记录插件
sudo apt install ros-humble-rosbag2-storage-mcap
```

### 21.7.2 存储格式

| 格式 | 扩展名 | 说明 |
|-----|-------|------|
| SQLite3 | `.db3` | 默认格式 |
| MCAP | `.mcap` | 新的压缩格式 |

```bash
# 使用 MCAP 格式
ros2 bag record -a -s mcap
```

## 21.8 编程接口

### 21.8.1 在代码中录制

```python
from rclpy.node import Node
from rclpy.qos import QoSProfile
from ros2bag.api import create_writer
from rosbag2_py import Reader, Writer
from rosbag2_py import StorageOptions, ConverterOptions, RecordOptions


def record_bag():
    # 配置录制选项
    record_options = RecordOptions()
    record_options.all = True
    record_options.topic_polling_interval = 0.1

    # 配置存储选项
    storage_options = StorageOptions(
        uri='my_bag',
        storage_id='sqlite3'
    )

    # 创建录制器
    writer = Writer()
    writer.open(storage_options)

    # 添加话题
    writer.add_topic('/chatter', 'std_msgs/msg/String', QoSProfile())
```

### 21.8.2 在代码中回放

```python
from rosbag2_py import Reader
from rosbag2_py import StorageOptions, ConverterOptions


def play_bag():
    storage_options = StorageOptions(
        uri='my_bag',
        storage_id='sqlite3'
    )

    reader = Reader()
    reader.open(storage_options)

    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        print(f"Topic: {topic}, Timestamp: {timestamp}")
        # 处理数据...
```

## 21.9 最佳实践

### 21.9.1 录制建议

| 场景 | 建议 |
|-----|------|
| **调试** | 录制所有话题 |
| **长期存储** | 使用压缩 |
| **特定测试** | 只录制相关话题 |
| **性能敏感** | 使用 MCAP 格式 |

### 21.9.2 存储建议

```bash
# 定期备份
ros2 bag record -a -o daily_bags_$(date +%Y%m%d)

# 按大小分割
ros2 bag record -a --max-size 100 --split --output-prefix split_bag
```

### 21.9.3 隐私和安全

```bash
# 排除敏感话题
ros2 bag record -a -e /camera/image_raw -e /camera/camera_info

# 匿名化数据（需要后处理）
# 使用工具处理图像数据...
```

## 21.10 常见问题

### 21.10.1 录制失败

**问题:** 录制时遇到错误

**解决:**
```bash
# 检查磁盘空间
df -h

# 检查权限
ls -la my_bag

# 检查话题是否存在
ros2 topic list
```

### 21.10.2 回放问题

**问题:** 回放时节点无法接收消息

**解决:**
```bash
# 检查话题名称
ros2 bag info my_bag

# 使用 QoS 兼容性
ros2 bag play my_bag --qos-profile-overrides-path qos_override.yaml
```

## 21.11 下一步

1. **[22 URDF](./22-urdf.md)** - 学习机器人建模
2. **[23 Gazebo](./23-gazebo.md)** - 学习物理仿真

---
**✅ 21 录制回放 - 已完成**
