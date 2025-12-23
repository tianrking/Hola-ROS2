# 03 集成开发环境 (IDE Setup)

## 3.1 概述

良好的开发环境配置可以显著提高 ROS 2 开发效率。本章将详细介绍如何配置 Visual Studio Code (VS Code) 作为 ROS 2 的主要开发环境，包括插件安装、智能代码补全、调试配置等。

### 3.1.1 开发环境选择

| IDE | 优点 | 缺点 | 推荐度 |
|-----|------|------|-------|
| **VS Code** | 轻量、插件丰富、免费 | C++ 支持需额外配置 | ⭐⭐⭐⭐⭐ |
| **CLion** | 强大的 C++ 支持、内置调试 | 付费、较重 | ⭐⭐⭐⭐ |
| **Qt Creator** | 跨平台、CMake 支持好 | ROS 2 支持需手动配置 | ⭐⭐⭐ |
| **Vim/Neovim** | 轻量、高度可定制 | 学习曲线陡峭 | ⭐⭐ |

### 3.1.2 推荐配置

本文推荐使用 **VS Code** + **ROS 扩展** 的组合：
- 跨平台支持（Linux/Windows/macOS）
- 丰富的插件生态
- 完善的 ROS 2 支持
- 免费

## 3.2 Visual Studio Code 安装

### 3.2.1 安装 VS Code

**Ubuntu 22.04 通过 APT 安装：**

```bash
# 下载并安装 VS Code
wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg
sudo install -o root -g root -m 644 packages.microsoft.gpg /etc/apt/trusted.gpg.d/
sudo sh -c 'echo "deb [arch=amd64,arm64,armhf signed-by=/etc/apt/trusted.gpg.d/packages.microsoft.gpg] https://packages.microsoft.com/repos/code stable main" > /etc/apt/sources.list.d/vscode.list'
sudo apt update
sudo apt install -y code
```

**通过 Snap 安装：**
```bash
sudo snap install --classic code
```

**验证安装：**
```bash
code --version
```

### 3.2.2 VS Code 基本配置

启动 VS Code：
```bash
code
# 或者打开特定目录
code ~/ros2_ws
```

## 3.3 必装插件

### 3.3.1 ROS 2 核心插件

| 插件名称 | 发布者 | 用途 | 安装命令 |
|---------|--------|------|---------|
| **ROS** | Microsoft | ROS 支持 | `ext install ms-iot.vscode-ros` |
| **C/C++** | Microsoft | C++ 语言支持 | `ext install ms-vscode.cpptools` |
| **Python** | Microsoft | Python 语言支持 | `ext install ms-python.python` |
| **CMake Tools** | Microsoft | CMake 支持 | `ext install ms-vscode.cmake-tools` |

### 3.3.2 推荐插件

| 插件名称 | 发布者 | 用途 |
|---------|--------|------|
| **YAML** | Red Hat | YAML 文件支持 |
| **XML** | Red Hat | XML 文件支持 |
| **Better Comments** | Aaron Petheram | 更好的注释显示 |
| **GitLens** | GitKraken | Git 增强工具 |
| **TODO Highlight** | Wayou Liu | 高亮 TODO 注释 |
| **Bracket Pair Colorizer** | CoenraadS | 括号配对着色 |
| **Thunder Client** | Ranga Vadhineni | REST API 测试（替代 Postman） |

### 3.3.3 安装插件的方法

**方法 1：通过命令面板安装**

```
1. 按 Ctrl+Shift+P 打开命令面板
2. 输入 "Extensions: Install Extensions"
3. 搜索插件名称
4. 点击 Install 按钮
```

**方法 2：通过命令行安装**

```bash
# 安装 ROS 插件
code --install-extension ms-iot.vscode-ros

# 安装 C/C++ 插件
code --install-extension ms-vscode.cpptools

# 安装 Python 插件
code --install-extension ms-python.python

# 安装 CMake Tools
code --install-extension ms-vscode.cmake-tools
```

**方法 3：通过界面安装**

```
1. 点击左侧扩展图标 (或 Ctrl+Shift+X)
2. 搜索插件名称
3. 点击 Install
```

## 3.4 ROS 2 工作空间配置

### 3.4.1 打开 ROS 2 工作空间

```bash
# 打开工作空间
code ~/ros2_ws
```

### 3.4.2 配置 C/C++ 智能提示

VS Code 需要知道 ROS 2 的头文件路径才能提供准确的代码补全。

**创建 `.vscode/c_cpp_properties.json`：**

```json
{
    "configurations": [
        {
            "name": "Linux",
            "includePath": [
                "${workspaceFolder}/**",
                "/opt/ros/humble/include/**",
                "/usr/include/**"
            ],
            "defines": [],
            "compilerPath": "/usr/bin/gcc",
            "cStandard": "c17",
            "cppStandard": "c++17",
            "intelliSenseMode": "linux-gcc-x64",
            "compileCommands": "${workspaceFolder}/build/compile_commands.json"
        }
    ],
    "version": 4
}
```

**关键配置说明：**

| 配置项 | 说明 |
|-------|------|
| `includePath` | 头文件搜索路径，必须包含 ROS 2 路径 |
| `compileCommands` | 编译命令数据库，用于精确的代码分析 |
| `cppStandard` | C++ 标准，ROS 2 使用 C++17 |

### 4.4.3 生成 compile_commands.json

配置 colcon 生成编译命令数据库：

```bash
# 在工作空间根目录
cd ~/ros2_ws

# 编译时生成 compile_commands.json
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

# 创建符号链接到 src 目录（可选，方便某些工具访问）
ln -s build/compile_commands.json
```

**验证生成：**
```bash
cat build/compile_commands.json | jq '.[] | .directory' | head -5
```

### 3.4.4 配置 Python 环境

**创建 `.vscode/settings.json`：**

```json
{
    "python.autoComplete.extraPaths": [
        "${workspaceFolder}/install/*/lib/python3.10/site-packages",
        "/opt/ros/humble/lib/python3.10/site-packages"
    ],
    "python.analysis.extraPaths": [
        "${workspaceFolder}/install/*/lib/python3.10/site-packages",
        "/opt/ros/humble/lib/python3.10/site-packages"
    ],
    "python.formatting.provider": "black",
    "python.linting.enabled": true,
    "python.linting.pylintEnabled": true,
    "python.linting.pylintArgs": [
        "--rcfile=${workspaceFolder}/.pylintrc"
    ]
}
```

## 3.5 VS Code 任务配置

### 3.5.1 配置构建任务

**创建 `.vscode/tasks.json`：**

```json
{
    "version": "2.0.0",
    "tasks": [
        {
            "label": "colcon build",
            "type": "shell",
            "command": "colcon build --symlink-install",
            "group": {
                "kind": "build",
                "isDefault": true
            },
            "problemMatcher": [],
            "presentation": {
                "reveal": "always",
                "panel": "new"
            }
        },
        {
            "label": "colcon build (selected package)",
            "type": "shell",
            "command": "colcon build --symlink-install --packages-select ${input:packageName}",
            "group": "build",
            "problemMatcher": []
        },
        {
            "label": "source workspace",
            "type": "shell",
            "command": "source install/setup.bash && echo 'Workspace sourced'",
            "problemMatcher": []
        },
        {
            "label": "clean build",
            "type": "shell",
            "command": "rm -rf build install log && colcon build --symlink-install",
            "group": "build",
            "problemMatcher": []
        }
    ],
    "inputs": [
        {
            "id": "packageName",
            "type": "promptString",
            "description": "Enter package name to build"
        }
    ]
}
```

**使用任务：**

```
Ctrl+Shift+B                    # 运行默认构建任务
Ctrl+Shift+P -> Tasks: Run Task # 选择其他任务
```

### 3.5.2 配置测试任务

在 `tasks.json` 中添加：

```json
{
    "label": "colcon test",
    "type": "shell",
    "command": "colcon test --packages-select ${input:testPackage}",
    "group": "test",
    "problemMatcher": []
},
{
    "label": "colcon test --event-handlers",
    "type": "shell",
    "command": "colcon test --packages-select ${input:testPackage} --event-handlers console_direct+",
    "group": "test",
    "problemMatcher": []
},
{
    "label": "show test results",
    "type": "shell",
    "command": "colcon test-result --all --verbose",
    "group": "test",
    "problemMatcher": []
}
```

## 3.6 调试配置

### 3.6.1 C++ 节点调试

**创建 `.vscode/launch.json`：**

```json
{
    "version": "0.2.0",
    "configurations": [
        {
            "name": "ROS2: C++ Node",
            "type": "cppdbg",
            "request": "launch",
            "program": "${workspaceFolder}/install/${input:packageName}/lib/${input:packageName}/${input:executableName}",
            "args": [],
            "stopAtEntry": false,
            "cwd": "${workspaceFolder}",
            "environment": [
                {
                    "name": "ROS_DOMAIN_ID",
                    "value": "0"
                },
                {
                    "name": "RMW_IMPLEMENTATION",
                    "value": "rmw_cyclonedds_cpp"
                }
            ],
            "externalConsole": false,
            "MIMode": "gdb",
            "setupCommands": [
                {
                    "description": "Enable pretty-printing",
                    "text": "-enable-pretty-printing",
                    "ignoreFailures": true
                }
            ]
        }
    ],
    "inputs": [
        {
            "id": "packageName",
            "type": "promptString",
            "description": "Package name"
        },
        {
            "id": "executableName",
            "type": "promptString",
            "description": "Executable name"
        }
    ]
}
```

### 3.6.2 Python 节点调试

```json
{
    "name": "ROS2: Python Node",
    "type": "python",
    "request": "launch",
    "module": "rclpy.executors",
    "args": [
        "${workspaceFolder}/install/${input:packageName}/lib/${input:packageName}/${input:moduleName}"
    ],
    "console": "integratedTerminal",
    "env": {
        "ROS_DOMAIN_ID": "0",
        "PYTHONPATH": "${workspaceFolder}/install/${input:packageName}/lib/python3.10/site-packages:${env:PYTHONPATH}"
    }
}
```

### 3.6.3 调试操作

**调试快捷键：**

| 快捷键 | 功能 |
|-------|------|
| `F5` | 开始调试 |
| `Ctrl+Shift+F5` | 重启调试 |
| `Shift+F5` | 停止调试 |
| `F9` | 设置/取消断点 |
| `F10` | 单步跳过 |
| `F11` | 单步进入 |
| `Shift+F11` | 单步跳出 |

## 3.7 ROS 2 专用功能配置

### 3.7.1 ROS 扩展配置

**创建 `.vscode/settings.json`（ROS 相关）：**

```json
{
    "ros.distro": "humble",
    "ros.pythonPath": "/usr/bin/python3",
    "ros.defaultWorkspace": "${workspaceFolder}",
    "ros.rosSetupScript": "/opt/ros/humble/setup.bash",
    "ros.rosWorkspace": "${workspaceFolder}",
    "files.associations": {
        "*.world": "xml",
        "*.urdf": "xml",
        "*.xacro": "xml",
        "*.rviz": "yaml",
        "*.launch.py": "python"
    }
}
```

### 3.7.2 代码片段（Snippets）

**创建 `.vscode/ros2.code-snippets`：**

```json
{
    "ROS2 C++ Node Minimal": {
        "prefix": "ros2_cpp_node",
        "description": "Minimal ROS2 C++ node template",
        "body": [
            "#include \"rclcpp/rclcpp.hpp\"",
            "",
            "class ${1:NodeName} : public rclcpp::Node {",
            "public:",
            "    ${1:NodeName}() : Node(\"${1:NodeName}\") {",
            "        RCLCPP_INFO(this->get_logger(), \"${1:NodeName} has been started.\");",
            "    }",
            "};",
            "",
            "int main(int argc, char** argv) {",
            "    rclcpp::init(argc, argv);",
            "    auto node = std::make_shared<${1:NodeName}>();",
            "    rclcpp::spin(node);",
            "    rclcpp::shutdown();",
            "    return 0;",
            "}"
        ]
    },
    "ROS2 Python Node Minimal": {
        "prefix": "ros2_py_node",
        "description": "Minimal ROS2 Python node template",
        "body": [
            "import rclpy",
            "from rclpy.node import Node",
            "",
            "",
            "class ${1:NodeName}(Node):",
            "    def __init__(self):",
            "        super().__init__('${1:NodeName}')",
            "        self.get_logger().info('${1:NodeName} has been started.')",
            "",
            "",
            "def main(args=None):",
            "    rclpy.init(args=args)",
            "    node = ${1:NodeName}()",
            "    rclpy.spin(node)",
            "    node.destroy_node()",
            "    rclpy.shutdown()",
            "",
            "",
            "if __name__ == '__main__':",
            "    main()"
        ]
    },
    "ROS2 Publisher C++": {
        "prefix": "ros2_cpp_pub",
        "description": "ROS2 C++ publisher template",
        "body": [
            "auto publisher_ = this->create_publisher<${1:std_msgs::msg::String}>(\"${2:topic_name}\", 10);",
            "auto timer_ = this->create_wall_timer(",
            "    std::chrono::milliseconds(500),",
            "    [this]() {",
            "        auto message = ${1:std_msgs::msg::String}();",
            "        message.data = \"Hello, ROS 2!\";",
            "        publisher_->publish(message);",
            "    });"
        ]
    },
    "ROS2 Subscriber C++": {
        "prefix": "ros2_cpp_sub",
        "description": "ROS2 C++ subscriber template",
        "body": [
            "auto subscription_ = this->create_subscription<${1:std_msgs::msg::String}>(",
            "    \"${2:topic_name}\", 10,",
            "    [this](const ${1:std_msgs::msg::String}::SharedPtr msg) {",
            "        RCLCPP_INFO(this->get_logger(), \"Received: '%s'\", msg->data.c_str());",
            "    });"
        ]
    }
}
```

### 3.7.3 预定义变量

VS Code 中可用的预定义变量：

| 变量 | 说明 |
|-----|------|
| `${workspaceFolder}` | 工作空间根目录 |
| `${workspaceFolderBasename}` | 工作空间文件夹名 |
| `${file}` | 当前打开的文件 |
| `${fileBasename}` | 当前文件名 |
| `${fileDirname}` | 当前文件所在目录 |
| `${env:ENV_VAR}` | 环境变量 |

## 3.8 推荐的工作流

### 3.8.1 标准开发流程

```
1. 打开工作空间
   code ~/ros2_ws

2. Source ROS 2 环境（在集成终端中）
   source /opt/ros/humble/setup.bash

3. 构建工作空间
   Ctrl+Shift+B

4. 开发代码
   - 使用代码片段快速编写模板
   - 使用智能提示和自动补全
   - 保存后自动格式化

5. 运行节点进行测试
   Ctrl+Shift+` 打开新终端
   ros2 run package_name node_name

6. 如需调试
   设置断点 -> F5 开始调试
```

### 3.8.2 常用快捷键

| 快捷键 | 功能 |
|-------|------|
| `Ctrl+Shift+P` | 命令面板 |
| `Ctrl+P` | 快速打开文件 |
| `Ctrl+\`` | 切换集成终端 |
| `Ctrl+B` | 切换侧边栏 |
| `Ctrl+Shift+E` | 显示资源管理器 |
| `Ctrl+Shift+F` | 全局搜索 |
| `Alt+↑/↓` | 移动行 |
| `Ctrl+/` | 注释/取消注释 |
| `F2` | 重命名符号 |
| `Ctrl+Space` | 触发建议 |

## 3.9 其他 IDE 配置

### 3.9.1 CLion 配置

CLion 是专业的 C++ IDE，对 ROS 2 支持较好。

**导入 ROS 2 工作空间：**

1. 打开 CLion
2. `File` -> `Open` -> 选择工作空间 `src` 目录
3. `File` -> `Settings` -> `Build, Execution, Deployment` -> `CMake`

**配置 CMake 选项：**

```
CMake options: -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
Build directory: ../build
Environment: ROS_DISTRO=humble; source /opt/ros/humble/setup.bash
```

### 3.9.2 Qt Creator 配置

1. 打开 Qt Creator
2. `File` -> `Open File or Project`
3. 选择工作空间根目录的 `CMakeLists.txt`
4. 配置构建目录为 `build`
5. 设置环境变量

## 3.10 常见问题

### 3.10.1 C++ 智能提示不工作

**问题：** 无法找到 ROS 2 头文件

**解决方案：**

1. 确保 `c_cpp_properties.json` 包含正确的路径
2. 重新生成 `compile_commands.json`
3. 重启 VS Code

```bash
# 重新生成 compile_commands.json
cd ~/ros2_ws
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

### 3.10.2 Python 导入错误

**问题：** Python 节点无法导入 ROS 2 模块

**解决方案：**

检查 `.vscode/settings.json` 中的 Python 路径配置：

```json
{
    "python.autoComplete.extraPaths": [
        "/opt/ros/humble/lib/python3.10/site-packages",
        "${workspaceFolder}/install/*/lib/python3.10/site-packages"
    ]
}
```

### 3.10.3 调试时找不到节点

**问题：** 启动调试时提示找不到可执行文件

**解决方案：**

确保已经构建了包，并且可执行文件路径正确：

```bash
# 构建包
colcon build --packages-select <package_name>

# 查看可执行文件位置
find install -name <executable_name> -type f
```

### 3.10.4 ROS 扩展无法识别工作空间

**问题：** ROS 扩展显示无法识别工作空间

**解决方案：**

1. 确保 `.vscode/settings.json` 中配置了工作空间路径
2. 重启 VS Code
3. 手动运行 `source /opt/ros/humble/setup.bash`

## 3.11 完整配置示例

### 3.11.1 .vscode/settings.json（完整版）

```json
{
    // C/C++ 配置
    "C_Cpp.default.configurationProvider": "ms-vscode.cmake-tools",
    "C_Cpp.default.cppStandard": "c++17",
    "C_Cpp.default.cStandard": "c11",

    // Python 配置
    "python.defaultInterpreterPath": "/usr/bin/python3",
    "python.autoComplete.extraPaths": [
        "${workspaceFolder}/install/*/lib/python3.10/site-packages",
        "/opt/ros/humble/lib/python3.10/site-packages"
    ],

    // ROS 配置
    "ros.distro": "humble",
    "ros.rosSetupScript": "/opt/ros/humble/setup.bash",

    // 文件关联
    "files.associations": {
        "*.urdf": "xml",
        "*.xacro": "xml",
        "*.rviz": "yaml",
        "*.world": "xml",
        "*.launch.py": "python",
        "*.action": "yaml"
    },

    // 编辑器配置
    "editor.formatOnSave": true,
    "editor.tabSize": 4,
    "editor.insertSpaces": true,

    // CMake Tools 配置
    "cmake.sourceDirectory": "${workspaceFolder}/src",
    "cmake.buildDirectory": "${workspaceFolder}/build",
    "cmake.configureArgs": [
        "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON"
    ]
}
```

### 3.11.2 .vscode/tasks.json（完整版）

```json
{
    "version": "2.0.0",
    "tasks": [
        {
            "label": "colcon build",
            "type": "shell",
            "command": "colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON",
            "group": {
                "kind": "build",
                "isDefault": true
            },
            "problemMatcher": []
        },
        {
            "label": "colcon build --packages-select",
            "type": "shell",
            "command": "colcon build --symlink-install --packages-select ${input:packageName}",
            "group": "build",
            "problemMatcher": []
        },
        {
            "label": "colcon test",
            "type": "shell",
            "command": "colcon test --packages-select ${input:testPackage} --event-handlers console_direct+",
            "group": "test",
            "problemMatcher": []
        }
    ],
    "inputs": [
        {
            "id": "packageName",
            "type": "promptString",
            "description": "Package name to build"
        },
        {
            "id": "testPackage",
            "type": "promptString",
            "description": "Package name to test"
        }
    ]
}
```

## 3.12 下一步

开发环境配置完成后，您将可以：

1. **[04 Workspace](./04-workspace.md)** - 学习工作空间管理
2. **[05 Packages](./05-packages.md)** - 创建功能包
3. **[06 Nodes](./06-nodes.md)** - 编写节点代码

