# ZRobot Deploy - 文件清单

## 核心代码文件

### 头文件 (include/zrobot_deploy/)
| 文件名 | 说明 | 行数 |
|--------|------|------|
| FSM.h | FSM基类定义，包含状态枚举和基础接口 | ~80 |
| FixStand.h | FixStand状态机定义 | ~60 |

### 源文件 (src/)
| 文件名 | 说明 | 行数 |
|--------|------|------|
| FSM.cpp | FSM基类实现，包含服务调用逻辑 | ~110 |
| FixStand.cpp | FixStand状态机实现，包含运动控制算法 | ~140 |
| main.cpp | 主程序，包含键盘控制和主循环 | ~170 |

## 配置文件

| 文件名 | 说明 |
|--------|------|
| CMakeLists.txt | CMake构建配置 |
| package.xml | ROS2包配置和依赖声明 |

## 文档文件

| 文件名 | 说明 | 用途 |
|--------|------|------|
| README.md | 完整项目文档 | 详细介绍、架构设计、使用说明 |
| QUICK_START.md | 快速使用指南 | 新用户快速上手 |
| IMPLEMENTATION.md | 实现技术细节 | 开发者参考、扩展开发 |
| CHANGELOG.md | 变更日志 | 版本历史和更新记录 |
| FILE_LIST.md | 本文件 | 文件清单 |

## 脚本文件

| 文件名 | 说明 | 权限 |
|--------|------|------|
| run.sh | 启动脚本 | 可执行 (755) |

## 目录结构

```
zrobot_deploy/
│
├── include/                    # 头文件目录
│   └── zrobot_deploy/
│       ├── FSM.h              # FSM基类
│       └── FixStand.h         # FixStand状态机
│
├── src/                        # 源文件目录
│   ├── FSM.cpp                # FSM实现
│   ├── FixStand.cpp           # FixStand实现
│   └── main.cpp               # 主程序
│
├── CMakeLists.txt             # CMake配置
├── package.xml                # ROS2包配置
│
├── README.md                  # 主文档
├── QUICK_START.md             # 快速指南
├── IMPLEMENTATION.md          # 技术文档
├── CHANGELOG.md               # 变更日志
├── FILE_LIST.md               # 本文件
│
└── run.sh                     # 启动脚本
```

## 代码统计

### 总览
- **总文件数**: 13
- **代码文件**: 5 (3个.cpp + 2个.h)
- **配置文件**: 2 (CMakeLists.txt + package.xml)
- **文档文件**: 5 (.md文件)
- **脚本文件**: 1 (.sh文件)

### 代码行数（估算）
- **头文件**: ~140 行
- **源文件**: ~420 行
- **总代码**: ~560 行
- **文档**: ~1000+ 行

## 依赖关系

### 内部依赖
```
main.cpp
  ├── FSM.h
  └── FixStand.h
      └── FSM.h

FixStand.cpp
  └── FixStand.h
      └── FSM.h

FSM.cpp
  └── FSM.h
```

### 外部依赖
```
ROS2依赖:
  ├── rclcpp (C++客户端库)
  ├── rs_interface (RobStrideMsgs服务)
  └── std_msgs (标准消息)

系统依赖:
  ├── termios.h (键盘输入)
  ├── unistd.h (系统调用)
  └── fcntl.h (文件控制)

C++标准库:
  ├── <memory> (智能指针)
  ├── <array> (固定数组)
  ├── <chrono> (时间处理)
  └── <iostream> (输入输出)
```

## 编译产物

编译后生成的文件位置：

```
install/zrobot_deploy/
├── lib/zrobot_deploy/
│   └── main                    # 可执行文件
├── include/zrobot_deploy/
│   ├── FSM.h                   # 已安装的头文件
│   └── FixStand.h
└── share/zrobot_deploy/
    ├── package.xml
    └── cmake/                  # CMake配置文件
```

## 文件大小（估算）

| 类型 | 大小 |
|------|------|
| 可执行文件 (main) | ~567 KB |
| 源代码总计 | ~15 KB |
| 文档总计 | ~30 KB |

## 许可证信息

所有文件头部应包含：
```cpp
//
// Created by Bill on 2026 01 16.
//
```

TODO: 添加完整的许可证信息

## 维护说明

### 添加新文件时需要更新：
1. CMakeLists.txt (如果是代码文件)
2. 本文件清单
3. CHANGELOG.md

### 代码规范
- 头文件使用 `#ifndef` 保护
- 类名使用大驼峰命名法
- 函数名使用小驼峰命名法
- 成员变量使用下划线后缀

### 文档规范
- 所有公共接口需要注释
- 关键算法需要说明
- 使用Markdown格式编写文档

---
**版本**: 1.0.0  
**生成日期**: 2026-01-16  
**最后更新**: 2026-01-16
