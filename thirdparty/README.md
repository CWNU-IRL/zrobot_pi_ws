# 第三方依赖

## ROS2 发行版
- **ROS2 Jazzy**（`ros-jazzy-desktop` 包含的包不再单独列出）

## ROS2 额外包（不在 desktop 中）
| 名称 | 版本 | 所属 metapackage |
|------|------|------------------|
| controller_manager | Jazzy | ros-jazzy-ros2-control |
| gz_ros2_control | Jazzy | ros-jazzy-ros2-control |
| ros_gz_bridge | Jazzy | ros-jazzy-ros-gz |
| ros_gz_sim | Jazzy | ros-jazzy-ros-gz |

## 系统库（apt 安装）
| 名称 | 版本 | 安装命令 |
|------|------|----------|
| Eigen3 | - | `apt install libeigen3-dev` |
| LibSerial | - | `apt install libserial-dev` |
| can-utils | - | `apt install can-utils` |

## 预编译第三方库
手动下载并放置在 `thirdparty/` 目录下。
| 名称 | 版本 | 位置 |
|------|------|------|
| MuJoCo | 3.x+ | thirdparty/mujoco/ |
| LibTorch (PyTorch C++) | 2.11.0+cu126 | thirdparty/libtorch/ |
| ONNX Runtime | 1.16.3 | thirdparty/onnxruntime-linux-{aarch64,x64}-1.16.3/ |

