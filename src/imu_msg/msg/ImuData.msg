# 头文件依赖
std_msgs/Header header

# 标准 IMU 消息 (嵌套)
sensor_msgs/Imu imu

# 额外添加的欧拉角 (单位: rad 或 deg, 自己规定清楚)
float64 roll
float64 pitch
float64 yaw
