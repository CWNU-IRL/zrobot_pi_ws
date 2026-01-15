import math
import serial
import struct
import numpy as np
import threading
import rclpy
from enum import Enum
from rclpy.node import Node
from sensor_msgs.msg import Imu
from imu_msg.msg import ImuData
import time

# 定义协议类型枚举
class ProtocolType(Enum):
    TTL = 1      # TTL协议
    CAN = 2      # CAN协议
    RS485 = 3    # 485协议

# 全局配置 - 根据需要修改
CURRENT_PROTOCOL = ProtocolType.TTL
CURRENT_Buad = 2000000
ModBus_ID = 0x50

# RS485寄存器命令表
RS485_COMMANDS = {
    0x34: bytes([ModBus_ID, 0x03, 0x00, 0x34, 0x00, 0x03, 0x49, 0x84]),  # 加速度
    0x37: bytes([ModBus_ID, 0x03, 0x00, 0x37, 0x00, 0x03, 0x48, 0x55]),  # 角速度
    0x3A: bytes([ModBus_ID, 0x03, 0x00, 0x3A, 0x00, 0x03, 0x48, 0xC2]),  # 磁场
    0x3D: bytes([ModBus_ID, 0x03, 0x00, 0x3D, 0x00, 0x03, 0x49, 0x33])   # 角度
}

Key = 0
Buff = {}
AngleDegree = [0, 0, 0]
Magnetometer = [0, 0, 0]
Acceleration = [0, 0, 0]
AngularVelocity = [0, 0, 0]

def HexToShort(Hex, protocol_type):
    if protocol_type == ProtocolType.TTL:
        return list(struct.unpack("hhhh", bytearray(Hex)))

    elif protocol_type == ProtocolType.CAN:
        if len(Hex) != 6:
            return [0, 0, 0]
        
        data1 = (Hex[1] << 8) | Hex[0]
        data2 = (Hex[3] << 8) | Hex[2]
        data3 = (Hex[5] << 8) | Hex[4]

        # 处理有符号数（16位补码）
        def to_signed_16bit(value):
            if value & 0x8000:  # 检查符号位
                return value - 0x10000
            return value
        
        return [to_signed_16bit(data1), to_signed_16bit(data2), to_signed_16bit(data3)]
    
    elif protocol_type == ProtocolType.RS485:
        if len(Hex) != 6:
            return [0, 0, 0]
        
        data1 = (Hex[0] << 8) | Hex[1]
        data2 = (Hex[2] << 8) | Hex[3]
        data3 = (Hex[4] << 8) | Hex[5]

        # 处理有符号数（16位补码）
        def to_signed_16bit(value):
            if value & 0x8000:  # 检查符号位
                return value - 0x10000
            return value
        
        return [to_signed_16bit(data1), to_signed_16bit(data2), to_signed_16bit(data3)]

def CheckSum(ListData, CheckData):
    return sum(ListData) & 0xFF == CheckData

def HandleSerialData(Data, protocol_type):
    global Buff, Key, AngleDegree, Magnetometer, Acceleration, AngularVelocity
    AngleFlag = False
    Buff[Key] = Data
    Key += 1
    
    if protocol_type == ProtocolType.TTL or protocol_type == ProtocolType.CAN:
        # 协议头
        if Buff[0] != 0x55:
            Key = 0
            return
    elif protocol_type == ProtocolType.RS485:
        # 协议头
        if Buff[0] != ModBus_ID:
            Key = 0
            return
    
    if protocol_type == ProtocolType.TTL or protocol_type == ProtocolType.RS485:
        # TTL协议包长11
        if Key < 11:
            return
    elif protocol_type == ProtocolType.CAN:
        # Can包长8
        if Key < 8:
            return

    DataBuff = list(Buff.values())
    
    if protocol_type == ProtocolType.TTL:
        if Buff[1] == 0x51:
            if CheckSum(DataBuff[0:10], DataBuff[10]):
                Acceleration = [HexToShort(DataBuff[2:10], protocol_type)[i] for i in range(3)]
            else:
                print('0x51 Check failure')
        elif Buff[1] == 0x52:
            if CheckSum(DataBuff[0:10], DataBuff[10]):
                AngularVelocity = [HexToShort(DataBuff[2:10], protocol_type)[i] for i in range(3)]
            else:
                print('0x52 Check failure')
        elif Buff[1] == 0x53:
            if CheckSum(DataBuff[0:10], DataBuff[10]):
                AngleDegree = [HexToShort(DataBuff[2:10], protocol_type)[i] for i in range(3)]
                AngleFlag = True
            else:
                print('0x53 Check failure')
        elif Buff[1] == 0x54:
            if CheckSum(DataBuff[0:10], DataBuff[10]):
                Magnetometer = HexToShort(DataBuff[2:10], protocol_type)
            else:
                print('0x54 Check failure')
        else:
            Buff = {}
            Key = 0

    elif protocol_type == ProtocolType.CAN:
        raw_data = [DataBuff[2], DataBuff[3], DataBuff[4], DataBuff[5], DataBuff[6], DataBuff[7]]
        if Buff[1] == 0x51:
            Acceleration = HexToShort(raw_data, protocol_type)
        elif Buff[1] == 0x52:
            AngularVelocity = HexToShort(raw_data, protocol_type)
        elif Buff[1] == 0x53:
            AngleDegree = HexToShort(raw_data, protocol_type)
            AngleFlag = True  # 标记有新的欧拉角数据
        elif Buff[1] == 0x54:
            Magnetometer = HexToShort(raw_data, protocol_type)
        else:
            print(f"Unknown data type: 0x{Buff[1]:02x}")
    
    elif protocol_type == ProtocolType.RS485:
        if Key < 11:  # Modbus RTU返回通常11字节（地址、功能码、长度、数据6字节、CRC）
            return
        DataBuff = list(Buff.values())
        # 校验从机地址与功能码
        if DataBuff[1] != 0x03:
            Buff = {}
            Key = 0
            return
        
        # 提取6字节数据（Byte3~8）
        raw_data = [DataBuff[3], DataBuff[4], DataBuff[5], DataBuff[6], DataBuff[7], DataBuff[8]]
        values = HexToShort(raw_data, protocol_type)

        # 判断起始寄存器
        # Modbus返回帧中无寄存器号，这里我们用命令轮询顺序来匹配
        if not hasattr(HandleSerialData, "RS485_last_addr"):
            HandleSerialData.RS485_last_addr = 0x34  # 初始化为加速度寄存器

        last_addr = HandleSerialData.RS485_last_addr

        if last_addr == 0x34:
            Acceleration[:] = values
        elif last_addr == 0x37:
            AngularVelocity[:] = values
        elif last_addr == 0x3A:
            Magnetometer[:] = values
        elif last_addr == 0x3D:
            AngleDegree[:] = values
            AngleFlag = True

        # 更新记录
        HandleSerialData.RS485_last_addr = last_addr + 3 if last_addr < 0x3D else 0x34

        Buff = {}
        Key = 0
        return AngleFlag
    
    # 重置缓冲区
    Buff = {}
    Key = 0
    return AngleFlag

def GetQuaternionFromEuler(Roll, Pitch, Yaw):
    """
    Convert an Euler angle to a quaternion.

    Input
      :param roll: The roll (rotation around x-axis) angle in radians.
      :param pitch: The pitch (rotation around y-axis) angle in radians.
      :param yaw: The yaw (rotation around z-axis) angle in radians.

    Output
      :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
    """
    Qx = np.sin(Roll/2) * np.cos(Pitch/2) * np.cos(Yaw/2) - np.cos(Roll/2) * np.sin(Pitch/2) * np.sin(Yaw/2)
    Qy = np.cos(Roll/2) * np.sin(Pitch/2) * np.cos(Yaw/2) + np.sin(Roll/2) * np.cos(Pitch/2) * np.sin(Yaw/2)
    Qz = np.cos(Roll/2) * np.cos(Pitch/2) * np.sin(Yaw/2) - np.sin(Roll/2) * np.sin(Pitch/2) * np.cos(Yaw/2)
    Qw = np.cos(Roll/2) * np.cos(Pitch/2) * np.cos(Yaw/2) + np.sin(Roll/2) * np.sin(Pitch/2) * np.sin(Yaw/2)

    return [Qx, Qy, Qz, Qw]

# 
# IMU驱动节点
# 
class IMUDriverNode(Node):
    def __init__(self, PortName="/dev/imu_usb", BaudRate=9600, protocol_type=ProtocolType.TTL):
        super().__init__("imu_driver_node")
        self.PortName = PortName
        self.BaudRate = BaudRate
        self.protocol_type = protocol_type

        # 初始化IMU消息
        self.ImuMsg = Imu()
        self.ImuMsg.header.frame_id = "imu_link"

        # 创建IMU数据发布器
        self.ImuPublisher = self.create_publisher(Imu, "imu/data", 10)
        self.ImuMsgWithRPY = self.create_publisher(ImuData, "imu/ImuDataWithRPY", 10)

        # 启用IMU驱动线程
        self.DriverThread = threading.Thread(target=self.DriverLoop)
        self.DriverThread.start()

    # 
    # IMU驱动主循环
    #
    def DriverLoop(self):
        # 打开串口
        self.IMU_OpenPort()
        # 读取IMU数据
        self.IMU_ReadData()

    # 
    # 打开串口
    # 
    def IMU_OpenPort(self):
        try:
            self.WT_Imu = serial.Serial(self.PortName, self.BaudRate, timeout=0.5)
            if self.WT_Imu.isOpen():
                self.get_logger().info("\033[32mSerial port opened successfully...\033[0m")
            else:
                self.WT_Imu.open()
                self.get_logger().info("\033[32mSerial port opened successfully...\033[0m")
        except Exception as e:
            print(e)
            self.get_logger().info("\033[31mSerial port opening failure\033[0m")
            exit(0)

    # 
    # 轮询485模块
    # 
    def RS485_RequestData(self):
        # 轮询四种寄存器
        if not hasattr(self, "RS485_index"):
            self.RS485_index = 0
        keys = list(RS485_COMMANDS.keys())
        addr = keys[self.RS485_index]
        self.WT_Imu.write(RS485_COMMANDS[addr])
        self.RS485_index = (self.RS485_index + 1) % len(keys)

    # 
    # 读取IMU数据
    #
    def IMU_ReadData(self):
        while rclpy.ok():
            try:
                BuffCount = self.WT_Imu.in_waiting
            except Exception as e:
                print("exception:" + str(e))
                print("Imu Disconnect")
                exit(0)
            else:
                if(self.protocol_type == ProtocolType.RS485):
                    self.RS485_RequestData()
                    time.sleep(0.02)        # 50Hz
                if BuffCount > 0:
                    BuffData = self.WT_Imu.read(BuffCount)
                    for i in range(BuffCount):
                        tag = HandleSerialData(BuffData[i], self.protocol_type)
                        if tag:
                            # 处理IMU数据
                            self.Imu_ProcessData()
                            # 发布IMU数据
                            self.Imu_PublishData()

    #
    # 处理IMU数据
    #     
    def Imu_ProcessData(self):
        # 读取加速度数据
        Accel_x, Accel_y, Accel_z = Acceleration[0], Acceleration[1], Acceleration[2]
        AccelScale = 16 / 32768.0
        self.Accel_x, self.Accel_y, self.Accel_z = Accel_x * AccelScale, Accel_y * AccelScale, Accel_z * AccelScale

        # 读取陀螺仪数据
        Gyro_x, Gyro_y, Gyro_z = AngularVelocity[0], AngularVelocity[1], AngularVelocity[2]
        GyroScale = 2000 / 32768.0
        self.Gyro_x, self.Gyro_y, self.Gyro_z = math.radians(Gyro_x * GyroScale), math.radians(Gyro_y * GyroScale), math.radians(Gyro_z * GyroScale)

        # 读取欧拉角数据
        Angle_x, Angle_y, Angle_z = AngleDegree[0], AngleDegree[1], AngleDegree[2]
        AngleScale = 180 / 32768.0
        self.Roll, self.Pitch, self.Yaw = math.radians(Angle_x * AngleScale), math.radians(Angle_y * AngleScale), math.radians(Angle_z * AngleScale)
        
        # 计算四元数
        self.Quaternion = GetQuaternionFromEuler(self.Roll, self.Pitch, self.Yaw)
        
    # 
    # 发布IMU数据
    #
    def Imu_PublishData(self):
        # 发布IMU数据
        self.ImuMsg.header.stamp = self.get_clock().now().to_msg()
        self.ImuMsg.linear_acceleration.x = self.Accel_x
        self.ImuMsg.linear_acceleration.y = self.Accel_y
        self.ImuMsg.linear_acceleration.z = self.Accel_z
        self.ImuMsg.angular_velocity.x = self.Gyro_x
        self.ImuMsg.angular_velocity.y = self.Gyro_y
        self.ImuMsg.angular_velocity.z = self.Gyro_z
        self.ImuMsg.orientation.x = self.Quaternion[0]
        self.ImuMsg.orientation.y = self.Quaternion[1]
        self.ImuMsg.orientation.z = self.Quaternion[2]
        self.ImuMsg.orientation.w = self.Quaternion[3]

        # 自定义消息包
        imu_with_rpy = ImuData()
        imu_with_rpy.header = self.ImuMsg.header
        imu_with_rpy.imu = self.ImuMsg
        imu_with_rpy.roll  = math.degrees(self.Roll)
        imu_with_rpy.pitch = math.degrees(self.Pitch)
        imu_with_rpy.yaw   = math.degrees(self.Yaw)

        # 发布IMU消息
        self.ImuPublisher.publish(self.ImuMsg)
        self.ImuMsgWithRPY.publish(imu_with_rpy)

        print(f"roll: {math.degrees(self.Roll):.2f}°, pitch: {math.degrees(self.Pitch):.2f}°, yaw: {math.degrees(self.Yaw):.2f}°")

def main():
    # 初始化ROS 2节点
    rclpy.init()

    protocol = CURRENT_PROTOCOL
    node = IMUDriverNode("/dev/imu_usb", CURRENT_Buad, protocol)

    # 运行ROS 2节点
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # 停止ROS 2节点
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
