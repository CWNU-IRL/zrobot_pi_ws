#pragma once

#include <atomic>
#include <iostream>
#include <cstring>
#include <cstdio>
#include <cerrno>
#include <cmath>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <thread>
#include <chrono>
#include <ctime>
#include <optional>
#include <tuple>
#include <string>
#include <vector>
#include <map>
#include <bitset>
#include <iomanip>

#define Set_mode 	  'j' //设置控制模式
#define Set_parameter 'p' //设置参数
//各种控制模式
#define move_control_mode  0 //运控模式
#define PosPP_control_mode   1 //位置模式
#define Speed_control_mode 2 //速度模式
#define Elect_control_mode 3 //电流模式
#define Set_Zero_mode      4 //零点模式
#define PosCSP_control_mode 5 //位置模式CSP

#define SC_MAX 23.0f
#define SC_MIN 0.0f
#define SV_MAX 20.0f
#define SV_MIN -20.0f
#define SCIQ_MIN -23.0f

//通信地址
#define Communication_Type_Get_ID 0x00  //获取设备的ID和64位MCU唯一标识符`
#define Communication_Type_MotionControl 0x01  //运控模式用来向主机发送控制指令
#define Communication_Type_MotorRequest 0x02  //用来向主机反馈电机运行状态
#define Communication_Type_MotorEnable 0x03  //电机使能运行
#define Communication_Type_MotorStop 0x04  //电机停止运行
#define Communication_Type_SetPosZero 0x06  //设置电机机械零位
#define Communication_Type_Can_ID 0x07  //更改当前电机CAN_ID
#define Communication_Type_Control_Mode 0x12  //设置电机模式
#define Communication_Type_GetSingleParameter 0x11  //读取单个参数
#define Communication_Type_SetSingleParameter 0x12  //设定单个参数
#define Communication_Type_ErrorFeedback 0x15  //故障反馈帧

// 定义返回类型（需要 C++17）
using ReceiveResult = std::optional<std::tuple<uint8_t, uint16_t, uint8_t, std::vector<uint8_t>>>;

typedef union
{
    float f;
    unsigned char c[4];
} float2uchar;

typedef struct
{
    int set_motor_mode;
    float set_current;
    float set_speed;
    float set_Torque;
    float set_angle;
    float set_limit_cur;
    float set_Kp;
    float set_Ki;
    float set_Kd;
    float set_iq;
    float set_id;
    float set_acc;
} Motor_Set;

class data_read_write_one
{
public:
    uint16_t index;
    float data;
};

//---------------------------------------------
// 电机类型定义
//---------------------------------------------
enum class ActuatorType
{
    ROBSTRIDE_00 = 0,
    ROBSTRIDE_01 = 1,
    ROBSTRIDE_02 = 2,
    ROBSTRIDE_03 = 3,
    ROBSTRIDE_04 = 4,
    ROBSTRIDE_05 = 5,
    ROBSTRIDE_06 = 6
};

//---------------------------------------------
// 电机运行参数结构体
//---------------------------------------------
struct ActuatorOperation
{
    double position; // rad
    double velocity; // rad/s
    double torque; // Nm
    double kp;
    double kd;
};

//---------------------------------------------
// 电机类型对应运行参数映射
//---------------------------------------------
static const std::map<ActuatorType, ActuatorOperation> ACTUATOR_OPERATION_MAPPING = {
    {ActuatorType::ROBSTRIDE_00, {4 * M_PI, 50, 17, 500.0, 5.0,}},
    {ActuatorType::ROBSTRIDE_01, {4 * M_PI, 44, 17, 500.0, 5.0,}},
    {ActuatorType::ROBSTRIDE_02, {4 * M_PI, 44, 17, 500.0, 5.0,}},
    {ActuatorType::ROBSTRIDE_03, {4 * M_PI, 50, 60, 5000.0, 100.0,}},
    {ActuatorType::ROBSTRIDE_04, {4 * M_PI, 15, 120, 5000.0, 100.0,}},
    {ActuatorType::ROBSTRIDE_05, {4 * M_PI, 33, 17, 500.0, 5.0,}},
    {ActuatorType::ROBSTRIDE_06, {4 * M_PI, 20, 60, 5000.0, 100.0,}},
};

static const uint16_t Index_List[] = {
    0X7005, 0X7006, 0X700A, 0X700B, 0X7010, 0X7011, 0X7014, 0X7016, 0X7017, 0X7018, 0x7019, 0x701A, 0x701B, 0x701C,
    0x701D
};
//18通信类型可以写入的参数列表
//参数变量名  参数地址  描述  类型  字节数  单位/说明
class data_read_write
{
public:
    data_read_write_one run_mode; //0:运控模式 1:位置模式（PP） 2:速度模式 3:电流模式 4:零点模式 5:位置模式（CSP） uint8  1byte
    data_read_write_one iq_ref; //电流模式Iq指令  float 	4byte 	-23~23A
    data_read_write_one spd_ref; //转速模式转速指令  float 	4byte 	-30~30rad/s
    data_read_write_one imit_torque; //转矩限制  float 	4byte 	0~12Nm
    data_read_write_one cur_kp; //电流的 Kp  float 	4byte 	默认值 0.125
    data_read_write_one cur_ki; //电流的 Ki  float 	4byte 	默认值 0.0158
    data_read_write_one cur_filt_gain; //电流滤波系数filt_gain  float 	4byte 	0~1.0，默认值0.1
    data_read_write_one loc_ref; //位置模式角度指令  float 	4byte 	rad
    data_read_write_one limit_spd; //位置模式速度设置  float 	4byte 	0~30rad/s
    data_read_write_one limit_cur; //速度位置模式电流设置  float 	4byte 	0~23A
    //以下只可读
    data_read_write_one mechPos; //负载端计圈机械角度  float 	4byte 	rad
    data_read_write_one iqf; //iq 滤波值  float 	4byte 	-23~23A
    data_read_write_one mechVel; //负载端转速  float 	4byte 	-30~30rad/s
    data_read_write_one VBUS; //母线电压  float 	4byte 	V
    data_read_write_one rotation; //圈数  int16 	2byte   圈数
    data_read_write(const uint16_t* index_list = Index_List)
    {
        run_mode.index = index_list[0];
        iq_ref.index = index_list[1];
        spd_ref.index = index_list[2];
        imit_torque.index = index_list[3];
        cur_kp.index = index_list[4];
        cur_ki.index = index_list[5];
        cur_filt_gain.index = index_list[6];
        loc_ref.index = index_list[7];
        limit_spd.index = index_list[8];
        limit_cur.index = index_list[9];
        mechPos.index = index_list[10];
        iqf.index = index_list[11];
        mechVel.index = index_list[12];
        VBUS.index = index_list[13];
        rotation.index = index_list[14];
    }
};

class RobStrideMotor
{
public:
    RobStrideMotor(const std::string can_interface, uint8_t master_id, uint8_t motor_id, int actuator_type)
        : iface(can_interface), master_id(master_id), motor_id(motor_id), actuator_type(actuator_type)
    {
        init_socket();
    }

    ~RobStrideMotor()
    {
        if (socket_fd >= 0)
            close(socket_fd);
    }

    // 等待可读并收一帧；超时/短帧/非扩展帧返回空，不抛异常。
    ReceiveResult receive(double timeout_sec = 0);

    std::tuple<float, float, float, float> return_data_pvtt()
    {
        return std::make_tuple(position_, velocity_, torque_, temperature_);
    }

    // 收一帧并解析；收到任何有效帧（含参数回帧）返回 true。
    bool receive_status_frame(int timeout_ms = 5);

    // 供 tick 使用：只有收到 MotorRequest 反馈帧并刷新位置/速度/力矩/温度才返回 true。
    bool poll_status(int timeout_ms);

    void Set_RobStrite_Motor_parameter(uint16_t Index, float Value, char Value_mode);

    std::tuple<float, float, float, float> send_velocity_mode_command(float velocity_rad_s);

    // 发送使能指令（通信类型3）
    std::tuple<float, float, float, float> enable_motor();

    float read_initial_position();

    void init_socket();

    uint16_t float_to_uint(float x, float x_min, float x_max, int bits);
    // 发送运控模式（控制角度 + 速度 + KP + KD）
    // wait_for_reply=false 时只写不等，供稳态 tick 使用。
    std::tuple<float, float, float, float> send_motion_command(float torque,
                                                               float position_rad,
                                                               float velocity_rad_s,
                                                               float kp = 0.5f,
                                                               float kd = 0.1f,
                                                               bool wait_for_reply = true);

    float uint_to_float(uint16_t x_int, float x_min, float x_max, int bits)
    {
        float span = x_max - x_min;
        return ((float)x_int) * span / ((1 << bits) - 1) + x_min;
    }

    void Get_RobStrite_Motor_parameter(uint16_t Index);
    // 位置模式（PP）
    std::tuple<float, float, float, float> RobStrite_Motor_PosPP_control(float Speed, float Acceleration, float Angle);
    // 位置模式（CSP）
    std::tuple<float, float, float, float> RobStrite_Motor_PosCSP_control(float Speed, float Angle);
    // 电流模式（Iq + Id）
    std::tuple<float, float, float, float> RobStrite_Motor_Current_control(float IqCommand, float IdCommand);
    // 设置零点模式
    void RobStrite_Motor_Set_Zero_control();
    // 发送失能指令
    void Disenable_Motor(uint8_t clear_error);
    void Set_CAN_ID(uint8_t Set_CAN_ID);
    // 设置机械零点
    void Set_ZeroPos();

    float Byte_to_float(uint8_t* bytedata)
    {
        uint32_t data = bytedata[7] << 24 | bytedata[6] << 16 | bytedata[5] << 8 | bytedata[4];
        float data_float = *(float*)(&data);
        return data_float;
    }

    float Byte_to_float(const std::vector<uint8_t>& bytedata)
    {
        if (bytedata.size() < 8) return 0.0f; // 防止越界
        uint32_t data = (bytedata[7] << 24) | (bytedata[6] << 16) | (bytedata[5] << 8) | bytedata[4];
        float data_float;
        std::memcpy(&data_float, &data, sizeof(float)); // 比 *(float*)&data 更安全
        return data_float;
    }

private:
    ReceiveResult receive_frame(int timeout_ms);
    bool write_frame(const can_frame& frame);
    bool parse_status_frame(const ReceiveResult& result, bool* got_motor_request);
    void log_error_throttled(const char* message);

public:
    std::string iface;
    uint8_t master_id;
    uint8_t motor_id;
    int socket_fd = -1;
    Motor_Set Motor_Set_All; // 设定值
    data_read_write_one params;
    data_read_write drw;

    float position_ = 0.0;
    float velocity_ = 0.0;
    float torque_ = 0.0;
    float temperature_ = 0.0;

    uint8_t error_code = 0;
    uint8_t pattern = 0;
    std::atomic<bool> is_move_control_first = true;
    int actuator_type;

    int fd() const { return socket_fd; }
    uint64_t tx_failures() const { return tx_failures_; }
    uint64_t feedback_count() const { return feedback_count_; }

private:
    uint64_t tx_failures_ = 0;
    uint64_t feedback_count_ = 0;
    std::chrono::steady_clock::time_point last_error_log_{};
    std::chrono::steady_clock::time_point last_mode_log_{};
};
