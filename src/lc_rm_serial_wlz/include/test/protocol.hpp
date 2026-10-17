#pragma once
#include <cstdint>
#include <iostream>

namespace lc_serial
{   

//协议常量
constexpr uint8_t PROTOCOL_SOF = 0x11;
constexpr uint16_t CMD_ID_SEND = 0x0001;
constexpr size_t TX_DATA_LEN = 20;
constexpr size_t RX_DATA_LEN = 12;
constexpr size_t TX_FRAME_LEN = 5+2+TX_DATA_LEN+2;//上位机发送消息的长度
constexpr size_t RX_FRAME_LEN = 5+2+RX_DATA_LEN+2;//下位机发送消息的长度
constexpr uint16_t CHASSIS_CMD_SHIFT = 0;
constexpr uint16_t GIMBAL_CMD_SHIFT  = 4;
constexpr uint16_t SHOOT_CMD_SHIFT   = 8;

//命令码的枚举
enum class ChassisCmd : uint16_t
{
    OFF = 0,
    NO_FOLLOWING = 1,
    FOLLOWING = 2

};

enum class GimbalCmd : uint16_t
{
    OFF = 0,
    ENABLE = 1,
    SMALL_TOP = 2
};

enum class ShootCmd : uint16_t
{
    OFF = 0,
    FICTION_ON = 1,
    FIRE = 2
};

//数据结构体
//发送结构体
struct TxData
{
    float forward;
    float turn;
    float rotate;
    float yaw;
    float pitch;

    /* data */
};

//接收结构体
struct RxData
{
    float yaw;
    float pitch;
    float free_data;
};

//解析后的完整接收帧
struct RxFrame
{
    uint8_t seq;
    uint16_t cmd_id;
    RxData data;
    /* data */
};

}


