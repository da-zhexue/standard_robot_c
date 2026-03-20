#ifndef STANDARD_ROBOT_C_COMM_H
#define STANDARD_ROBOT_C_COMM_H
#include "typedef.h"
#include "usart/bsp_uart.h"
#include "user_lib.h"
#define SOF_VALUE       0xA5
#define FRAME_HEADER_LEN  5
#define CMD_ID_LEN        2
#define CRC16_LEN         2

typedef enum
{
    SEND_PARAM = 0x200,
    SEND_ATTITUDE = 0x201,
    SEND_ONLINE = 0x202,
    SEND_START = 0x203,
    SEND_ONLINECB = 0x207,
    SEND_SUPERCAP = 0x208,
    SEDN_DEBUG = 0x233,

    // CMD_IMU_S_INFO = 0x101,
    CMD_IMU_L_INFO = 0x102,
    CMD_MOVE = 0x103,
    CMD_ROTATE = 0x104,
    CMD_MODE = 0x105,
    CMD_START = 0x106,
    CMD_ONLINECB = 0x107,
    CMD_POWER = 0x108,
    CMD_GIMBAL_ROTATE = 0x109,
    CMD_DEBUG = 0x133,

} comm_cmd_t;

typedef struct
{
    fp32 vx, vy, vw;
    fp32 gimbal_yaw, chassis_yaw;
    uint8_t gimbal_scan;
    uint8_t chassis_spin;
} comm_ctrl_t;

typedef struct
{
    UART_Instance_t uart_instance;
    uint8_t game_start;
    uint8_t comm_ctrl;
    comm_ctrl_t comm_ctrl_param;
    fp32 big_gimbal_angle[3];
} comm_t;

void comm_init(comm_t* comm_instance, UART_HandleTypeDef* comm_uart);

void comm_send_attitude(fp32 chassis_yaw);
void comm_send_onlinestate();
void comm_send_cap();


#endif //STANDARD_ROBOT_C_COMM_H