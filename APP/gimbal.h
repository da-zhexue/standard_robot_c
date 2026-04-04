#ifndef STANDARD_ROBOT_C_GIMBAL_H
#define STANDARD_ROBOT_C_GIMBAL_H
#include "pid.h"
#include "typedef.h"
#include "user_lib.h"
#include "motor/DJI/GM6020/gm6020.h"
#include "motor/DJI/M2006/m2006.h"
#include "motor/DJI/M3508/m3508.h"
#include "COMM/CBoard_gimbal.h"
#include "NX/nx.h"
#include "DBUS/dbus.h"
#include "IMU/HI12/hi12.h"

#define SMALL_GIMBAL_ANGLE_DELTA_MAX 0.04f
#define TRIGGER_SPEED_MAX 1000.0f
#define FRICTION_SPEED_MAX 10000.0f
#define FRICTION_READY_SPEED 3000
#define YAW_LIMIT 30.0f
#define PITCH_LIMIT 20.0f
#define ECD_YAW_OFFSET 0
#define ECD_PITCH_OFFSET 0

typedef enum
{
    GIMBAL_RC_OFFLINE = 0, // 遥控器掉线
    GIMBAL_RC = 3, // 遥控器控制
    GIMBAL_UPC = 2, // 上位机控制
    GIMBAL_SHUTDOWN = 1 // 关机状态
} Gimbal_Controller;

// M3508 电机控制结构体
typedef struct
{
    pid_t pid; // PID控制器
    int16_t given_speed; // 给定速度
    fp32 out; // 输出值
} friction_ctrl_t;

// M2006 电机控制结构体
typedef struct
{
    pid_t pid; // PID控制器
    int16_t given_speed; // 给定速度
    fp32 out; // 输出值
} trigger_ctrl_t;

// GM6020 电机控制结构体
typedef struct
{
    pid_t pid; // PID控制器
    int16_t given_speed; // 给定速度
    fp32 out; // 输出值
} angle_ctrl_t;

typedef struct
{
    Gimbal_Controller gimbal_controller;
    fp32 target_yaw;
    fp32 target_pitch;
    uint8_t open_friction;
    uint8_t friction_ready;
    uint8_t fire;

    friction_ctrl_t friction_ctrl[2];
    trigger_ctrl_t trigger_ctrl;
    angle_ctrl_t angle_ctrl[2];
} gimbal_ctrl_t;

typedef struct
{
    gimbal_ctrl_t ctrl;
    gm6020_instance angle_motor;
    m2006_instance trigger_motor;
    m3508_instance friction_motor;
    cboard_gimbal_t cboard_gimbal;
    nx_ctrl_t nx_ctrl;
    rc_instance rc;
    hi12_t hi12;

    angle_t angle;
} gimbal_t;

void Gimbal_Task(const void* argument);

#endif //STANDARD_ROBOT_C_GIMBAL_H