#ifndef STANDARD_ROBOT_C_CHASSIS_H
#define STANDARD_ROBOT_C_CHASSIS_H

#include "pid.h"
#include "user_lib.h"
#include "motor/DJI/M3508/m3508.h"
#include "motor/LK/MF9025/mf9025.h"
#include "DBUS/dbus.h"
#include "IMU/INS/ins.h"
#include "COMM/comm.h"
#include "Power_Ctrl/power_ctrl.h"
#include "Power_Ctrl/power_ctrl_param_get.h"

#define GIMBAL_ANGLE_DELTA_MAX 0.12f
#define CHASSIS_MAX_V 8.0f
#define CHASSIS_MAX_W 8.0f

#define CHASSIS_CONTROL_TIME 0.003f

#define REDUCTION_RATIO 19.2032f // 减速比
#define WHEEL_RADIUS 0.08f // 轮子半径，单位m
#define LINEAR_TO_RPM (PI * 2.0f / WHEEL_RADIUS * REDUCTION_RATIO) // 轮子线速度到电机转子转速比例系数
#define ROOT_2 1.41421356237309504880l

typedef enum
{
    CHASSIS_RC_OFFLINE = 0,
    CHASSIS_RC = 1,
    CHASSIS_UPC = 2,
    CHASSIS_SHUTDOWN = 3
} CHASSIS_CONTROLLER;

typedef struct
{
    pid_t pid;
    int16_t given_speed;
} m3508_ctrl_t;
typedef struct
{
    pid_t pid;
    fp32 given_angle;
    fp32 cur_angle;
    fp32 ff_speed;
} m9025_ctrl_t;

typedef struct
{
    uint8_t start;

    fp32 given_chassis_v[2];
    fp32 given_chassis_w;
    fp32 given_gimbal_l_yaw;
    fp32 given_chassis_yaw; // 暂时不用

    first_order_filter_type_t chassis_speed_filter[3];

    CHASSIS_CONTROLLER chassis_controller;
    uint8_t chassis_spin;
    uint8_t gimbal_scan;
    uint8_t gimbal_shutdown_flag;
    uint8_t last_gimbal_shutdown_flag;

    m3508_ctrl_t m3508_controller[4];
    m9025_ctrl_t m9025_controller;
} ctrl_data_t;

typedef struct
{
    ctrl_data_t ctrl;

    rc_instance* rc;
    m3508_instance* m3508;
    mf9025_instance* mf9025;
    INS_t* ins;
    comm_t* comm;

    angle_t chassis_angle;
    angle_t gimbal_angle;
} chassis_t;

void chassis_task(const void* argument);

#endif //STANDARD_ROBOT_C_CHASSIS_H