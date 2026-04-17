#ifndef STANDARD_ROBOT_C_CHASSIS_H
#define STANDARD_ROBOT_C_CHASSIS_H

#include "pid.h"
#include "user_lib.h"
#include "motor/DJI/M3508/m3508.h"
#include "DBUS/dbus.h"
#include "IMU/INS/ins.h"
#include "Odom/odom.h"
#include "NX/nx.h"
#include "motor/RS/rs02.h"

#define PITCH_DELTA_MAX (-0.000040f)
#define CHASSIS_MAX_V 2.0f // 底盘最大线速度 m/s
#define CHASSIS_MAX_W 2.0f // 底盘最大角速度 rad/s

#define CHASSIS_CONTROL_TIME 0.003f // 控制周期 3ms
#define DWT_CLOCK_FREQ 168 // DWT时钟频率 168MHz
#define RAD_TO_DEG_FACTOR 5729.5779513f // 弧度转角度系数
#define DEG_PER_CIRCLE 360.0f // 一圈的角度
#define TORQUE_TO_CURRENT_FACTOR 0.3f // 扭矩转电流系数

#define REDUCTION_RATIO 19.2032f // 减速比
#define WHEEL_RADIUS 0.08f // 轮子半径，单位m
#define LINEAR_TO_RPM (PI * 2.0f / WHEEL_RADIUS * REDUCTION_RATIO) // 轮子线速度到电机转子转速比例系数
#define ROOT_2 1.41421356237309504880l

#define MIT_HIT_TORCH 3.0f
#define MIT_SHOOT_TORCH 5.0f
#define MIT_PITCH_TORCH 1.5f
#define MIT_KP 10.0f
#define MIT_KD 0.5f

#define ms_to_us(ms) ((uint64_t)(ms * 1000))

typedef enum
{
    CHASSIS_RC_OFFLINE = 0, // 遥控器掉线
    CHASSIS_RC = 1, // 遥控器控制
    CHASSIS_UPC = 2, // 上位机控制
    CHASSIS_SHUTDOWN = 3 // 关机状态
} Robot_Controller;

typedef enum
{
    SHOOT_SERVE = 0, // 发球
    SHOOT_RECEIVE = 1, // 接球
    SHOOT_STOP = 2 // 停止发接球
} Shoot_Mode;

typedef enum
{
    SHOOT_DEFAULT = 0, // 静止状态
    SHOOT_PREPARE = 1, // 发球准备状态，出限位一点
    SHOOT_DELAY_1 = 2, // 发球延时状态1
    SHOOT_HIT = 3, // 高速击球
    SHOOT_DELAY_2 = 4, // 发球延时状态2
    SHOOT_LEANING = 5, // 云台前倾，防止被发球板打到
    SHOOT_DELAY_3 = 6, // 发球延时状态3
    SHOOT_SHOOT = 7, // 发球板击球
    SHOOT_DELAY_4 = 8, // 发球延时状态4
    SHOOT_BACK = 9, // 发球板回到原位
    SHOOT_DELAY_5 = 10 // 发球延时状态5

} Shoot_step;

// M3508 电机控制结构体
typedef struct
{
    pid_t pid; // PID控制器
    int16_t given_speed; // 给定速度
    fp32 out; // 输出值
} m3508_ctrl_t;

// 底盘控制数据中心
typedef struct
{
    fp32 given_chassis_v[2]; // 给定底盘速度 [0]线速度大小 [1]线速度方向角度
    fp32 given_chassis_w; // 给定底盘自转角速度
    fp32 given_yaw; // 给定底盘Yaw角度
    fp32 given_pitch;
    fp32 given_hit_pos, given_hit_speed, given_shoot_angle; // 给定击发球位置和速度
    uint8_t shoot;
    Shoot_step shoot_step;

    first_order_filter_type_t chassis_speed_filter[3]; // 底盘速度一阶低通滤波 [0]x向 [1]y向 [2]角速度

    Robot_Controller robot_controller; // 控制模式
    Shoot_Mode shoot_mode; // 发接球模式

    m3508_ctrl_t m3508_controller[3]; // 3个轮毂电机控制数据

} ctrl_data_t;

// 底盘系统总实例
typedef struct
{
    ctrl_data_t ctrl; // 核心控制数据

    rc_instance rc; // 遥控器实例
    m3508_instance m3508; // 3508电机实例
    rs02_instance rs02[5]; // RS02电机实例
    INS_t ins; // 惯性导航系统实例
    nx_ctrl_t nx_ctrl;
    odom_t odom; // 里程计实例
} robot_t;

/**
 * @brief 底盘控制任务函数，运行在RTOS任务中
 */
void Control_Task(const void* argument);

#endif //STANDARD_ROBOT_C_CHASSIS_H

