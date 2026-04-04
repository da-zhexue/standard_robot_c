#ifndef STANDARD_ROBOT_C_CHASSIS_H
#define STANDARD_ROBOT_C_CHASSIS_H

#include "pid.h"
#include "user_lib.h"
#include "motor/DJI/M3508/m3508.h"
#include "motor/LK/MF9025/mf9025.h"
#include "DBUS/dbus.h"
#include "IMU/INS/ins.h"
#include "Power_Ctrl/power_ctrl.h"
#include "Power_Ctrl/power_ctrl_param_get.h"
#include "Super_Cap/super_cap.h"
#include "NUC/nuc.h"
#include "Referee/referee.h"
#include "COMM/CBoard_chassis.h"

#define GIMBAL_ANGLE_DELTA_MAX 0.12f
#define CHASSIS_MAX_V 8.0f // 底盘最大线速度 m/s
#define CHASSIS_MAX_W 8.0f // 底盘最大角速度 rad/s

#define CHASSIS_CONTROL_TIME 0.003f // 控制周期 3ms
#define DWT_CLOCK_FREQ 168 // DWT时钟频率 168MHz
#define RAD_TO_DEG_FACTOR 5729.5779513f // 弧度转角度系数
#define DEG_PER_CIRCLE 360.0f // 一圈的角度
#define TORQUE_TO_CURRENT_FACTOR 0.3f // 扭矩转电流系数
#define SPIN_VW_SPEED 2.0f // 旋转摸鱼(小陀螺)角速度

#define REDUCTION_RATIO 19.2032f // 减速比
#define WHEEL_RADIUS 0.08f // 轮子半径，单位m
#define LINEAR_TO_RPM (PI * 2.0f / WHEEL_RADIUS * REDUCTION_RATIO) // 轮子线速度到电机转子转速比例系数
#define ROOT_2 1.41421356237309504880l

typedef enum
{
    CHASSIS_RC_OFFLINE = 0, // 遥控器掉线
    CHASSIS_RC = 1, // 遥控器控制
    CHASSIS_UPC = 2, // 上位机控制
    CHASSIS_SHUTDOWN = 3 // 关机状态
} Chassis_Controller;

// M3508 电机控制结构体
typedef struct
{
    pid_t pid; // PID控制器
    int16_t given_speed; // 给定速度
    fp32 out; // 输出值
} m3508_ctrl_t;

// MF9025 电机(云台)控制结构体
typedef struct
{
    pid_t pid; // PID控制器
    fp32 given_angle; // 给定角度
    fp32 cur_angle; // 当前角度
    fp32 ff_speed; // 前馈速度
} m9025_ctrl_t;

// 底盘控制数据中心
typedef struct
{
    uint8_t start; // 启动标志

    fp32 given_chassis_v[2]; // 给定底盘速度 [0]线速度大小 [1]线速度方向角度
    fp32 given_chassis_w; // 给定底盘自转角速度
    fp32 given_gimbal_l_yaw; // 给定大云台Yaw角度
    fp32 given_chassis_yaw; // 暂时不用

    first_order_filter_type_t chassis_speed_filter[3]; // 底盘速度一阶低通滤波 [0]x向 [1]y向 [2]角速度

    Chassis_Controller chassis_controller; // 控制模式
    uint8_t use_cap; // 使用超电标志
    fp32 maxpower; // 最大功率限制
    uint8_t chassis_spin; // 小陀螺模式标志
    uint8_t gimbal_scan; // 云台扫描模式标志
    uint8_t gimbal_shutdown_flag; // 云台关闭标志
    uint8_t last_gimbal_shutdown_flag; // 上次云台关闭标志

    m3508_ctrl_t m3508_controller[4]; // 4个轮毂电机控制数据
    m9025_ctrl_t m9025_controller; // 航向云台控制数据
} ctrl_data_t;

// 底盘系统总实例
typedef struct
{
    ctrl_data_t ctrl; // 核心控制数据

    rc_instance rc; // 遥控器实例
    m3508_instance m3508; // 3508电机实例
    mf9025_instance mf9025; // 9025电机实例
    INS_t ins; // 惯性导航系统实例
    PowerControllerConfig power_ctrl_config; // 功率控制配置
    PowerAllocationResult power_ctrl_result; // 功率分配结果
    PowerControlParam ctx; // 功率控制参数
    super_cap_instance super_cap; // 超级电容实例
    nuc_ctrl_t nuc_ctrl; // NUC控制实例
    referee_t referee; // 裁判系统实例
    cbord_chassis_t cbord_chassis; // 底盘通信实例

    angle_t chassis_angle; // 底盘角度数据
    angle_t gimbal_angle; // 云台角度数据
} chassis_t;

/**
 * @brief 底盘控制任务函数，运行在RTOS任务中
 */
void Chassis_Task(const void* argument);

/**
 * @brief 底盘初始化函数
 */
void Chassis_Init(chassis_t* chassis_ptr);

#endif //STANDARD_ROBOT_C_CHASSIS_H

