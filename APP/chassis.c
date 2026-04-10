#include "chassis.h"
#include <string.h>
#include "robot_config.h"
#include "arm_math.h"
#include "can.h"
#include "usart.h"
#include "user_lib.h"
#include "pid.h"
#include "dwt/bsp_dwt.h"

const fp32 filter_speed_alpha[3] = {0.2f, 0.2f, 0.2f};
const pid_config m3508_pid_config = {.mode = PID_POSITION, .kp = 8.0f, .ki = 0.0f, .kd = 0.0f, .max_out = 16000.0f, .max_iout = 3000.0f,
    .out_limit_delta_P = 1000.0f, .out_limit_delta_N = 4000.0f, .deadzone = 0.0f};
const pid_config chassisyaw_pid_config = {.mode = PID_POSITION, .kp = 10.0f, .ki = 0.0f, .kd = 0.0f, .max_out = 20000.0f, .max_iout = 5000.0f,
    .out_limit_delta_P = 1000.0f, .out_limit_delta_N = 4000.0f, .deadzone = 0.0f};
chassis_t chassis;

// ---------- 内部静态函数声明 ----------
static void chassis_filter_init(chassis_t* chassis_ptr);
static void chassis_pid_init(chassis_t* chassis_ptr);
static void chassis_switch_controller(chassis_t* chassis_ptr);
static void chassis_calc_move_speed(chassis_t* chassis_ptr);
static void chassis_calc_wheelmotor_speed(chassis_t* chassis_ptr);
static void chassis_calc_wheelmotor_pidout(chassis_t* chassis_ptr);
static void chassis_send_wheelmotor_cmd(const chassis_t* chassis_ptr);
/**
 * @brief 底盘初始化函数
 *        初始化各类传感器、电机控制、PID等
 */
void Chassis_Init(chassis_t* chassis_ptr)
{
    DWT_Init(DWT_CLOCK_FREQ);
    dbus_init(&chassis_ptr->rc, RC_CAN, &hcan2);
    m3508_init(&chassis_ptr->m3508, &hcan1, M3508_TX_1, 3);
    
    INS_Init(&chassis_ptr->ins);
    CBoard_Chassis_Init(&chassis_ptr->cbord_chassis, &hcan2);

    memset(&chassis_ptr->ctrl, 0, sizeof(chassis_ptr->ctrl));
    chassis_filter_init(chassis_ptr);
    chassis_pid_init(chassis_ptr);
}

/**
 * @brief 底盘控制任务，供FreeRTOS调用
 */
void Chassis_Task(const void* argument)
{
    Chassis_Init(&chassis);
    static uint16_t ctrl_loop = 0;
    while(1)
    {
        osDelay(1);
        // if (!chassis_check_game_start(&chassis)) continue;
        if (ctrl_loop % 3 == 0) // 控制循环 300Hz左右
        {

            chassis_switch_controller(&chassis);       // 切换控制源（遥控/上位机等）
            chassis_calc_move_speed(&chassis);         // 计算并过滤移动速度
            chassis_calc_wheelmotor_speed(&chassis);   // 解算底盘轮毂电机速度
            chassis_calc_wheelmotor_pidout(&chassis);  // 计算电机控制PID输出
            chassis_send_wheelmotor_cmd(&chassis);     // 下发3508控制指令
        }

        if (ctrl_loop % 10 == 0) // 通信循环 100Hz左右
        {
            ;
        }

        ctrl_loop++;
        if (ctrl_loop > 3000) ctrl_loop = 0;
    }
}

/**
 * @brief 底盘一阶低通滤波初始化
 */
static void chassis_filter_init(chassis_t* chassis_ptr)
{
    first_order_filter_init(&chassis_ptr->ctrl.chassis_speed_filter[0], CHASSIS_CONTROL_TIME, filter_speed_alpha[0]);
    first_order_filter_init(&chassis_ptr->ctrl.chassis_speed_filter[1], CHASSIS_CONTROL_TIME, filter_speed_alpha[1]);
    first_order_filter_init(&chassis_ptr->ctrl.chassis_speed_filter[2], CHASSIS_CONTROL_TIME, filter_speed_alpha[2]);
}

/**
 * @brief 初始化PID参数
 */
static void chassis_pid_init(chassis_t* chassis_ptr)
{
    for (int i = 0; i < 4; i++)
        PID_init(&chassis_ptr->ctrl.m3508_controller[i].pid, m3508_pid_config);
}

/**
 * @brief 切换底盘控制器（通过遥控器S1拨杆）
 */
static void chassis_switch_controller(chassis_t* chassis_ptr)
{
    chassis_ptr->ctrl.chassis_controller = chassis_ptr->rc.rc_data.s1;
}

/**
 * @brief 解算给定移动速度（获取控制源指令并滤波）
 */
static void chassis_calc_move_speed(chassis_t* chassis_ptr)
{
    fp32 vx, vy, vw;
    if (chassis_ptr->ctrl.chassis_controller == CHASSIS_RC)
    {
        chassis_ptr->ctrl.chassis_spin = 0;
        chassis_ptr->ctrl.gimbal_scan = 0;

        vx = (fp32)chassis_ptr->rc.rc_data.ch3 * CHASSIS_MAX_V / RC_VAL_MAX;
        vy = (fp32)chassis_ptr->rc.rc_data.ch2 * CHASSIS_MAX_V / RC_VAL_MAX;
        vw = (fp32)chassis_ptr->rc.rc_data.ch1 * CHASSIS_MAX_V / RC_VAL_MAX;
    }
    else if (chassis_ptr->ctrl.chassis_controller == CHASSIS_UPC)
    {
        chassis_ptr->ctrl.chassis_spin = chassis_ptr->nuc_ctrl.chassis_spin;

        vx = loop_float_constrain(chassis_ptr->nuc_ctrl.vx, -CHASSIS_MAX_V, CHASSIS_MAX_V);
        vy = loop_float_constrain(chassis_ptr->nuc_ctrl.vy, -CHASSIS_MAX_V, CHASSIS_MAX_V);
        vw = loop_float_constrain(chassis_ptr->nuc_ctrl.vw, -CHASSIS_MAX_V, CHASSIS_MAX_V);

        if (chassis_ptr->ctrl.chassis_spin)
            vw = SPIN_VW_SPEED;
    }
    else
    {
        chassis_ptr->ctrl.chassis_spin = 0;
        chassis_ptr->ctrl.gimbal_scan = 0;

        vx = 0;
        vy = 0;
        vw = 0;
    }

    // 速度滤波
    first_order_filter_cali(&chassis_ptr->ctrl.chassis_speed_filter[0], vx);
    first_order_filter_cali(&chassis_ptr->ctrl.chassis_speed_filter[1], vy);
    first_order_filter_cali(&chassis_ptr->ctrl.chassis_speed_filter[2], vw);
    const fp32 vx_filter = chassis_ptr->ctrl.chassis_speed_filter[0].out;
    const fp32 vy_filter = chassis_ptr->ctrl.chassis_speed_filter[1].out;
    const fp32 vw_filter = chassis_ptr->ctrl.chassis_speed_filter[2].out;

    // 转换为合速度大小与角度（极坐标格式存储进 given_chassis_v）
    fp32 norm_v;
    arm_sqrt_f32(vx_filter * vx_filter + vy_filter * vy_filter, &norm_v);
    chassis_ptr->ctrl.given_chassis_v[0] = norm_v * LINEAR_TO_RPM;
    chassis_ptr->ctrl.given_chassis_v[1] = atan2f(vy_filter, vx_filter);
    chassis_ptr->ctrl.given_chassis_w = vw_filter * LINEAR_TO_RPM;
}

/**
 * @brief 全向轮底盘运动学解算函数
 * @param vx x向速度 m/s
 * @param vy y向速度 m/s
 * @param vw 自转角速度 rad/s
 * @param spin_yaw 当前Yaw向扭矩偏移
 * @param motor_speeds 输出到4个电机的目标转速
 */
void Omni_Kinematics_Resolve(const fp32 vx, const fp32 vy, const fp32 vw, const fp32 spin_yaw, int16_t* motor_speeds)
{
    const fp32 sin_yaw = sinf(radian_format(spin_yaw));
    const fp32 cos_yaw = cosf(radian_format(spin_yaw));
    const fp32 vx_set = vx * cos_yaw - vy * sin_yaw;
    const fp32 vy_set = vx * sin_yaw + vy * cos_yaw;

    motor_speeds[0] = (int16_t)((vx_set + vy_set) / ROOT_2 + vw);
    motor_speeds[1] = (int16_t)((-vx_set + vy_set) / ROOT_2 + vw);
    motor_speeds[2] = (int16_t)((-vx_set - vy_set) / ROOT_2 + vw);
    motor_speeds[3] = (int16_t)((vx_set - vy_set) / ROOT_2 + vw);
}

/**
 * @brief 解算3508电机的目标速度（给四个轮毂下发指令速度）
 */
static void chassis_calc_wheelmotor_speed(chassis_t* chassis_ptr)
{
    const fp32 chassis_v = chassis_ptr->ctrl.given_chassis_v[0];
    const fp32 theta = chassis_ptr->ctrl.given_chassis_v[1];
    const fp32 chassis_w = chassis_ptr->ctrl.given_chassis_w + chassis_ptr->gimbal_angle.yaw_rad;
    const fp32 vx = chassis_v * cosf(theta);
    const fp32 vy = chassis_v * sinf(theta);
    int16_t motor_speeds[4];
#ifdef OMNI
    Omni_Kinematics_Resolve(vx, vy, chassis_w, chassis_ptr->chassis_angle.yaw_rad, motor_speeds);
#endif

    chassis_ptr->ctrl.m3508_controller[0].given_speed = motor_speeds[0];
    chassis_ptr->ctrl.m3508_controller[1].given_speed = motor_speeds[1];
    chassis_ptr->ctrl.m3508_controller[2].given_speed = motor_speeds[2];
    chassis_ptr->ctrl.m3508_controller[3].given_speed = motor_speeds[3];
}

/**
 * @brief PID控制律计算，包括3508速度环的PID计算和MF9025航向角度环的PID计算
 */
static void chassis_calc_wheelmotor_pidout(chassis_t* chassis_ptr)
{
    for (int i = 0; i < 4; i++)
    {
        PID_calc(&chassis_ptr->ctrl.m3508_controller[i].pid, chassis_ptr->m3508.ecd[i].speed, chassis_ptr->ctrl.m3508_controller[i].given_speed);
        chassis_ptr->ctrl.m3508_controller[i].out = chassis_ptr->ctrl.m3508_controller[i].pid.out[0];
    }
}


/**
 * @brief 将控制计算结果下发到CAN总线上
 */
static void chassis_send_wheelmotor_cmd(const chassis_t* chassis_ptr)
{
    int16_t m3508_iq[4];
    for (int i = 0; i < 4; i++)
        m3508_iq[i] = (int16_t)chassis_ptr->ctrl.m3508_controller[i].out;
    m3508_ctrl(&chassis_ptr->m3508, m3508_iq);
}
