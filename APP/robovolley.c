#include "robovolley.h"
#include <string.h>
#include "robot_config.h"
#include "arm_math.h"
#include "can.h"
#include "usart.h"
#include "user_lib.h"
#include "pid.h"
#include "dwt/bsp_dwt.h"
#include "NX/nx.h"

const fp32 filter_speed_alpha[3] = {0.2f, 0.2f, 0.2f};
const pid_config m3508_pid_config = {.mode = PID_POSITION, .kp = 8.0f, .ki = 0.0f, .kd = 0.0f, .max_out = 16000.0f, .max_iout = 3000.0f,
    .out_limit_delta_P = 1000.0f, .out_limit_delta_N = 4000.0f, .deadzone = 0.0f};
const pid_config chassisyaw_pid_config = {.mode = PID_POSITION, .kp = 10.0f, .ki = 0.0f, .kd = 0.0f, .max_out = 20000.0f, .max_iout = 5000.0f,
    .out_limit_delta_P = 1000.0f, .out_limit_delta_N = 4000.0f, .deadzone = 0.0f};
robot_t chassis;

// ---------- 内部静态函数声明 ----------
static void chassis_filter_init(robot_t* chassis_ptr);
static void chassis_pid_init(robot_t* chassis_ptr);
static void chassis_switch_controller(robot_t* chassis_ptr);
static void chassis_calc_move_speed(robot_t* chassis_ptr);
static void chassis_calc_wheelmotor_speed(robot_t* chassis_ptr);
static void chassis_calc_wheelmotor_pidout(robot_t* chassis_ptr);
static void chassis_send_wheelmotor_cmd(const robot_t* chassis_ptr);
static void rc_offline_process(robot_t* chassis_ptr);
static void gimbal_calc_pitch_target(robot_t* chassis_ptr);
static void gimbal_calc_shoot_target(robot_t* chassis_ptr);
static void gimbal_shoot_process(robot_t* chassis_ptr);
static void gimbal_send_cmd(const robot_t* chassis_ptr);
static void chasssis_send_pos(const robot_t* chassis_ptr);
/**
 * @brief 底盘初始化函数
 *        初始化各类传感器、电机控制、PID等
 */
void Chassis_Init(robot_t* chassis_ptr)
{
    DWT_Init(DWT_CLOCK_FREQ);
    dbus_init(&chassis_ptr->rc, RC_CAN, &hcan1);
    m3508_init(&chassis_ptr->m3508, &hcan1, M3508_TX_1, 3);
    for (uint8_t i = 0; i < 5; i++)
        rs02_init(&chassis_ptr->rs02[i], &hcan1, i + 0x01, i + 0xF0, RS02_MODE_MIT, RS02_PROTOCOL_PRIVATE);
    for (uint8_t i = 0; i < 5; i++)
    {
        rs02_set_mitctrl_private(&chassis_ptr->rs02[i]);
        osDelay(1);
        rs02_enable_private(&chassis_ptr->rs02[i]);
        osDelay(1);
    }
    static IMU_Data_t bmi088;
    BMI088_Init(&bmi088);
    INS_Init(&chassis.ins, &bmi088);
    Odom_Init(&chassis_ptr->odom, &huart1);
    NX_Init(&chassis_ptr->nx_ctrl, &hcan2);

    memset(&chassis_ptr->ctrl, 0, sizeof(chassis_ptr->ctrl));
    chassis_filter_init(chassis_ptr);
    chassis_pid_init(chassis_ptr);
}

/**
 * @brief 底盘控制任务，供FreeRTOS调用
 */
void Control_Task(const void* argument)
{
    Chassis_Init(&chassis);
    static uint16_t ctrl_loop = 0;
    while(1)
    {
        osDelay(1);
        if (ctrl_loop % 3 == 0) // 控制循环 300Hz左右
        {
            chassis_switch_controller(&chassis);       // 切换控制源（遥控/上位机等）
            chassis_calc_move_speed(&chassis);         // 计算并过滤移动速度
            chassis_calc_wheelmotor_speed(&chassis);   // 解算底盘轮毂电机速度
            chassis_calc_wheelmotor_pidout(&chassis);  // 计算电机控制PID输出
            chassis_send_wheelmotor_cmd(&chassis);     // 下发3508控制指令
        }
        if (ctrl_loop % 3 == 1) // 云台控制循环 300Hz左右
        {
            chassis_switch_controller(&chassis);       // 切换控制源（遥控/上位机等）
            rc_offline_process(&chassis);          // 处理遥控器离线情况
            gimbal_calc_pitch_target(&chassis); // 计算云台pitch轴目标位置
            gimbal_calc_shoot_target(&chassis); // 计算击球目标位置和发球状态
            gimbal_shoot_process(&chassis);      // 发球状态机处理
            gimbal_send_cmd(&chassis);           // 下发云台控制指令
        }
        if (ctrl_loop % 5 == 0) // 通信循环 200Hz左右
        {
            chasssis_send_pos(&chassis);
        }

        ctrl_loop++;
        if (ctrl_loop > 3000) ctrl_loop = 0;
    }
}

/**
 * @brief 底盘一阶低通滤波初始化
 */
static void chassis_filter_init(robot_t* chassis_ptr)
{
    first_order_filter_init(&chassis_ptr->ctrl.chassis_speed_filter[0], CHASSIS_CONTROL_TIME, filter_speed_alpha[0]);
    first_order_filter_init(&chassis_ptr->ctrl.chassis_speed_filter[1], CHASSIS_CONTROL_TIME, filter_speed_alpha[1]);
    first_order_filter_init(&chassis_ptr->ctrl.chassis_speed_filter[2], CHASSIS_CONTROL_TIME, filter_speed_alpha[2]);
}

/**
 * @brief 初始化PID参数
 */
static void chassis_pid_init(robot_t* chassis_ptr)
{
    for (int i = 0; i < 3; i++)
        PID_init(&chassis_ptr->ctrl.m3508_controller[i].pid, m3508_pid_config);
}

/**
 * @brief 切换底盘控制器（通过遥控器S1拨杆）
 */
static void chassis_switch_controller(robot_t* chassis_ptr)
{
    chassis_ptr->ctrl.robot_controller = chassis_ptr->rc.rc_data.s1;
    chassis_ptr->ctrl.shoot_mode = chassis_ptr->rc.rc_data.s2;
}

static void rc_offline_process(robot_t* chassis_ptr)
{
    static uint8_t offline = 0, reconnect = 0;
    if (DWT_GetTimeline_us() - chassis_ptr->rc.last_online > ms_to_us(500))
    {
        chassis_ptr->ctrl.robot_controller = CHASSIS_RC_OFFLINE;
        chassis_ptr->ctrl.shoot_mode = SHOOT_RECEIVE;
        offline = 1;
    }
    else
        offline = 0;
    if (offline)
    {
        for (uint8_t i = 0; i < 5; i++)
        {
            rs02_disable_private(&chassis_ptr->rs02[i]);
            osDelay(1);
        }
        reconnect = 1;
    }
    if (!offline && reconnect)
    {
        for (uint8_t i = 0; i < 5; i++)
        {
            rs02_enable_private(&chassis_ptr->rs02[i]);
            osDelay(1);
        }
        reconnect = 0;
    }
}

/**
 * @brief 解算给定移动速度（获取控制源指令并滤波）
 */
static void chassis_calc_move_speed(robot_t* chassis_ptr)
{
    static fp32 vx, vy, vw;
    if (chassis_ptr->ctrl.robot_controller == CHASSIS_RC)
    {
        vx = (fp32)chassis_ptr->rc.rc_data.ch3 * CHASSIS_MAX_V / RC_VAL_MAX;
        vy = (fp32)chassis_ptr->rc.rc_data.ch2 * CHASSIS_MAX_V / RC_VAL_MAX;
        vw = (fp32)chassis_ptr->rc.rc_data.ch1 * CHASSIS_MAX_V / RC_VAL_MAX;
    }
    else if (chassis_ptr->ctrl.robot_controller == CHASSIS_UPC)
    {
        ; // 需要计算位置环pid
        // vx = loop_float_constrain(chassis_ptr->nx_ctrl.target_x, -CHASSIS_MAX_V, CHASSIS_MAX_V);
        // vy = loop_float_constrain(chassis_ptr->nx_ctrl.target_y, -CHASSIS_MAX_V, CHASSIS_MAX_V);
        // vw = loop_float_constrain(chassis_ptr->nx_ctrl.target_yaw, -CHASSIS_MAX_V, CHASSIS_MAX_V);
    }
    else
    {
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
 * @param motor_speeds 输出到3个电机的目标转速
 */
void Omni_Kinematics_Resolve(const fp32 vx, const fp32 vy, const fp32 vw, const fp32 spin_yaw, int16_t* motor_speeds)
{
    const fp32 sin_yaw = sinf(radian_format(spin_yaw));
    const fp32 cos_yaw = cosf(radian_format(spin_yaw));
    const fp32 vx_set = vx * cos_yaw - vy * sin_yaw;
    const fp32 vy_set = vx * sin_yaw + vy * cos_yaw;

    motor_speeds[0] = (int16_t)(-vx_set * 0.667f + vw * 0.333f);
    motor_speeds[1] = (int16_t)(vx_set * 0.333f + vy_set * 0.577f + vw * 0.333f);
    motor_speeds[2] = (int16_t)(vx_set * 0.333f - vy_set * 0.577f + vw * 0.333f);
}

/**
 * @brief 解算3508电机的目标速度（给3个轮毂下发指令速度），以上电时刻向前为y正方向
 */
static void chassis_calc_wheelmotor_speed(robot_t* chassis_ptr)
{
    const fp32 chassis_v = chassis_ptr->ctrl.given_chassis_v[0];
    const fp32 theta = chassis_ptr->ctrl.given_chassis_v[1];
    const fp32 chassis_w = chassis_ptr->ctrl.given_chassis_w;
    const fp32 vx = chassis_v * cosf(theta);
    const fp32 vy = chassis_v * sinf(theta);
    int16_t motor_speeds[3];
#ifdef OMNI
    Omni_Kinematics_Resolve(vx, vy, chassis_w, chassis_ptr->ins.ins.Yaw, motor_speeds);
#endif

    chassis_ptr->ctrl.m3508_controller[0].given_speed = motor_speeds[0];
    chassis_ptr->ctrl.m3508_controller[1].given_speed = motor_speeds[1];
    chassis_ptr->ctrl.m3508_controller[2].given_speed = motor_speeds[2];
}

/**
 * @brief PID控制计算
 */
static void chassis_calc_wheelmotor_pidout(robot_t* chassis_ptr)
{
    for (int i = 0; i < 3; i++)
    {
        PID_calc(&chassis_ptr->ctrl.m3508_controller[i].pid, chassis_ptr->m3508.ecd[i].speed, chassis_ptr->ctrl.m3508_controller[i].given_speed);
        chassis_ptr->ctrl.m3508_controller[i].out = chassis_ptr->ctrl.m3508_controller[i].pid.out[0];
    }
}

/**
 * @brief 将控制计算结果下发到CAN总线上
 */
static void chassis_send_wheelmotor_cmd(const robot_t* chassis_ptr)
{
    int16_t m3508_iq[4];
    for (int i = 0; i < 3; i++)
        m3508_iq[i] = (int16_t)chassis_ptr->ctrl.m3508_controller[i].out;
    m3508_ctrl(&chassis_ptr->m3508, m3508_iq);
}

static void gimbal_calc_pitch_target(robot_t* chassis_ptr)
{
    if (chassis_ptr->ctrl.shoot_mode == SHOOT_RECEIVE)
        switch (chassis_ptr->ctrl.robot_controller)
        {
            case CHASSIS_RC:
                chassis_ptr->ctrl.given_pitch += ((fp32)chassis_ptr->rc.rc_data.ch0 * PITCH_DELTA_MAX);
                break;
            case CHASSIS_UPC:
                chassis_ptr->ctrl.given_pitch = chassis_ptr->nx_ctrl.target_pitch;
                break;
            default:
                break;
        }
    else
        chassis_ptr->ctrl.given_yaw = 0;
}

static void gimbal_calc_shoot_target(robot_t* chassis_ptr)
{
    if (chassis_ptr->ctrl.robot_controller == CHASSIS_RC)
        chassis_ptr->ctrl.shoot = chassis_ptr->rc.rc_data.roll > 600 ? 1 : 0;
    else if (chassis_ptr->ctrl.robot_controller == CHASSIS_UPC)
        chassis_ptr->ctrl.shoot = chassis_ptr->nx_ctrl.fire;
    else
        chassis_ptr->ctrl.shoot = 0;
    if (chassis_ptr->ctrl.shoot && chassis_ptr->ctrl.shoot_step == 0)
        chassis_ptr->ctrl.shoot_step = 1;
}

static void gimbal_shoot_process(robot_t* chassis_ptr)
{
    static uint64_t last_time = 0;
    switch (chassis_ptr->ctrl.shoot_step)
    {
        case SHOOT_DEFAULT:
        {
            if (chassis_ptr->ctrl.shoot_mode == SHOOT_SERVE)
            {
                chassis_ptr->ctrl.given_hit_pos = 0.007f;
                chassis_ptr->ctrl.given_hit_speed = 4.0f;
                chassis_ptr->ctrl.given_shoot_angle = 1.5f;
            }
            else
            {
                chassis_ptr->ctrl.given_hit_pos = 0.04f;
                chassis_ptr->ctrl.given_hit_speed = 8.0f;
                chassis_ptr->ctrl.given_shoot_angle = 0.0f;
            }
            break;
        }
        case SHOOT_PREPARE:
        {
            chassis_ptr->ctrl.given_hit_pos = 0.04f;
            chassis_ptr->ctrl.given_hit_speed = 1.0f;
            last_time = DWT_GetTimeline_us();
            chassis_ptr->ctrl.shoot_step = SHOOT_DELAY_1;
            break;
        }
        case SHOOT_DELAY_1:
        {
            if (DWT_GetTimeline_us() - last_time >= ms_to_us(25)) // 25ms准备时间
            {
                chassis_ptr->ctrl.shoot_step = SHOOT_HIT;
            }
            break;
        }
        case SHOOT_HIT:
        {
            if (chassis_ptr->ctrl.shoot_mode == SHOOT_SERVE)
            {
                chassis_ptr->ctrl.given_hit_pos = 0.25f;
                chassis_ptr->ctrl.given_hit_speed = 8.0f;
            }
            else
            {
                chassis_ptr->ctrl.given_hit_pos = 0.57f;
                chassis_ptr->ctrl.given_hit_speed = 24.0f;
                chassis_ptr->ctrl.given_shoot_angle = 0.0f;
            }
            last_time = DWT_GetTimeline_us();
            chassis_ptr->ctrl.shoot_step = SHOOT_DELAY_2;
            break;
        }
        case SHOOT_DELAY_2:
        {
            if (DWT_GetTimeline_us() - last_time >= ms_to_us(175)) // 175ms击球时间
            {
                if (chassis_ptr->ctrl.shoot_mode == SHOOT_SERVE)
                    chassis_ptr->ctrl.shoot_step = SHOOT_LEANING;
                else
                    chassis_ptr->ctrl.shoot_step = SHOOT_DEFAULT;
            }
        }
        case SHOOT_LEANING:
        {
            chassis_ptr->ctrl.given_hit_pos = 0.007f;
            chassis_ptr->ctrl.given_hit_speed = 8.0f;
            chassis_ptr->ctrl.given_pitch = -0.8f;
            last_time = DWT_GetTimeline_us();
            chassis_ptr->ctrl.shoot_step = SHOOT_DELAY_3;
            break;
        }
        case SHOOT_DELAY_3:
        {
            if (DWT_GetTimeline_us() - last_time >= ms_to_us(120))
            {
                chassis_ptr->ctrl.shoot_step = SHOOT_SHOOT;
            }
            break;
        }
        case SHOOT_SHOOT:
        {
            chassis_ptr->ctrl.given_shoot_angle = -2.9f;
            last_time = DWT_GetTimeline_us();
            chassis_ptr->ctrl.shoot_step = SHOOT_DELAY_4;
            break;
        }
        case SHOOT_DELAY_4:
        {
            if (DWT_GetTimeline_us() - last_time >= ms_to_us(250))
            {
                chassis_ptr->ctrl.shoot_step = SHOOT_BACK;
            }
            break;
        }
        case SHOOT_BACK:
        {
            chassis_ptr->ctrl.given_shoot_angle = 1.5f;
            last_time = DWT_GetTimeline_us();
            chassis_ptr->ctrl.shoot_step = SHOOT_DELAY_5;
            break;
        }
        case SHOOT_DELAY_5:
        {
            if (DWT_GetTimeline_us() - last_time >= ms_to_us(1000))
            {
                chassis_ptr->ctrl.shoot_step = SHOOT_DEFAULT;
            }
            break;
        }
        default:
            break;
    }
}

static void gimbal_send_cmd(const robot_t* chassis_ptr)
{
    static uint8_t send_which = 0;
    if (send_which == 0)
    {
        for (uint8_t i = 0; i < 3; i++)
            rs02_ctrl_move_private(&chassis_ptr->rs02[0], chassis_ptr->ctrl.given_hit_pos, chassis_ptr->ctrl.given_hit_speed, MIT_HIT_TORCH, MIT_KP, MIT_KD);
        send_which = 1;
    }
    else if (send_which == 1)
    {
        rs02_ctrl_move_private(&chassis_ptr->rs02[3], chassis_ptr->ctrl.given_pitch, 4.0f, MIT_PITCH_TORCH, MIT_KP, MIT_KD);
        send_which = 2;
    }
    else if (send_which == 2)
    {
        rs02_ctrl_move_private(&chassis_ptr->rs02[4], chassis_ptr->ctrl.given_shoot_angle, 56.0f, MIT_SHOOT_TORCH, MIT_KP, MIT_KD);
        send_which = 0;
    }

}

/**
 * @brief 将底盘位置信息发送到上位机
 * pitch轴数据可能需要改为计算电机编码器获得，是否要改取决于C板是否能安装在云台上
 */
static void chasssis_send_pos(const robot_t* chassis_ptr)
{
    NX_SendPos(&chassis_ptr->nx_ctrl, chassis_ptr->odom.x, chassis_ptr->odom.y, chassis_ptr->ins.ins.Yaw, chassis_ptr->ins.ins.Pitch);
}