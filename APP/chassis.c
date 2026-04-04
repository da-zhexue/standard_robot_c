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
const pid_config mf9025_angle_pid_config = {.mode = PID_POSITION, .kp = 600.0f, .ki = 0.0f, .kd = 45000.0f, .max_out = 30000.0f, .max_iout = 3000.0f,
    .out_limit_delta_P = 10000.0f, .out_limit_delta_N = 40000.0f, .deadzone = 0.0f};
const uint16_t mf9025_speed_pid_config[3] = {80, 10, 10};
static chassis_t chassis;

// ---------- 内部静态函数声明 ----------
static void chassis_filter_init(chassis_t* chassis_ptr);
static void chassis_pid_init(chassis_t* chassis_ptr);
//static uint8_t chassis_check_game_start(chassis_t* chassis_ptr);
static void chassis_switch_controller(chassis_t* chassis_ptr);
static void chassis_calc_move_speed(chassis_t* chassis_ptr);
static void chassis_calc_rotate_angle(chassis_t* chassis_ptr);
static void chassis_calc_angle(chassis_t* chassis_ptr);
static void chassis_calc_m3508_speed(chassis_t* chassis_ptr);
static void chassis_calc_mf9025_angle(chassis_t* chassis_ptr);
static void chassis_calc_pid_out(chassis_t* chassis_ptr);
static void chassis_set_max_power(chassis_t* chassis_ptr);
static void chassis_update_power_param(chassis_t* chassis_ptr);
static void chassis_limit_power(chassis_t* chassis_ptr);
static void chassis_send_can_cmd(const chassis_t* chassis_ptr);
static void chassis_send_referee_info(const chassis_t* chassis_ptr);

/**
 * @brief 底盘初始化函数
 *        初始化各类传感器、电机控制、PID等
 */
void Chassis_Init(chassis_t* chassis_ptr)
{
    DWT_Init(DWT_CLOCK_FREQ);
    dbus_init(&chassis_ptr->rc, RC_CHASSIS, &hcan1);
    m3508_init(&chassis_ptr->m3508, &hcan1, M3508_TX_1, 4);
    mf9025_init(&chassis_ptr->mf9025, &hcan1, MF9025_TX_MIN);
    INS_Init(&chassis_ptr->ins);
    initPowerControllerConfig(&chassis_ptr->power_ctrl_config, M3508_TORQUE_CONST, M3508_CURRENT_LIMIT, M3508_OUTPUT_LIMIT,
        K1_CONST,  K2_CONST, K3_CONST, sentinelMaxPower[0]);
    PowerControl_Init(&chassis_ptr->ctx);
    super_cap_init(&chassis_ptr->super_cap, &hcan1);
    NUC_Init(&chassis_ptr->nuc_ctrl, &huart6);
    Referee_Init(&chassis_ptr->referee, &huart1);

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
    uint16_t ctrl_loop = 0;
    while(1)
    {
        if (ctrl_loop % 3 == 0) // 控制循环 300Hz左右
        {
            //if (chassis_check_game_start(&chassis)) continue;
            chassis_switch_controller(&chassis);      // 切换控制源（遥控/上位机等）

            // 运动学计算与航向更新
            chassis_calc_move_speed(&chassis);        // 计算并过滤移动速度
            chassis_calc_rotate_angle(&chassis);      // 计算云台旋转角度
            chassis_calc_angle(&chassis);             // 计算绝对角度信息

            // 控制闭环计算
            chassis_calc_m3508_speed(&chassis);       // 解算底盘轮毂电机速度
            chassis_calc_mf9025_angle(&chassis);      // 解算航向云台角度环

            chassis_calc_pid_out(&chassis);           // 计算电机控制PID输出

            // 功率控制策略
            chassis_set_max_power(&chassis);          // 设置最大允许功率限制（包括超电策略）
            //chassis_update_power_param(&chassis);   // 功率控制参数更新 (当前未使用)
            //chassis_limit_power(&chassis);          // 功率分配控制算法 (当前未使用)

            // 下发控制指令
            chassis_send_can_cmd(&chassis);
        }
        if (ctrl_loop % 100 == 0) // 通信循环 10Hz左右
        {
            chassis_send_referee_info(&chassis); // 发送裁判系统信息到云台
        }
        ctrl_loop++;
        if (ctrl_loop > 3000) ctrl_loop = 0;
        osDelay(1);
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
    mf9025_set_pid(&chassis_ptr->mf9025, PARAM_9025_SPEED_PID, mf9025_speed_pid_config[0], mf9025_speed_pid_config[1], mf9025_speed_pid_config[2]);
    PID_init(&chassis_ptr->ctrl.m9025_controller.pid, mf9025_angle_pid_config);
    for (int i = 0; i < 4; i++)
        PID_init(&chassis_ptr->ctrl.m3508_controller[i].pid, m3508_pid_config);
}

/**
 * @brief 检查游戏是否开始 底盘由NUC控制时由NUC判断是否开始
 */
// static uint8_t chassis_check_game_start(chassis_t* chassis_ptr)
// {
//     chassis_ptr->ctrl.start = chassis_ptr->comm.game_start;
//     return chassis_ptr->ctrl.start;
// }

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
 * @brief 解算云台旋转的给定角度
 */
static void chassis_calc_rotate_angle(chassis_t* chassis_ptr)
{
    if (chassis_ptr->ctrl.chassis_controller == CHASSIS_RC)
    {
        chassis_ptr->ctrl.gimbal_shutdown_flag = 0;
        chassis_ptr->ctrl.gimbal_scan = 0;
        const fp32 yaw_delta = (fp32)chassis_ptr->rc.rc_data.ch0 * GIMBAL_ANGLE_DELTA_MAX / RC_VAL_MAX;
        chassis_ptr->ctrl.given_gimbal_l_yaw += yaw_delta;
    }
    else if (chassis_ptr->ctrl.chassis_controller == CHASSIS_UPC)
    {
        chassis_ptr->ctrl.gimbal_shutdown_flag = 0;
        chassis_ptr->ctrl.gimbal_scan = chassis_ptr->nuc_ctrl.gimbal_scan;
        if (chassis_ptr->ctrl.gimbal_scan)
            chassis_ptr->nuc_ctrl.gimbal_yaw += GIMBAL_ANGLE_DELTA_MAX;
        chassis_ptr->ctrl.given_gimbal_l_yaw = chassis_ptr->nuc_ctrl.gimbal_yaw;
    }
    else
    {
        chassis_ptr->ctrl.gimbal_scan = 0;
        chassis_ptr->ctrl.gimbal_shutdown_flag = 1;
    }
}

/**
 * @brief 计算目前云台和底盘系统的绝对角度
 *        融合MF9025编码器数据和大云台传输过来的位姿计算本底底盘系统的世界系位姿
 */
static void chassis_calc_angle(chassis_t* chassis_ptr)
{
    static uint8_t gimbal_l_get_offset = 0;
    static fp32 gimbal_l_ptr[3];
    static float yaw_diff = 0.0f, zero_diff = 0.0f;
    static float chassis_angle_temp[3] = {0.0f, 0.0f, 0.0f};
    static float gimbal_l_offset[3] = {0.0f, 0.0f, 0.0f};

    memcpy(gimbal_l_ptr, chassis_ptr->cbord_chassis.imu_angle, sizeof(chassis_ptr->cbord_chassis.imu_angle));
    if (!gimbal_l_get_offset){
        gimbal_l_get_offset = 1;
        memcpy(gimbal_l_offset, gimbal_l_ptr, sizeof(gimbal_l_ptr));
    }
    else
        for (int i = 0; i < 3; i++) gimbal_l_ptr[i] -= gimbal_l_offset[i];
    Angle_Update(&chassis_ptr->gimbal_angle, gimbal_l_ptr);
    yaw_diff = theta_format((fp32)(chassis_ptr->mf9025.ecd.ecd - chassis_ptr->mf9025.ecd.ecd_offset)/ MF9025_ECD_MAX * DEG_PER_CIRCLE);
    zero_diff = theta_format((fp32)chassis_ptr->mf9025.ecd.zero_offset/ MF9025_ECD_MAX * DEG_PER_CIRCLE);
    chassis_angle_temp[0] = theta_format(yaw_diff + gimbal_l_ptr[0] + zero_diff);

    Angle_Update(&chassis_ptr->gimbal_angle, gimbal_l_ptr);
    Angle_Update(&chassis_ptr->chassis_angle, chassis_angle_temp);
}

/**
 * @brief 麦克纳姆轮底盘运动学解算函数
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
static void chassis_calc_m3508_speed(chassis_t* chassis_ptr)
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
 * @brief 解算MF9025航向电机角度及前馈量
 */
static void chassis_calc_mf9025_angle(chassis_t* chassis_ptr)
{
    INS_Update(&chassis_ptr->ins);
    chassis_ptr->ctrl.m9025_controller.given_angle = chassis_ptr->ctrl.given_gimbal_l_yaw;
    chassis_ptr->ctrl.m9025_controller.ff_speed = RAD_TO_DEG_FACTOR * chassis_ptr->ins.imu.Gyro[2];
}

/**
 * @brief PID控制律计算，包括3508速度环的PID计算和MF9025航向角度环的PID计算
 */
static void chassis_calc_pid_out(chassis_t* chassis_ptr)
{
    for (int i = 0; i < 4; i++)
    {
        PID_calc(&chassis_ptr->ctrl.m3508_controller[i].pid, chassis_ptr->m3508.ecd[i].speed, chassis_ptr->ctrl.m3508_controller[i].given_speed);
        chassis_ptr->ctrl.m3508_controller[i].out = chassis_ptr->ctrl.m3508_controller[i].pid.out[0];
    }
    PID_calc(&chassis_ptr->ctrl.m9025_controller.pid, -chassis_ptr->gimbal_angle.yaw_total_angle, chassis_ptr->ctrl.m9025_controller.given_angle);
}

/**
 * @brief 最大功率限制参数设置与超级电容控制模式控制策略
 */
static void chassis_set_max_power(chassis_t* chassis_ptr)
{
    chassis_ptr->ctrl.use_cap = chassis_ptr->nuc_ctrl.use_cap;
    switch (chassis_ptr->ctrl.use_cap) // 该策略仅作联盟赛哨兵使用
    {
        case 0: // 常规情况，不使用超电
            chassis_ptr->ctrl.maxpower = sentinelMaxPower[0];
            super_cap_use(&chassis_ptr->super_cap, 0);
            break;
        case 1: // ♿️冲刺♿️，使用超电抢占中心点
            chassis_ptr->ctrl.maxpower = sentinelMaxPower[0] + chassis_ptr->super_cap.power_data.supercap_power;
            super_cap_use(&chassis_ptr->super_cap, 1);
            break;
        default:
            chassis_ptr->ctrl.maxpower = sentinelMaxPower[0];
            super_cap_use(&chassis_ptr->super_cap, 0);
            break;
    }
    if (chassis_ptr->super_cap.power_data.remain_v < REMAINPOWER_MIN) // 超电剩余能量过低时不使用超电
    {
        chassis_ptr->ctrl.maxpower = sentinelMaxPower[0];
        super_cap_use(&chassis_ptr->super_cap, 0);
    }
    setMaxPower(&chassis_ptr->power_ctrl_config, chassis_ptr->ctrl.maxpower);
    limitMaxPower(&chassis_ptr->power_ctrl_config, chassis_ptr->referee.power_heat_data.buffer_energy);
}

/**
 * @brief 将功率预测参数更新
 */
static void chassis_update_power_param(chassis_t* chassis_ptr)
{
    const float measuredpower = chassis_ptr->super_cap.power_data.total_power;

    float torque_feedback[4];
    float rpm_feedback[4];

    for (int i = 0; i < 4; i++) {
        torque_feedback[i] = (float)chassis_ptr->m3508.ecd[i].current * M3508_CURRENT_LIMIT / M3508_ECD_MAX * TORQUE_TO_CURRENT_FACTOR;
        rpm_feedback[i] = chassis_ptr->m3508.ecd[i].speed;
    }

    float effective_power = 0.0f;
    for (int i = 0; i < 4; i++) {
        const float angular_velocity = rpm_feedback[i] * (PI / 30.0f);
        effective_power += torque_feedback[i] * angular_velocity;
    }

    PowerControl_CollectMotorData(&chassis_ptr->ctx, torque_feedback, rpm_feedback, measuredpower, 4);
    PowerControl_Update(&chassis_ptr->ctx, effective_power);

    updatePowerControlConfig(&chassis_ptr->power_ctrl_config, chassis_ptr->ctx.k2, chassis_ptr->ctx.k3);
}

/**
 * @brief 利用功率控制器计算各电机的限制输出
 */
static void chassis_limit_power(chassis_t* chassis_ptr)
{
    static MotorPowerObj motor_power[4];

    for (int i = 0; i < 4; i++)
    {
        motor_power[i].curAv = (fp32)chassis_ptr->m3508.ecd[i].speed * ECD_TO_AV;
        motor_power[i].setAv = (fp32)chassis_ptr->ctrl.m3508_controller[i].given_speed * ECD_TO_AV;
        motor_power[i].pidOutput = chassis_ptr->ctrl.m3508_controller[i].pid.out[0];
        motor_power[i].pidMaxOutput = chassis_ptr->ctrl.m3508_controller[i].pid.max_out;
    }

    MotorPowerObj *motors[4] = {&motor_power[0], &motor_power[1], &motor_power[2], &motor_power[3]};
    allocatePowerWithLimit(motors, &chassis_ptr->power_ctrl_config, &chassis_ptr->power_ctrl_result);

    for (int i = 0; i < 4; i++)
    {
        chassis_ptr->ctrl.m3508_controller[i].out = (int16_t)chassis_ptr->power_ctrl_result.newTorqueCurrent[i];
    }
}

/**
 * @brief 将控制计算结果下发到CAN总线上
 */
static void chassis_send_can_cmd(const chassis_t* chassis_ptr)
{
    int16_t m3508_iq[4];
    for (int i = 0; i < 4; i++)
        m3508_iq[i] = (int16_t)chassis_ptr->ctrl.m3508_controller[i].out;
    const int32_t mf9025_speed_out = (int32_t)(chassis_ptr->ctrl.m9025_controller.pid.out[0] + chassis_ptr->ctrl.m9025_controller.ff_speed);
    m3508_ctrl(&chassis_ptr->m3508, m3508_iq);
    mf9025_ctrl_speed(&chassis_ptr->mf9025, MF9025_MAX_IQ, mf9025_speed_out);
}


static void chassis_send_referee_info(const chassis_t* chassis_ptr)
{
    const uint8_t game_start = chassis_ptr->referee.game_status.game_progress == 4 ? 1 : 0;
    const uint8_t camp = (chassis_ptr->referee.robot_performance.robot_id > 100) ? 1 : 0;
    const uint8_t attitude = (chassis_ptr->referee.sentry_info.sentry_info_2 >> 12) & 0x03;
    const uint16_t shoot_heat = chassis_ptr->referee.power_heat_data.shooter_17mm_1_barrel_heat;
    const uint16_t bullet_allow = chassis_ptr->referee.projectile_allowance.projectile_allowance_17mm;
    CBoard_Referee_Tranmit(&chassis_ptr->cbord_chassis, game_start, camp, attitude, shoot_heat, bullet_allow);
}