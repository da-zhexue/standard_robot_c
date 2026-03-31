#include "chassis.h"
#include <string.h>

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
void Filter_Init();
void PID_Init();

uint8_t Check_GameStart();
void Switch_Controller();
void Calc_MoveSpeed();
void Calc_RotateAngle();
void Calc_Angle();
void Send_CommInfo();
void Calc_M3508Speed();
void Calc_MF9025Angle();
void Calc_PIDOut();
void Set_MaxPower();
void Update_PowerParam();
void Limit_Power();
void Send_CanCmd();

void chassis_init()
{
    static rc_instance rc_ins;
    static m3508_instance m3508_ins;
    static mf9025_instance mf9025_ins;
    static INS_t ins_ins;
    static comm_t comm_ins;
    static PowerControllerConfig power_ctrl_config;
    static PowerAllocationResult power_ctrl_result;
    static PowerControlParam ctx;
    static super_cap_instance supercap_ins;

    DWT_Init(168);
    dbus_init(&rc_ins);
    m3508_init(&m3508_ins, &hcan1, M3508_TX_1, 4);
    mf9025_init(&mf9025_ins, &hcan1, MF9025_TX_MIN);
    INS_Init(&ins_ins);
    comm_init(&comm_ins, &huart6);
    initPowerControllerConfig(&power_ctrl_config, M3508_TORQUE_CONST, M3508_CURRENT_LIMIT, M3508_OUTPUT_LIMIT,
        K1_CONST,  K2_CONST, K3_CONST, sentinelMaxPower[0]);
    PowerControl_Init(&ctx);
    super_cap_init(&supercap_ins, &hcan1);

    chassis.rc = &rc_ins;
    chassis.m3508 = &m3508_ins;
    chassis.mf9025 = &mf9025_ins;
    chassis.ins = &ins_ins;
    chassis.comm = &comm_ins;
    chassis.power_ctrl_config = &power_ctrl_config;
    chassis.power_ctrl_result = &power_ctrl_result;
    chassis.ctx = &ctx;
    chassis.super_cap = &supercap_ins;

    memset(&chassis.ctrl, 0, sizeof(chassis.ctrl));
    Filter_Init();
    PID_Init();
}

void chassis_task(const void* argument)
{
    chassis_init();
    while(1)
    {
        //if (Check_GameStart()) continue;
        Switch_Controller();
        Calc_MoveSpeed();
        Calc_RotateAngle();
        Calc_Angle();
        Send_CommInfo();
        Calc_M3508Speed();
        Calc_MF9025Angle();
        Calc_PIDOut();
        Set_MaxPower();
        //Update_PowerParam();
        //Limit_Power();
        Send_CanCmd();
        osDelay((uint32_t)(CHASSIS_CONTROL_TIME * 1000));
    }

}

void Filter_Init()
{
    first_order_filter_init(&chassis.ctrl.chassis_speed_filter[0], CHASSIS_CONTROL_TIME, filter_speed_alpha[0]);
    first_order_filter_init(&chassis.ctrl.chassis_speed_filter[1], CHASSIS_CONTROL_TIME, filter_speed_alpha[1]);
    first_order_filter_init(&chassis.ctrl.chassis_speed_filter[2], CHASSIS_CONTROL_TIME, filter_speed_alpha[2]);
}

void PID_Init()
{
    mf9025_set_pid(chassis.mf9025, PARAM_9025_SPEED_PID, mf9025_speed_pid_config[0], mf9025_speed_pid_config[1], mf9025_speed_pid_config[2]);
    PID_init(&chassis.ctrl.m9025_controller.pid, mf9025_angle_pid_config);
    for (int i = 0; i < 4; i++)
        PID_init(&chassis.ctrl.m3508_controller[i].pid, m3508_pid_config);
}

uint8_t Check_GameStart()
{
    chassis.ctrl.start = chassis.comm->game_start;
    return chassis.ctrl.start;
}

void Switch_Controller()
{
    chassis.ctrl.chassis_controller = chassis.rc->rc_data.s1;
}

void Calc_MoveSpeed()
{
    fp32 vx, vy, vw;
    if (chassis.ctrl.chassis_controller == CHASSIS_RC)
    {
        chassis.ctrl.chassis_spin = 0;
        chassis.ctrl.gimbal_scan = 0;

        vx = (fp32)chassis.rc->rc_data.ch3 * CHASSIS_MAX_V / RC_VAL_MAX;
        vy = (fp32)chassis.rc->rc_data.ch2 * CHASSIS_MAX_V / RC_VAL_MAX;
        vw = (fp32)chassis.rc->rc_data.ch1 * CHASSIS_MAX_V / RC_VAL_MAX;
    }
    else if (chassis.ctrl.chassis_controller == CHASSIS_UPC)
    {
        chassis.ctrl.chassis_spin = chassis.comm->comm_ctrl_param.chassis_spin;

        vx = loop_float_constrain(chassis.comm->comm_ctrl_param.vx, -CHASSIS_MAX_V, CHASSIS_MAX_V);
        vy = loop_float_constrain(chassis.comm->comm_ctrl_param.vy, -CHASSIS_MAX_V, CHASSIS_MAX_V);
        vw = loop_float_constrain(chassis.comm->comm_ctrl_param.vw, -CHASSIS_MAX_V, CHASSIS_MAX_V);

        if (chassis.ctrl.chassis_spin)
            vw = 2.0f;
    }
    else
    {
        chassis.ctrl.chassis_spin = 0;
        chassis.ctrl.gimbal_scan = 0;

        vx = 0;
        vy = 0;
        vw = 0;
    }

    // 速度滤波
    first_order_filter_cali(&chassis.ctrl.chassis_speed_filter[0], vx);
    first_order_filter_cali(&chassis.ctrl.chassis_speed_filter[1], vy);
    first_order_filter_cali(&chassis.ctrl.chassis_speed_filter[2], vw);
    const fp32 vx_filter = chassis.ctrl.chassis_speed_filter[0].out;
    const fp32 vy_filter = chassis.ctrl.chassis_speed_filter[1].out;
    const fp32 vw_filter = chassis.ctrl.chassis_speed_filter[2].out;

    fp32 norm_v;
    arm_sqrt_f32(vx_filter * vx_filter + vy_filter * vy_filter, &norm_v);
    chassis.ctrl.given_chassis_v[0] = norm_v * LINEAR_TO_RPM;
    chassis.ctrl.given_chassis_v[1] = atan2f(vy_filter, vx_filter);
    chassis.ctrl.given_chassis_w = vw_filter * LINEAR_TO_RPM;
}

void Calc_RotateAngle()
{
    if (chassis.ctrl.chassis_controller == CHASSIS_RC)
    {
        chassis.ctrl.gimbal_shutdown_flag = 0;
        chassis.ctrl.gimbal_scan = 0;
        const fp32 yaw_delta = (fp32)chassis.rc->rc_data.ch0 * GIMBAL_ANGLE_DELTA_MAX / RC_VAL_MAX;
        chassis.ctrl.given_gimbal_l_yaw += yaw_delta;
    }
    else if (chassis.ctrl.chassis_controller == CHASSIS_UPC)
    {
        chassis.ctrl.gimbal_shutdown_flag = 0;
        chassis.ctrl.gimbal_scan = chassis.comm->comm_ctrl_param.gimbal_scan;
        if (chassis.ctrl.gimbal_scan)
            chassis.comm->comm_ctrl_param.gimbal_yaw += GIMBAL_ANGLE_DELTA_MAX;
        chassis.ctrl.given_gimbal_l_yaw = chassis.comm->comm_ctrl_param.gimbal_yaw;
    }
    else
    {
        chassis.ctrl.gimbal_scan = 0;
        chassis.ctrl.gimbal_shutdown_flag = 1;
    }
}

void Calc_Angle()
{
    static uint8_t gimbal_l_get_offset = 0;
    static fp32 gimbal_l_ptr[3];
    static float Yaw_diff = 0.0f, zero_diff = 0.0f;
    static float chassis_angle_temp[3] = {0.0f, 0.0f, 0.0f};
    static float gimbal_l_offset[3] = {0.0f, 0.0f, 0.0f};
    memcpy(gimbal_l_ptr, chassis.comm->big_gimbal_angle, sizeof(chassis.comm->big_gimbal_angle));
    if (!gimbal_l_get_offset){
        gimbal_l_get_offset = 1;
        memcpy(gimbal_l_offset, gimbal_l_ptr, sizeof(gimbal_l_ptr));
    }
    else
        for (int i = 0; i < 3; i++) gimbal_l_ptr[i] -= gimbal_l_offset[i];
    Angle_Update(&chassis.gimbal_angle, gimbal_l_ptr);
    Yaw_diff = theta_format((fp32)(chassis.mf9025->ecd.ecd - chassis.mf9025->ecd.ecd_offset)/ MF9025_ECD_MAX * 360.0f);
    zero_diff = theta_format((fp32)chassis.mf9025->ecd.zero_offset/ MF9025_ECD_MAX * 360.0f);
    chassis_angle_temp[0] = theta_format(Yaw_diff + gimbal_l_ptr[0] + zero_diff);

    Angle_Update(&chassis.gimbal_angle, gimbal_l_ptr);
    Angle_Update(&chassis.chassis_angle, chassis_angle_temp);
}

void Calc_M3508Speed()
{
    const fp32 chassis_v = chassis.ctrl.given_chassis_v[0];
    const fp32 theta = chassis.ctrl.given_chassis_v[1];
    const fp32 chassis_w = chassis.ctrl.given_chassis_w + chassis.gimbal_angle.yaw_rad;

    const fp32 sin_yaw = sinf(radian_format(chassis.chassis_angle.yaw_rad));// - chassis.gimbal_angle.yaw_rad));
    const fp32 cos_yaw = cosf(radian_format(chassis.chassis_angle.yaw_rad));// - chassis.gimbal_angle.yaw_rad));
    const fp32 vx = chassis_v * cosf(theta);
    const fp32 vy = chassis_v * sinf(theta);
    const fp32 vx_set = vx * cos_yaw - vy * sin_yaw;
    const fp32 vy_set = vx * sin_yaw + vy * cos_yaw;

    chassis.ctrl.m3508_controller[0].given_speed = (int16_t)((vx_set + vy_set) / ROOT_2 + chassis_w);
    chassis.ctrl.m3508_controller[1].given_speed = (int16_t)((-vx_set + vy_set) / ROOT_2 + chassis_w);
    chassis.ctrl.m3508_controller[2].given_speed = (int16_t)((-vx_set - vy_set) / ROOT_2 + chassis_w);
    chassis.ctrl.m3508_controller[3].given_speed = (int16_t)((vx_set - vy_set) / ROOT_2 + chassis_w);
}

void Calc_MF9025Angle()
{
    INS_Update(chassis.ins);
    chassis.ctrl.m9025_controller.given_angle = chassis.ctrl.given_gimbal_l_yaw;
    chassis.ctrl.m9025_controller.ff_speed = 5729.5779513f * chassis.ins->imu.Gyro[2];
}

void Calc_PIDOut()
{
    for (int i = 0; i < 4; i++)
    {
        PID_calc(&chassis.ctrl.m3508_controller[i].pid, chassis.m3508->ecd->speed, chassis.ctrl.m3508_controller[i].given_speed);
        chassis.ctrl.m3508_controller[i].out = chassis.ctrl.m3508_controller[i].pid.out[0];
    }
    PID_calc(&chassis.ctrl.m9025_controller.pid, -chassis.gimbal_angle.yaw_total_angle, chassis.ctrl.m9025_controller.given_angle);
}

void Set_MaxPower()
{
    chassis.ctrl.nav_state = chassis.comm->comm_ctrl_param.nav_state;
    switch (chassis.ctrl.nav_state) // 该策略仅作联盟赛哨兵使用
    {
        case NAV_NORMAL: // 常规情况，不使用超电
            chassis.ctrl.maxpower = sentinelMaxPower[0];
            super_cap_use(chassis.super_cap, 0);
            break;
        case NAV_️RUSH: // ♿️冲刺♿️，使用超电抢占中心点
            chassis.ctrl.maxpower = sentinelMaxPower[0] + chassis.super_cap->power_data.supercap_power;
            super_cap_use(chassis.super_cap, 1);
            break;
        default:
            chassis.ctrl.maxpower = sentinelMaxPower[0];
            super_cap_use(chassis.super_cap, 0);
            break;
    }
    if (chassis.super_cap->power_data.remain_v < REMAINPOWER_MIN) // 超电剩余能量过低时不使用超电
    {
        chassis.ctrl.maxpower = sentinelMaxPower[0];
        super_cap_use(chassis.super_cap, 0);
    }
    setMaxPower(chassis.power_ctrl_config, chassis.ctrl.maxpower);
    limitMaxPower(chassis.power_ctrl_config, chassis.comm->buffer);
}

void Update_PowerParam()
{
    const float measuredpower = chassis.super_cap->power_data.total_power;

    float torqueFeedback[4];
    float rpmFeedback[4];

    for (int i = 0; i < 4; i++) {
        torqueFeedback[i] = (float)chassis.m3508[i].ecd->current * M3508_CURRENT_LIMIT / M3508_ECD_MAX * 0.3f; // m3508的力矩与电流比例大致等于0.3
        rpmFeedback[i] = chassis.m3508[i].ecd->speed;
    }

    float effectivePower = 0.0f;
    for (int i = 0; i < 4; i++) {
        const float angularVelocity = rpmFeedback[i] * (3.1415926535f / 30.0f);
        effectivePower += torqueFeedback[i] * angularVelocity;
    }

    PowerControl_CollectMotorData(chassis.ctx, torqueFeedback, rpmFeedback, measuredpower, 4);
    PowerControl_Update(chassis.ctx, effectivePower);

    updatePowerControlConfig(chassis.power_ctrl_config, chassis.ctx->k2, chassis.ctx->k3);
}

void Limit_Power()
{
    static MotorPowerObj motorpower[4];

    for (int i = 0; i < 4; i++)
    {
        motorpower[i].curAv = (fp32)chassis.m3508->ecd->speed * ECD_TO_AV;
        motorpower[i].setAv = (fp32)chassis.ctrl.m3508_controller[i].given_speed * ECD_TO_AV;
        motorpower[i].pidOutput = chassis.ctrl.m3508_controller[i].pid.out[0];
        motorpower[i].pidMaxOutput = chassis.ctrl.m3508_controller[i].pid.max_out;
    }

    MotorPowerObj *motors[4] = {&motorpower[0], &motorpower[1], &motorpower[2], &motorpower[3]};
    allocatePowerWithLimit(motors, chassis.power_ctrl_config, chassis.power_ctrl_result);

    for (int i = 0; i < 4; i++)
    {
        chassis.ctrl.m3508_controller[i].out = (int16_t)chassis.power_ctrl_result->newTorqueCurrent[i];
    }
}

void Send_CanCmd()
{
    int16_t m3508_iq[4];
    for (int i = 0; i < 4; i++)
        m3508_iq[i] = (int16_t)chassis.ctrl.m3508_controller[i].out;
    const int32_t mf9025_speed = (int32_t)(chassis.ctrl.m9025_controller.pid.out[0] + chassis.ctrl.m9025_controller.ff_speed);
    m3508_ctrl(chassis.m3508, m3508_iq);
    mf9025_ctrl_speed(chassis.mf9025, MF9025_MAX_IQ, mf9025_speed);
}

void Send_CommInfo()
{
    comm_send_attitude(chassis.chassis_angle.yaw_deg);
}