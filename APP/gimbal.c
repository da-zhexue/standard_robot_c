#include "gimbal.h"
#include "can.h"
#include "usart.h"

const pid_config yaw_pid_config = {.mode = PID_POSITION, .kp = 50.0f, .ki = 0.0f, .kd = 10.0f, .max_out = 16000.0f, .max_iout = 3000.0f,
    .out_limit_delta_P = 1000.0f, .out_limit_delta_N = 4000.0f, .deadzone = 0.0f};
const pid_config pitch_pid_config = {.mode = PID_POSITION, .kp = 22.0f, .ki = 0.0f, .kd = 200.0f, .max_out = 16000.0f, .max_iout = 3000.0f,
    .out_limit_delta_P = 1000.0f, .out_limit_delta_N = 4000.0f, .deadzone = 0.0f};
const pid_config friction_pid_config = {.mode = PID_POSITION, .kp = 8.0f, .ki = 0.0f, .kd = 0.0f, .max_out = 16000.0f, .max_iout = 3000.0f,
    .out_limit_delta_P = 1000.0f, .out_limit_delta_N = 4000.0f, .deadzone = 0.0f};
const pid_config trigger_pid_config = {.mode = PID_POSITION, .kp = 8.0f, .ki = 0.0f, .kd = 0.0f, .max_out = 16000.0f, .max_iout = 3000.0f,
    .out_limit_delta_P = 1000.0f, .out_limit_delta_N = 4000.0f, .deadzone = 0.0f};

static gimbal_t gimbal;

// ---------- 内部静态函数声明 ----------
static void gimbal_pid_init(gimbal_t* gimbal_ptr);
static uint8_t gimbal_check_game_start(const gimbal_t* gimbal_ptr);
static void gimbal_switch_controller(gimbal_t* gimbal_ptr);
static void gimbal_calc_target_angle(gimbal_t* gimbal_ptr);
static void gimbal_calc_current_angle(gimbal_t* gimbal_ptr);
static void gimbal_fire_confirm(gimbal_t* gimbal_ptr);
static void gimbal_calc_pid_out(gimbal_t* gimbal_ptr);
static void gimbal_send_can_cmd(const gimbal_t* gimbal_ptr);
static void gimbal_send_rc(const gimbal_t* gimbal_ptr);
static void gimbal_send_angle(const gimbal_t* gimbal_ptr);

void gimbal_init(gimbal_t* gimbal_ptr)
{
    memset(gimbal_ptr, 0, sizeof(gimbal_t));

    gm6020_init(&gimbal_ptr->angle_motor, &hcan1, 0x2FF, 2);
    m2006_init(&gimbal_ptr->trigger_motor, &hcan1, 0x200, 1);
    m3508_init(&gimbal_ptr->friction_motor, &hcan1, 0x1FF, 2);

    CBoard_Gimbal_Init(&gimbal_ptr->cboard_gimbal, &hcan2);
    NX_Init(&gimbal_ptr->nx_ctrl, &hcan2);
    dbus_init(&gimbal_ptr->rc, RC_DIRECT, &hcan2);
    HI12_Init(&gimbal_ptr->hi12, &huart1);

    gimbal_pid_init(gimbal_ptr);
}

void Gimbal_Task(const void* argument)
{
    gimbal_init(&gimbal);
    static uint16_t ctrl_loop = 0;
    while(1)
    {
        if (ctrl_loop % 3 == 0) // 控制循环 300Hz左右
        {
            //if (!gimbal_check_game_start(&gimbal)) continue;
            gimbal_switch_controller(&gimbal);
            gimbal_calc_target_angle(&gimbal);
            gimbal_calc_current_angle(&gimbal);
            gimbal_fire_confirm(&gimbal);
            gimbal_calc_pid_out(&gimbal);
            gimbal_send_can_cmd(&gimbal);
        }
        if (ctrl_loop % 5 == 0) // 通信循环 200Hz左右
        {
            gimbal_send_rc(&gimbal);
            gimbal_send_angle(&gimbal);
        }
        ctrl_loop++;
        if (ctrl_loop > 3000) ctrl_loop = 0;
        osDelay(1);

    }
}

static void gimbal_pid_init(gimbal_t* gimbal_ptr)
{
    PID_init(&gimbal_ptr->ctrl.angle_ctrl[0].pid, yaw_pid_config);
    PID_init(&gimbal_ptr->ctrl.angle_ctrl[1].pid, pitch_pid_config);
    PID_init(&gimbal_ptr->ctrl.friction_ctrl[0].pid, friction_pid_config);
    PID_init(&gimbal_ptr->ctrl.friction_ctrl[1].pid, friction_pid_config);
    PID_init(&gimbal_ptr->ctrl.trigger_ctrl.pid, trigger_pid_config);
}

static uint8_t gimbal_check_game_start(const gimbal_t* gimbal_ptr)
{
    return gimbal_ptr->cboard_gimbal.game_start;
}

static void gimbal_switch_controller(gimbal_t* gimbal_ptr)
{
    gimbal_ptr->ctrl.gimbal_controller = gimbal_ptr->rc.rc_data.s1;
}

static void gimbal_calc_target_angle(gimbal_t* gimbal_ptr)
{
    switch (gimbal_ptr->ctrl.gimbal_controller)
    {
        case GIMBAL_RC:
            gimbal_ptr->ctrl.target_yaw += (fp32)gimbal_ptr->rc.rc_data.ch0 * SMALL_GIMBAL_ANGLE_DELTA_MAX / RC_VAL_MAX;
            gimbal_ptr->ctrl.target_pitch += (fp32)gimbal_ptr->rc.rc_data.ch1 * SMALL_GIMBAL_ANGLE_DELTA_MAX / RC_VAL_MAX;
            gimbal_ptr->ctrl.fire = (gimbal_ptr->rc.rc_data.roll >= 600) ? 1 : 0;
            gimbal_ptr->ctrl.open_friction = (gimbal_ptr->rc.rc_data.s2 == 2) ? 1 : 0;
            break;
        case GIMBAL_UPC:
            gimbal_ptr->ctrl.target_yaw = gimbal_ptr->nx_ctrl.target_yaw;
            gimbal_ptr->ctrl.target_pitch = gimbal_ptr->nx_ctrl.target_pitch;
            gimbal_ptr->ctrl.fire = gimbal_ptr->nx_ctrl.fire;
            break;
        case GIMBAL_SHUTDOWN:
        default:
            break;
    }
    if (gimbal_ptr->ctrl.target_yaw > YAW_LIMIT)
        gimbal_ptr->ctrl.target_yaw = YAW_LIMIT;
    else if (gimbal_ptr->ctrl.target_yaw < -YAW_LIMIT)
        gimbal_ptr->ctrl.target_yaw = -YAW_LIMIT;
    if (gimbal_ptr->ctrl.target_pitch > PITCH_LIMIT)
        gimbal_ptr->ctrl.target_pitch = PITCH_LIMIT;
    else if (gimbal_ptr->ctrl.target_pitch < -PITCH_LIMIT)
        gimbal_ptr->ctrl.target_pitch = -PITCH_LIMIT;
}

static void gimbal_calc_current_angle(gimbal_t* gimbal_ptr)
{
    const int yaw_ecd = gimbal_ptr->angle_motor.ecd[0].ecd - ECD_YAW_OFFSET;
    const int pitch_ecd = gimbal_ptr->angle_motor.ecd[1].ecd - ECD_PITCH_OFFSET;

    gimbal_ptr->angle.yaw_deg = theta_format((fp32)yaw_ecd * 360.0f / 8192.0f);
    gimbal_ptr->angle.pitch_deg = theta_format((fp32)pitch_ecd * 360.0f / 8192.0f);
}

static void gimbal_fire_confirm(gimbal_t* gimbal_ptr)
{
    if (gimbal_ptr->ctrl.open_friction
        && gimbal_ptr->friction_motor.ecd[0].speed > FRICTION_READY_SPEED
        && gimbal_ptr->friction_motor.ecd[1].speed < -FRICTION_READY_SPEED )
        // && gimbal_ptr->cbord_gimbal.bullet_allow > 0
        // && gimbal_ptr->cbord_gimbal.shoot_heat < 140)
        gimbal_ptr->ctrl.friction_ready = 1;
    else
        gimbal_ptr->ctrl.friction_ready = 0;
}

static void gimbal_calc_pid_out(gimbal_t* gimbal_ptr)
{

    PID_calc(&gimbal_ptr->ctrl.angle_ctrl[0].pid, gimbal_ptr->angle.yaw_deg, gimbal_ptr->ctrl.target_yaw);
    gimbal_ptr->ctrl.angle_ctrl[0].out = gimbal_ptr->ctrl.angle_ctrl[0].pid.out[0];
    PID_calc(&gimbal_ptr->ctrl.angle_ctrl[1].pid, gimbal_ptr->angle.pitch_deg, gimbal_ptr->ctrl.target_pitch);
    gimbal_ptr->ctrl.angle_ctrl[1].out = gimbal_ptr->ctrl.angle_ctrl[1].pid.out[0];
    PID_calc(&gimbal_ptr->ctrl.friction_ctrl[0].pid, gimbal_ptr->friction_motor.ecd[0].speed, (gimbal_ptr->ctrl.open_friction) ? FRICTION_SPEED_MAX : 0.0f);
    gimbal_ptr->ctrl.friction_ctrl[0].out = gimbal_ptr->ctrl.friction_ctrl[0].pid.out[0];
    PID_calc(&gimbal_ptr->ctrl.friction_ctrl[1].pid, gimbal_ptr->friction_motor.ecd[1].speed, (gimbal_ptr->ctrl.open_friction) ? -FRICTION_SPEED_MAX : 0.0f);
    gimbal_ptr->ctrl.friction_ctrl[1].out = gimbal_ptr->ctrl.friction_ctrl[1].pid.out[0];

    if (gimbal_ptr->ctrl.friction_ready)
    {
        PID_calc(&gimbal_ptr->ctrl.trigger_ctrl.pid, gimbal_ptr->trigger_motor.ecd[0].speed, (gimbal_ptr->ctrl.fire) ? TRIGGER_SPEED_MAX : 0.0f);
        gimbal_ptr->ctrl.trigger_ctrl.out = gimbal_ptr->ctrl.trigger_ctrl.pid.out[0];
    }
}

static void gimbal_send_can_cmd(const gimbal_t* gimbal_ptr)
{
    int16_t m3508_iq[4], gm6020_v[4], m2006_iq[4];
    m3508_iq[0] = (int16_t)gimbal_ptr->ctrl.friction_ctrl[0].out;
    m3508_iq[1] = (int16_t)gimbal_ptr->ctrl.friction_ctrl[1].out;
    m3508_iq[2] = 0; m3508_iq[3] = 0;

    gm6020_v[0] = (int16_t)gimbal_ptr->ctrl.angle_ctrl[0].out;
    gm6020_v[1] = (int16_t)gimbal_ptr->ctrl.angle_ctrl[1].out;
    gm6020_v[2] = 0; gm6020_v[3] = 0;

    m2006_iq[0] = (int16_t)gimbal_ptr->ctrl.trigger_ctrl.out;
    m2006_iq[1] = 0; m2006_iq[2] = 0; m2006_iq[3] = 0;

    m3508_ctrl(&gimbal_ptr->friction_motor, m3508_iq);
    gm6020_ctrl_voltage(&gimbal_ptr->angle_motor, gm6020_v);
    m2006_ctrl(&gimbal_ptr->trigger_motor, m2006_iq);
}

static void gimbal_send_rc(const gimbal_t* gimbal_ptr)
{
    int16_t ch[4];
    uint8_t sw[2];
    ch[0] = gimbal_ptr->rc.rc_data.ch0;
    ch[1] = gimbal_ptr->rc.rc_data.ch1;
    ch[2] = gimbal_ptr->rc.rc_data.ch2;
    ch[3] = gimbal_ptr->rc.rc_data.ch3;
    sw[0] = gimbal_ptr->rc.rc_data.s1;
    sw[1] = gimbal_ptr->rc.rc_data.s2;
    const int16_t roll = gimbal_ptr->rc.rc_data.roll;
    CBoard_RC_Transmit(&gimbal_ptr->cboard_gimbal, ch, sw, roll);
}

static void gimbal_send_angle(const gimbal_t* gimbal_ptr)
{
    fp32 q[4];
    const fp32 euler_angle[3] = {gimbal_ptr->angle.yaw_deg, gimbal_ptr->angle.pitch_deg, 0.0f};
    const fp32 gimbal_yaw = gimbal_ptr->hi12.imu_data.yaw;

    euler_to_quaternion(euler_angle, q);
    NX_SendAngle(&gimbal_ptr->nx_ctrl, q);
    CBoard_IMU_Transmit(&gimbal_ptr->cboard_gimbal, gimbal_yaw);
}