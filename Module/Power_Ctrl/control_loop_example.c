/*

底盘控制主循环中的功率控制相关函数实现

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
        Update_PowerParam();
        Limit_Power();
        Send_CanCmd();
        osDelay((uint32_t)(CHASSIS_CONTROL_TIME * 1000));
    }

}

// 根据当前导航状态和超电给出的功率设置最大功率限制
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

void Update_PowerParam() // 最小二乘法更新功率控制器的参数，供功率分配函数使用，以在Limit_Power中可以更准确地估计底盘功率
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

void Limit_Power() // 根据当前pid输出目标电流控制值和当前速度值预估功率需求，如果超过最大功率限制则重新分配四个电机电流控制值以满足功率限制
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

*/