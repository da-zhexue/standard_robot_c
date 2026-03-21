#include "power_ctrl.h"

/**
 * 功率控制器 - 目标功率输出和电机速度分配
 * 适用于机器人底盘功率限制场景
 */

#include <math.h>

// 不同车组功率上限
fp32 sentinelMaxPower[1] = {100.0f};

// 辅助函数声明
static float clamp(float value, float min, float max);
static float calculateMotorPower(float torque, float av, float k1, float k2, float k3, int motorCount);
static float calculateAllocatedPower(float k0, float k1, float k2, float k3,
                                     float curAv, float pidOutput,
                                     float allocatedPower);

/**
 * @brief 计算功率限制下的电机输出分配
 *
 * @param objs 电机对象数组指针
 * @param config 功率控制器配置
 * @param result 分配结果输出
 *
 * @note 该函数根据总功率限制重新分配四个电机的输出，
 *       优先保证跟随误差大的电机性能，同时满足功率约束
 */
void allocatePowerWithLimit(MotorPowerObj *objs[4], const PowerControllerConfig *config, PowerAllocationResult *result) {
    float cmdPower[4] = {0};      // 各电机原始功率需求
    float error[4] = {0};         // 各电机速度误差
    float sumCmdPower = 0.0f;     // 总功率需求
    float sumError = 0.0f;        // 总速度误差
    float allocatablePower = config->maxPower;  // 可分配功率
    float sumPowerRequired = 0.0f; // 功率需求为正的电机总需求

    // 计算每个电机的功率需求和误差
    for (int i = 0; i < 4; i++) {
        MotorPowerObj *p = objs[i];

        // 计算电机功率需求: P = τ·ω + k₁·|ω| + k₂·τ² + k₃/4
        // 其中 τ = k₀ * pidOutput
        const float torque = config->k0 * p->pidOutput;
        // cmdPower[i] = torque * p->curAv + fabsf(p->curAv) * config->k1 +
        //               torque * torque * config->k2 + config->k3 / 4.0f;
        cmdPower[i] = calculateMotorPower(torque, p->curAv, config->k1, config->k2, config->k3, 4);
        sumCmdPower += cmdPower[i];

        // 计算速度跟随误差
        error[i] = fabsf(p->setAv - p->curAv);

        // 如果功率需求为负（再生制动），释放其功率配额
        if (cmdPower[i] <= MIN_OUTPUT) {
            // 负功率意味着能量回馈，不占用功率配额
            allocatablePower += -cmdPower[i];
        } else {
            // 正功率需求，计入总需求和总误差
            sumError += error[i];
            sumPowerRequired += cmdPower[i];
        }
    }

    // 保存原始总功率需求
    result->sumPowerRequired = sumCmdPower;

    // 检查是否需要功率限制
    if (sumCmdPower <= config->maxPower) {
        // 功率充足，直接使用原始PID输出
        for (int i = 0; i < 4; i++) {
            result->newTorqueCurrent[i] = objs[i]->pidOutput;
        }
        result->sumPowerAfterAlloc = sumCmdPower;
        result->efficiency = 1.0f;  // 无限制，效率100%
        return;
    }

    // 需要功率限制，计算重新分配
    float errorConfidence = 0.0f;

    // 计算误差置信度，决定分配策略
    if (sumError > ERROR_POWER_DISTRIBUTION_SET) {
        // 误差较大，完全按误差比例分配
        errorConfidence = 1.0f;
    } else if (sumError > PROP_POWER_DISTRIBUTION_SET) {
        // 误差中等，混合分配
        errorConfidence = clamp(
            (sumError - PROP_POWER_DISTRIBUTION_SET) /
            (ERROR_POWER_DISTRIBUTION_SET - PROP_POWER_DISTRIBUTION_SET),
            0.0f, 1.0f);
    } else {
        // 误差较小，完全按功率比例分配
        errorConfidence = 0.0f;
    }

    // 第四步：重新分配功率，计算新输出
    float totalAllocatedPower = 0.0f;

    for (int i = 0; i < 4; i++) {
        MotorPowerObj *p = objs[i];

        if (cmdPower[i] <= MIN_OUTPUT) {
            // 功率需求为负，保持原输出
            result->newTorqueCurrent[i] = p->pidOutput;
        } else {
            // 计算分配权重
            const float powerWeight_Error = error[i] / sumError;
            const float powerWeight_Prop = cmdPower[i] / sumPowerRequired;
            const float powerWeight = errorConfidence * powerWeight_Error +
                         (1.0f - errorConfidence) * powerWeight_Prop;

            // 计算该电机分配到的功率
            const float allocatedPower = powerWeight * allocatablePower;

            // 解二次方程计算新输出
            result->newTorqueCurrent[i] = calculateAllocatedPower(
                config->k0, config->k1, config->k2, config->k3,
                p->curAv, p->pidOutput, allocatedPower
            );
        }

        // 钳位输出到允许范围
        result->newTorqueCurrent[i] = clamp(
            result->newTorqueCurrent[i],
            -p->pidMaxOutput,
            p->pidMaxOutput
        );

        // 计算重新分配后的总功率
        const float newTorque = config->k0 * result->newTorqueCurrent[i];
        totalAllocatedPower += newTorque * p->curAv +
                              fabsf(p->curAv) * config->k1 +
                              newTorque * newTorque * config->k2 +
                              config->k3 / 4.0f;
    }

    result->sumPowerAfterAlloc = totalAllocatedPower;

    // 计算分配效率
    if (sumCmdPower > 0) {
        result->efficiency = clamp(totalAllocatedPower / sumCmdPower, 0.0f, 1.0f);
    } else {
        result->efficiency = 0.0f;
    }

}

/**
 * @brief 计算分配给电机的功率对应的新输出
 *
 * 解二次方程: P_allocation = k₀·τ_new·ω + k₁·|ω| + k₂·(k₀·τ_new)² + k₃/4
 * 整理为: k₂·k₀²·τ_new² + k₀·ω·τ_new + (k₁·|ω| + k₃/4 - P_allocation) = 0
 */
static float calculateAllocatedPower(const float k0, const float k1, const float k2, const float k3,
                                     const float curAv, const float pidOutput, const float allocatedPower) {
    // 二次方程系数
    const float a = k2 * k0 * k0;
    const float b = k0 * curAv;
    const float c = k1 * fabsf(curAv) + k3 / 4.0f - allocatedPower;

    // 计算判别式
    const float discriminant = b * b - 4.0f * a * c;

    float newOutput = 0.0f;

    if (discriminant > 1e-5f) {
        // 两个实根，选择与原始输出同号的解
        const float sqrtDelta = sqrtf(discriminant);
        if (pidOutput > 0) {
            newOutput = (-b + sqrtDelta) / (2.0f * a);
        } else {
            newOutput = (-b - sqrtDelta) / (2.0f * a);
        }
    } else {
        // 无实根，取对称轴位置
        newOutput = -b / (2.0f * a);
    }

    return newOutput;
}

/**
 * @brief 计算电机功率需求
 */
static float calculateMotorPower(const float torque, const float av, const float k1, const float k2, const float k3, int motorCount) {
    return torque * av + fabsf(av) * k1 + torque * torque * k2 + k3 / (float)motorCount;
}

/**
 * @brief 钳位函数
 */
static float clamp(const float value, const float min, const float max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

/**
 * @brief 初始化功率控制器配置
 */
void initPowerControllerConfig(PowerControllerConfig *config,
                               const float torqueConst, const float currentLimit, const float outputLimit,
                               const float k1_init, const float k2_init, const float k3_init, const float maxPower) {
    // 计算k0: 扭矩常数 * 电流限制 / 输出限制
    config->k0 = torqueConst * currentLimit / outputLimit;
    config->k1 = k1_init;
    config->k2 = k2_init;
    config->k3 = k3_init;
    config->maxPower = maxPower;
}

/*
 * @brief 设置最大功率
 */
void setMaxPower(PowerControllerConfig * config, const float maxPower) // 裁判系统功率限制+超电
{
    config->maxPower = maxPower;
}

/*
 * @brief 输入当前从裁判系统获得的能量缓冲值，当缓冲值下降后或裁判系统断联后对最大功率进一步限制
 */
uint8_t limitMaxPower(PowerControllerConfig * config, const float buffer)
{
    const float raw_maxPower = config->maxPower;
    if (buffer - POWERBUFFER_MAX > 1e-5)
    {
        config->maxPower = raw_maxPower * 0.80f;
        return 1;
    }
    return 0;
}

void updatePowerControlConfig(PowerControllerConfig* config, const float k2, const float k3)
{
    config->k2 = k2;
    config->k3 = k3;
}

/**
 * @brief 示例使用代码
 */
// void exampleUsage(void) {
//      for(int i = 0; i < 4; i++){
//         PID_calc(&m3508_ctrl[i].pid, m3508_ptr[i].speed, m3508_ctrl[i].given_speed);
//     }
//     for (int i = 0; i < 4; i++)
//     {
//         motorpower[i].curAv = m3508_ptr[i].speed / 9.55f / 19.2f; // 9.55f是将rpm转为rad/s  19.2f是减速比
//         motorpower[i].setAv = m3508_ctrl[i].given_speed / 9.55f / 19.2f;
//         motorpower[i].pidOutput = m3508_ctrl[i].pid.out[0];
//         motorpower[i].pidMaxOutput = m3508_ctrl[i].pid.max_out;
//     }
//     MotorPowerObj *motors[4] = {&motorpower[0], &motorpower[1], &motorpower[2], &motorpower[3]};

//     allocatePowerWithLimit(motors, &power_ctrl_config, &result);
//     CAN_Control3508Current((int16_t)result.newTorqueCurrent[0], (int16_t)result.newTorqueCurrent[1] , 
// 			(int16_t)result.newTorqueCurrent[2], (int16_t)result.newTorqueCurrent[3]);
// }
