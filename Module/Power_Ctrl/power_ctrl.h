#ifndef NEW_CHASSIS_POWER_CTRL_H
#define NEW_CHASSIS_POWER_CTRL_H
#include "typedef.h"

// 常量定义
#define ERROR_POWER_DISTRIBUTION_SET 20.0f
#define PROP_POWER_DISTRIBUTION_SET 15.0f
#define MIN_OUTPUT 1e-5f
#define POWER_DISTRIBUTION_COEFF 0.85f

#define RPM_TO_RADS 0.104719755f // PI/30
#define ECD_TO_AV 0.0054537522f // M3508编码器每增量对应的角速度 (rad/s)

#define M3508_TORQUE_CONST 0.3f // M3508转矩常数 N·m/A
#define M3508_CURRENT_LIMIT 20.0f // M3508最大电流 C620说明书给出最大输出20A
#define M3508_OUTPUT_LIMIT 16384.0f // 归一化输出上限 -16384~16384 -> -20A~20A

#define K1_CONST 0.22f
#define K2_CONST 1.2f
#define K3_CONST 7.0f

#define POWERBUFFER_MAX 60.0f
#define REMAINPOWER_MIN 14.0f
extern fp32 sentinelMaxPower[1];

// 电机数据结构
typedef struct {
    float curAv;        // 当前角速度 (rad/s)
    float setAv;        // 目标角速度 (rad/s)
    float pidOutput;    // PID原始输出
    float pidMaxOutput; // PID输出限制
    float Current;      // 当前电流 (-16384~16384)
} MotorPowerObj;

// 功率控制器配置
typedef struct {
    float k0;           // 转矩常数: 扭矩/输出比 (Nm/output)
    float k1;           // 功率模型参数1: 与速度相关的损耗系数
    float k2;           // 功率模型参数2: 与转矩平方相关的损耗系数
    float k3;           // 功率模型参数3: 固定损耗
    float maxPower;     // 最大允许功率 (W)
} PowerControllerConfig;

// 功率分配结果
typedef struct {
    float newTorqueCurrent[4];  // 重新分配后的电机输出
    float sumPowerRequired;     // 总需求功率
    float sumPowerAfterAlloc;   // 重新分配后总功率
    float efficiency;          // 分配效率
} PowerAllocationResult;

void allocatePowerWithLimit(MotorPowerObj *objs[4], const PowerControllerConfig *config, PowerAllocationResult *result);
void initPowerControllerConfig(PowerControllerConfig *config, float torqueConst, float currentLimit, float outputLimit,
                                float k1_init, float k2_init, float k3_init, float maxPower);
void setMaxPower(PowerControllerConfig* config, float maxPower);
uint8_t limitMaxPower(PowerControllerConfig * config, float buffer);
void updatePowerControlConfig(PowerControllerConfig* config, float k2, float k3);

#endif //NEW_CHASSIS_POWER_CTRL_H