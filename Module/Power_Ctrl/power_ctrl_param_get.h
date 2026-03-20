#ifndef NEW_CHASSIS_POWER_CTRL_PARAM_GET_H
#define NEW_CHASSIS_POWER_CTRL_PARAM_GET_H
#include "rls.h"
#include "typedef.h"

typedef struct {
    RLS_Handle rls;           // RLS估计器

    // 功率模型参数
    float k1;      // 线性速度损耗系数
    float k2;      // 二次扭矩损耗系数
    float k3;      // 固定损耗系数

    // 系统状态
    float estimatedPower;    // 估算功率
    float measuredPower;     // 实测功率

    // 采样数据
    Vector2 samples;         // 采样向量 [Σ|转速|, Σ扭矩²]

} PowerControlParam;

void PowerControl_Init(PowerControlParam* ctx);
void PowerControl_Update(PowerControlParam* ctx, float effectivePower);
void PowerControl_CollectMotorData(PowerControlParam* ctx, const float* torqueFeedback,
    const float* rpmFeedback, float measuredPower, int motorCount);
float* PowerControl_GetParam(const PowerControlParam* ctx);
void PowerControl_ResetRLS(PowerControlParam* ctx);

#endif //NEW_CHASSIS_POWER_CTRL_PARAM_GET_H