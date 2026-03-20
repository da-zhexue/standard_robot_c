/*
 * @file: power_ctrl_param_get.c
 * @brief: 通过RLS算法预估功率控制所需参数，需要从外部获得准确功率
 */
#include "power_ctrl_param_get.h"
#include "math.h"
/**
 * @brief 初始化功率控制
 */
void PowerControl_Init(PowerControlParam* ctx) {
    if(ctx == NULL)
        return;

    const float initParams[2] = {ctx->k1, ctx->k2};
    RLS_Init(&ctx->rls, 1e-5f, 0.99999f, initParams);

    ctx->k1 = 0.0f;
    ctx->k2 = 0.0f;
    ctx->k3 = 0.0f;
    ctx->estimatedPower = 0.0f;
    ctx->measuredPower = 0.0f;

    ctx->samples.data[0] = 0.0f;
    ctx->samples.data[1] = 0.0f;
}

/**
 * @brief 更新功率模型参数
 * @param ctx 功率控制结构体
 * @param effectivePower 有效功率（机械功率）
 */
void PowerControl_Update(PowerControlParam* ctx, const float effectivePower) {
    if(ctx == NULL)
        return;

    // 计算功率误差
    const float powerError = ctx->measuredPower - effectivePower - ctx->k3;

    // 更新RLS参数
    const Vector2* updatedParams = RLS_Update(&ctx->rls, &ctx->samples, powerError);

    // 更新模型参数
    ctx->k1 = (updatedParams->data[0] > 1e-5f) ? updatedParams->data[0] : 1e-5f;
    ctx->k2 = (updatedParams->data[1] > 1e-5f) ? updatedParams->data[1] : 1e-5f;

    // 估算总功率
    ctx->estimatedPower = ctx->k1 * ctx->samples.data[0] +
                          ctx->k2 * ctx->samples.data[1] +
                          effectivePower +
                          ctx->k3;
}

/**
 * @brief 收集电机数据
 * @param ctx 功率控制结构体
 * @param torqueFeedback 扭矩反馈数组
 * @param rpmFeedback 转速反馈数组
 * @param measuredPower 实际功率
 * @param motorCount 电机数量
 */
void PowerControl_CollectMotorData(PowerControlParam* ctx,
                                   const float* torqueFeedback,
                                   const float* rpmFeedback,
                                   const float measuredPower,
                                   const int motorCount) {
    float sumSpeed = 0.0f;
    float sumTorqueSq = 0.0f;

    for (int i = 0; i < motorCount; i++) {
        // 转换RPM为角速度 (rad/s)
        const float angularVelocity = rpmFeedback[i] * (3.1415926535f / 30.0f);

        sumSpeed += fabsf(angularVelocity);
        sumTorqueSq += torqueFeedback[i] * torqueFeedback[i];
    }

    ctx->samples.data[0] = sumSpeed;
    ctx->samples.data[1] = sumTorqueSq;
    ctx->measuredPower = measuredPower;
}

float* PowerControl_GetParam(const PowerControlParam* ctx)
{
    static float k[2] = {0.0f, 0.0f};
    k[0] = ctx->k1;
    k[1] = ctx->k2;
    return k;
}

void PowerControl_ResetRLS(PowerControlParam* ctx) {
    if(ctx == NULL)
        return;
    RLS_Reset(&ctx->rls);

    ctx->k1 = ctx->rls.defaultParams.data[0];
    ctx->k2 = ctx->rls.defaultParams.data[1];
}
