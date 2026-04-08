#include "nx.h"

void NX_CtrlCallback(const uint8_t *data, uint32_t id, void *arg);
void NX_Init(nx_ctrl_t *nx_ctrl, CAN_HandleTypeDef *hcan)
{
    if (nx_ctrl == NULL || hcan == NULL)
        return;
    memset(nx_ctrl, 0, sizeof(nx_ctrl_t));
    nx_ctrl->nx_can = BSP_CAN_Init(hcan);
    BSP_CAN_RegisterStdCallback(nx_ctrl->nx_can, CMD_ID_GIMBAL_CTRL, NX_CtrlCallback, nx_ctrl);
}

void NX_CtrlCallback(const uint8_t *data, uint32_t id, void *arg)
{
    if (data == NULL || arg == NULL)
        return;
    nx_ctrl_t *nx_ctrl = (nx_ctrl_t *)arg;
    nx_ctrl->fire = data[0] & 0x01;
    nx_ctrl->scan = (data[0] >> 1) & 0x01;
    const int16_t yaw = (int16_t)(data[1] | (data[2] << 8));
    const int16_t pitch = (int16_t)(data[3] | (data[4] << 8));
    nx_ctrl->target_yaw = (float)yaw / 100.0f;   // 转换为度
    nx_ctrl->target_pitch = (float)pitch / 100.0f; // 转换为度
}

void NX_SendAngle(const nx_ctrl_t *nx_ctrl_ptr,const fp32 q[4])
{
    int16_t q_int[4];
    uint8_t data[8];
    for (int i = 0; i < 4; i++)
    {
        q_int[i] = (int16_t)(q[i] * 10000.0f);
        data[2*i] = q_int[i] & 0xFF;
        data[2*i+1] = (q_int[i] >> 8) & 0xFF;
    }
    BSP_CAN_Transmit(nx_ctrl_ptr->nx_can, CMD_ID_GIMBAL_ANGLE, CAN_ID_STD, data, 8);
}