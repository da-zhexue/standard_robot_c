#ifndef STANDARD_ROBOT_C_NX_H
#define STANDARD_ROBOT_C_NX_H
#include "can/bsp_can.h"
#include "typedef.h"

typedef enum
{
    CMD_ID_GIMBAL_CTRL = 0x404,
    CMD_ID_GIMBAL_ANGLE = 0x504,
} nx_cmd_id_t;

typedef struct
{
    CAN_Instance_t* nx_can;
    uint8_t fire;
    uint8_t scan;
    fp32 target_yaw;
    fp32 target_pitch;
} nx_ctrl_t;

void NX_Init(nx_ctrl_t *nx_ctrl, CAN_HandleTypeDef *hcan);
void NX_SendAngle(const nx_ctrl_t *nx_ctrl_ptr,const fp32 q[4]);

#endif //STANDARD_ROBOT_C_NX_H