#include "CBoard_gimbal.h"

#include "user_lib.h"

void CBoard_RefereeCallback(const uint8_t* data, void* arg);
void CBoard_Gimbal_Init(cboard_gimbal_t* cbord_gimbal_ptr, CAN_HandleTypeDef* hcan)
{
    if (cbord_gimbal_ptr == NULL || hcan == NULL)
        return;
    memset(cbord_gimbal_ptr, 0, sizeof(cboard_gimbal_t));
    cbord_gimbal_ptr->can_instance = BSP_CAN_Init(hcan);
    BSP_CAN_RegisterCallback(cbord_gimbal_ptr->can_instance, CMD_ID_REFEREE_GET, CAN_ID_STD, CBoard_RefereeCallback, cbord_gimbal_ptr);
}

void CBoard_RefereeCallback(const uint8_t* data, void* arg)
{
    if (data == NULL || arg == NULL)
        return;
    cboard_gimbal_t* cbord_gimbal_ptr = (cboard_gimbal_t*)arg;
    cbord_gimbal_ptr->game_start = data[0] & 0x01;
    cbord_gimbal_ptr->camp = (data[0] >> 1) & 0x01;
    cbord_gimbal_ptr->attitude = (data[0] >> 2) & 0x01;
    cbord_gimbal_ptr->shoot_heat = (uint16_t)(data[2] << 8 | data[1]);
    cbord_gimbal_ptr->bullet_allow = (uint16_t)(data[4] << 8 | data[3]);
}

void CBoard_RC_Transmit(const cboard_gimbal_t* cbord_gimbal_ptr, const int16_t ch[4], const uint8_t sw[2], const int16_t roll)
{
    if (cbord_gimbal_ptr == NULL)
        return;
    uint8_t data[8];

    const int16_t ch0 = (int16_t)(ch[0] + 1024);
    const int16_t ch1 = (int16_t)(ch[1] + 1024);
    const int16_t ch2 = (int16_t)(ch[2] + 1024);
    const int16_t ch3 = (int16_t)(ch[3] + 1024);

    data[0] = ch0 & 0xFF;
    data[1] = ((ch0 >> 8) & 0x07) | ((ch1 << 3) & 0xF8);
    data[2] = ((ch1 >> 5) & 0x3F) | ((ch2 << 6) & 0xC0);
    data[3] = (ch2 >> 2) & 0xFF;
    data[4] = ((ch2 >> 10) & 0x01) | ((ch3 << 1) & 0xFE);
    data[5] = ((ch3 >> 7) & 0x0F) | ((sw[0] & 0x03) << 4) | ((sw[1] & 0x03) << 6);
    data[6] = roll & 0xFF;
    data[7] = (roll >> 8) & 0xFF;

    BSP_CAN_Transmit(cbord_gimbal_ptr->can_instance, CMD_ID_RC_TRAN, CAN_ID_STD, data, 8);
}

void CBoard_IMU_Transmit(const cboard_gimbal_t* cbord_gimbal_ptr, const fp32 imu_yaw)
{
    if (cbord_gimbal_ptr == NULL)
        return;
    uint8_t data[8];

    pack_float_to_4bytes(imu_yaw, &data[0]);
    memset(data + 4, 0, 4);

    BSP_CAN_Transmit(cbord_gimbal_ptr->can_instance, CMD_ID_IMU_TRAN, CAN_ID_STD, data, 8);
}