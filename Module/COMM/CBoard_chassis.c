#include "CBoard_chassis.h"
#include "user_lib.h"
#include <string.h>

void CBoard_IMU_handler(const uint8_t* data, uint32_t id, void* arg);
void CBoard_Chassis_Init(cboard_chassis_t* cbord_chassis_ptr, CAN_HandleTypeDef* can_handle)
{
    if (cbord_chassis_ptr == NULL || can_handle == NULL)
        return;
    memset(cbord_chassis_ptr, 0, sizeof(cboard_chassis_t));
    cbord_chassis_ptr->can_instance = BSP_CAN_Init(can_handle);
    BSP_CAN_RegisterStdCallback(cbord_chassis_ptr->can_instance, CMD_ID_IMU_GET, CBoard_IMU_handler, &cbord_chassis_ptr->imu_angle);
}

void CBoard_Referee_Tranmit(const cboard_chassis_t* cbord_chassis_ptr, const uint8_t game_start, const uint8_t camp, const uint8_t attitude, const uint16_t shoot_heat, const uint16_t bullet_allow)
{
    if (cbord_chassis_ptr == NULL)
        return;
    uint8_t data[8];
    data[0] = (game_start & 0x01) | ((camp & 0x01) << 1) | ((attitude & 0x01) << 2);
    data[1] = 0; // 预留
    data[2] = shoot_heat & 0xFF;
    data[3] = (shoot_heat >> 8) & 0xFF;
    data[4] = bullet_allow & 0xFF;
    data[5] = (bullet_allow >> 8) & 0xFF;
    data[6] = 0; // 预留
    data[7] = 0; // 预留
    BSP_CAN_Transmit(cbord_chassis_ptr->can_instance, CMD_ID_REFEREE_TRAN, CAN_ID_STD, data, 8);
}

void CBoard_IMU_handler(const uint8_t* data, uint32_t id, void* arg)
{
    if (data == NULL || arg == NULL)
        return;
    fp32* imu_angle = (fp32*)arg;
    unpack_4bytes_to_floats(&data[0], &imu_angle[0]);
}