#ifndef STANDARD_ROBOT_C_CBOARD_CHASSIS_H
#define STANDARD_ROBOT_C_CBOARD_CHASSIS_H
#include "user_lib.h"
#include "can/bsp_can.h"

typedef enum
{
    CMD_ID_REFEREE_TRAN = 0x601,
    CMD_ID_RC_GET = 0x602, // 为统一，将can总线上传输的遥控器数据也放在了DBUS内，方便底盘控制器调用，这里只是说明这个数据是从云台发向底盘的
    CMD_ID_IMU = 0x0CFF46,
} cbord_chassis_cmd_id_t;

typedef struct
{
    CAN_Instance_t* can_instance;
    fp32 imu_angle[3];
} cbord_chassis_t;

void CBoard_Chassis_Init(cbord_chassis_t* cbord_chassis_ptr, CAN_HandleTypeDef* can_handle);
void CBoard_Referee_Tranmit(const cbord_chassis_t* cbord_chassis_ptr, uint8_t game_start, uint8_t camp, uint8_t attitude, uint16_t shoot_heat, uint16_t bullet_allow);

#endif //STANDARD_ROBOT_C_CBOARD_CHASSIS_H