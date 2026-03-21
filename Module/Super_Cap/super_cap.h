#ifndef STANDARD_ROBOT_C_SUPER_CAP_H
#define STANDARD_ROBOT_C_SUPER_CAP_H
#include "typedef.h"
#include "can/bsp_can.h"

#define SUPERCAP_TX_ID 0x300
#define SUPERCAP_RX_ID 0x301

#define SUPERCAP_MAXPOWER 200.0f

typedef struct
{
    fp32 total_power;
    fp32 referee_power;
    fp32 supercap_power;
} power_data_t;

typedef struct
{
    CAN_Instance_t *can_ins;
    power_data_t power_data;
} super_cap_instance;

void super_cap_init(super_cap_instance* supercap_ins, CAN_HandleTypeDef *hcan);
void super_cap_set_power(const super_cap_instance* supercap_ins, fp32 expected_power);

#endif //STANDARD_ROBOT_C_SUPER_CAP_H