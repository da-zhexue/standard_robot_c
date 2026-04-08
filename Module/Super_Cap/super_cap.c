#include "super_cap.h"
#include "user_lib.h"

void get_supercap_power(const uint8_t* rx_data, uint32_t id, void* arg);
void get_supercap_remain(const uint8_t* rx_data, uint32_t id, void* arg);
void super_cap_init(super_cap_instance* supercap_ins, CAN_HandleTypeDef *hcan)
{
    supercap_ins->can_ins = BSP_CAN_Init(hcan);
    memset(&supercap_ins->power_data, 0, sizeof(supercap_ins->power_data));
    BSP_CAN_RegisterStdCallback(supercap_ins->can_ins, SUPERCAP_RX_POWER_ID, get_supercap_power, &supercap_ins->power_data);
    BSP_CAN_RegisterStdCallback(supercap_ins->can_ins, SUPERCAP_RX_REMAIN_ID, get_supercap_remain, &supercap_ins->power_data);
}

void super_cap_use(const super_cap_instance* supercap_ins, const uint8_t use)
{
    uint8_t txdata[8];
    txdata[0] = use ? 1 : 0;
    BSP_CAN_Transmit(supercap_ins->can_ins, SUPERCAP_TX_ID, CAN_ID_STD, txdata, 8);
}

void get_supercap_power(const uint8_t* rx_data, uint32_t id, void* arg)
{
    if(rx_data == NULL || arg == NULL)
        return;

    power_data_t *power_data = (power_data_t*)arg;
    unpack_4bytes_to_floats(&rx_data[0], &power_data->total_power);
    unpack_4bytes_to_floats(&rx_data[4], &power_data->referee_power);
    power_data->supercap_power = power_data->total_power - power_data->referee_power;
}

void get_supercap_remain(const uint8_t* rx_data, uint32_t id, void* arg)
{
    if(rx_data == NULL || arg == NULL)
        return;

    power_data_t *power_data = (power_data_t*)arg;
    unpack_4bytes_to_floats(&rx_data[0], &power_data->remain_v);
}