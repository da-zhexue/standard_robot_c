#include "super_cap.h"
#include "user_lib.h"

void get_supercap_measure(const uint8_t* rx_data, void* arg);
void super_cap_init(super_cap_instance* supercap_ins, CAN_HandleTypeDef *hcan)
{
    supercap_ins->can_ins = BSP_CAN_Init(hcan);
    memset(&supercap_ins->power_data, 0, sizeof(supercap_ins->power_data));
    BSP_CAN_RegisterCallback(supercap_ins->can_ins, SUPERCAP_RX_ID, CAN_ID_STD, get_supercap_measure, &supercap_ins->power_data);
}

void super_cap_set_power(const super_cap_instance* supercap_ins, fp32 expected_power)
{
    uint8_t txdata[8];
    if (expected_power > SUPERCAP_MAXPOWER)
        expected_power = SUPERCAP_MAXPOWER;
    pack_float_to_4bytes(expected_power, &txdata[0]);
    BSP_CAN_Transmit(supercap_ins->can_ins, 0x300, CAN_ID_STD, txdata, 8);
}

void get_supercap_measure(const uint8_t* rx_data, void* arg)
{
    if(rx_data == NULL || arg == NULL)
        return;

    power_data_t *power_data = (power_data_t*)arg;
    unpack_4bytes_to_floats(&rx_data[0], &power_data->total_power);
    unpack_4bytes_to_floats(&rx_data[4], &power_data->referee_power);
    power_data->supercap_power = power_data->total_power - power_data->referee_power;
}