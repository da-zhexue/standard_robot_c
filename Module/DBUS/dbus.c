#include "dbus.h"
#include "usart.h"

static rc_instance* rc_ins_ptr = NULL;

void RemoteDataProcess(uint8_t *data, uint16_t len);
void dbus_init(rc_instance* rc_ins)
{
    BSP_UART_Init(&rc_ins->dbus_usart, &DBUS_HUART, RemoteDataProcess, NULL, 0, DBUS_MAX_LEN);
    memset(&rc_ins->rc_data, 0, sizeof(rc_data_t));
    rc_ins_ptr = rc_ins;
}

void RemoteDataProcess(uint8_t *data, const uint16_t len)
{
    rc_data_t* rc_data = &rc_ins_ptr->rc_data;
    if (data == NULL || len != 18)
    {
        return;
    }
    rc_data->ch0 = (int16_t)(((int16_t)data[0] | ((int16_t)data[1] << 8)) & 0x07FF);
    rc_data->ch1 = (int16_t)((((int16_t)data[1] >> 3) | ((int16_t)data[2] << 5)) & 0x07FF);
    rc_data->ch2 = (int16_t)((((int16_t)data[2] >> 6) | ((int16_t)data[3] << 2) | ((int16_t)data[4] << 10)) & 0x07FF);
    rc_data->ch3 = (int16_t)((((int16_t)data[4] >> 1) | ((int16_t)data[5] << 7)) & 0x07FF);
    rc_data->ch0 -= RC_CH_VALUE_OFFSET;
    rc_data->ch1 -= RC_CH_VALUE_OFFSET;
    rc_data->ch2 -= RC_CH_VALUE_OFFSET;
    rc_data->ch3 -= RC_CH_VALUE_OFFSET;
    rc_data->s1 = ((data[5] >> 4) & 0x0003);
    rc_data->s2 = ((data[5] >> 4) & 0x000C) >> 2;
    // | ((int16_t)data[15] << 8);

}
