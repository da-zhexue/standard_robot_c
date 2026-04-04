#include "hi12.h"

#include "crc.h"

void imuCallback(uint8_t* data, uint16_t len);
static hi12_t* hi12_ins = NULL;

void HI12_Init(hi12_t* hi12_ptr, UART_HandleTypeDef* uart_handle)
{
    if (hi12_ptr == NULL || uart_handle == NULL)
        return;
    hi12_ins = hi12_ptr;
    BSP_UART_Init(&hi12_ptr->imu_uart, uart_handle, imuCallback, NULL, 0, RX_BUFFER_SIZE);
}

void imuCallback(uint8_t* data, const uint16_t len)
{
    if(len <82)
        return;
    for(int i=0;i<len;i++)
    {
        if(data[i]==0x5A&&data[i+1]==0xA5)
        {
            const uint16_t data_len = (uint16_t)data[i+3]<<8 | data[i+2];
            uint16_t crc = 0;

            crc16_update(&crc, &data[i], 4);
            crc16_update(&crc, &data[i] + 6, data_len);

            if(crc == ((uint16_t)data[i+5]<<8 | data[i+4]))
            {
                memcpy(&hi12_ins->imu_data, &data[i], sizeof(SensorDataPacket));
                break;
            }
        }
    }
}