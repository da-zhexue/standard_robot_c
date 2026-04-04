#include "nuc.h"
#include "crc.h"
#include "usart.h"
#include "user_lib.h"

void NucRxCallback(uint8_t* data, uint16_t len);

nuc_ctrl_t *nuc_ins = NULL;
void NUC_Init(nuc_ctrl_t* nuc_ptr, UART_HandleTypeDef *nuc_uart)
{
    if (nuc_ptr == NULL)
        return;
    memset(nuc_ptr, 0, sizeof(nuc_ctrl_t));
    nuc_ins = nuc_ptr;
    BSP_UART_Init(&nuc_ptr->nuc_uart, nuc_uart, NucRxCallback, NULL, TX_BUFFER_SIZE, RX_BUFFER_SIZE);
}

void NUC_Chassis_Handler(const uint8_t* data, uint16_t len);
void NUC_Gimbal_Handler(const uint8_t* data, uint16_t len);
void NucRxCallback(uint8_t* data, const uint16_t len)
{
    for (int i = 0; i < len; i++)
    {
        if (data[i] == 0xA5 && i+4+2<len-1)
        {
            uint8_t *rx_data = &data[i];
            if (!Verify_CRC8_Check_Sum(rx_data, 5))
                return ;

            const uint16_t data_length = rx_data[1] | (rx_data[2] << 8);
            // if(rx_data[3] != seq_last +1 && rx_data > seq_last){
            //     lost_times++;
            //     lost_times_percent_times++;
            // }
            // if(rx_data[3] < seq_last){
            //     lost_times_percent = (float)lost_times_percent_times/256.0f;
            //     lost_times_percent_times =0;
            //
            // }
            // seq_last = rx_data[3];
            if(i+4+2+data_length+2 < len-1)
                if (!Verify_CRC16_Check_Sum(rx_data, data_length + 9))
                    return ;

            const uint16_t cmd_id = (rx_data[6] << 8) | rx_data[5];

            switch (cmd_id)
            {
                case CMD_ID_SELF_DECISION:
                    memcpy(&nuc_ins->self_decision, &rx_data[7], sizeof(uint32_t));
                    break;
                case CMD_ID_CHASSIS_CTRL:
                    NUC_Chassis_Handler(&rx_data[7], data_length);
                    break;
                case CMD_ID_BIG_GIMBAL_CTRL:
                    NUC_Gimbal_Handler(&rx_data[7], data_length);
                     break;
                default :
                    break;
            }

        }
    }
}

void NUC_Chassis_Handler(const uint8_t* data, const uint16_t len)
{
    if (nuc_ins == NULL)
        return;
    if (len < 9)
        return;

    unpack_4bytes_to_floats(&data[0], &nuc_ins->vx);
    unpack_4bytes_to_floats(&data[4], &nuc_ins->vy);
    nuc_ins->chassis_spin = data[8] & 0x01;
    nuc_ins->gimbal_scan = (data[8] >> 1) & 0x01;
    nuc_ins->use_cap = (data[8] >> 2) & 0x01;
}

void NUC_Gimbal_Handler(const uint8_t* data, const uint16_t len)
{
    if (nuc_ins == NULL)
        return;
    if (len < 4)
        return;

    unpack_4bytes_to_floats(&data[0], &nuc_ins->gimbal_yaw);
}
