#include "vofa.h"

#include "user_lib.h"

void vofacallback(uint8_t* data, uint16_t len);

vofa_t* vofa_ptr = NULL;

void vofa_init(vofa_t* vofa, UART_HandleTypeDef* huart, const uint16_t txnum, const uint16_t rxnum)
{
    vofa->tx_num = txnum;
    if (rxnum > VOFA_RX_NUM_MAX)
        return ;
    vofa->rx_num = rxnum;
    vofa->rx_cnt = 0;
    BSP_UART_Init(&vofa->vofa_uart, huart, 115200, vofacallback, NULL, TX_BUFFER_SIZE, RX_BUFFER_SIZE);
    vofa_ptr = vofa;
}

void vofacallback(uint8_t* data, uint16_t len)
{
    const uint16_t rxlen = vofa_ptr->rx_num * sizeof(fp32);
    if(data[0] != PARAM_HEADER || data[3] != 0 || data[4] != 0x5A)
        return ;
    if(data[1] != rxlen)
        return ;
    if (data[5] > 0 && data[5] <= vofa_ptr->rx_num)
    {
        unpack_4bytes_to_floats(&data[7], &vofa_ptr->rxdata[data[5]]);
        vofa_ptr->rx_cnt |= (1 << (data[5] - 1));
    }
    else if (data[5] == 0xFF)
    {
        __HAL_RCC_CLEAR_RESET_FLAGS();
        osDelay(100);
        HAL_NVIC_SystemReset();
    }
    if (vofa_ptr->rx_cnt == (1 << vofa_ptr->rx_num) - 1)
    {
        vofa_ptr->ready = 1;
        vofa_ptr->rx_cnt = 0;
    }
}

void vofa_print(const vofa_t* vofa, const fp32* data)
{
    const uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7F};
    uint8_t txdata[vofa->tx_num * sizeof(fp32) + 4];
    memcpy(txdata, data, vofa->tx_num * sizeof(fp32));
    memcpy(txdata + vofa->tx_num * sizeof(fp32), tail, 4);
    BSP_UART_Transmit_To_Mail(&vofa->vofa_uart, (uint8_t*)txdata, vofa->tx_num * sizeof(fp32) + 4, 100);
}