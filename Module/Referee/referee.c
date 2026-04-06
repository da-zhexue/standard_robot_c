#include "Referee/referee.h"
#include <string.h>
#include "crc.h"
#include "usart.h"
#include "NUC/nuc.h"

static referee_t* referee_ins;

void Referee_RxCallback(uint8_t* data, uint16_t len);
void Referee_Init(referee_t *referee_ptr, UART_HandleTypeDef *referee_uart)
{
    if (referee_ptr == NULL)
        return;
    memset(referee_ptr, 0, sizeof(referee_t));
    referee_ins = referee_ptr;
    BSP_UART_Init(&referee_ptr->uart_instance, referee_uart, 115200, Referee_RxCallback, NULL, 0, RX_BUFFER_SIZE);
}

void Referee_RxCallback(uint8_t* data, const uint16_t len)
{
    for (int i = 0; i < len; i++)
    {
        if (data[i] == 0xA5 && i+7<len)
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
            case CMD_ID_GAME_STATUS:
                memcpy(&referee_ins->game_status, &rx_data[7], sizeof(game_status_t));
                break;
            case CMD_ID_GAME_RESULT:
                memcpy(&referee_ins->game_result, &rx_data[7], sizeof(game_result_t));
                break;
            case CMD_ID_GAME_ROBOT_HP:
                memcpy(&referee_ins->game_robot_hp, &rx_data[7], sizeof(game_robot_HP_t));
                break;
            case CMD_ID_EVENT_DATA:
                memcpy(&referee_ins->event_data, &rx_data[7], sizeof(event_data_t));
                break;
            case CMD_ID_REFEREE_WARNING:
                memcpy(&referee_ins->referee_warning, &rx_data[7], sizeof(referee_warning_t));
                break;
            case CMD_ID_DART_INFO:
                memcpy(&referee_ins->dart_info, &rx_data[7], sizeof(dart_info_t));
                break;
            case CMD_ID_ROBOT_PERFORMANCE:
                memcpy(&referee_ins->robot_performance, &rx_data[7], sizeof(robot_performance_t));
                break;
            case CMD_ID_POWER_HEAT_DATA:
                memcpy(&referee_ins->power_heat_data, &rx_data[7], sizeof(power_heat_data_t));
                break;
            case CMD_ID_ROBOT_POS:
                memcpy(&referee_ins->robot_pos, &rx_data[7], sizeof(robot_pos_t));
                break;
            case CMD_ID_BUFF_DATA:
                memcpy(&referee_ins->buff_data, &rx_data[7], sizeof(buff_data_t));
                break;
            case CMD_ID_HURT_DATA:
                memcpy(&referee_ins->hurt_data, &rx_data[7], sizeof(hurt_data_t));
                break;
            case CMD_ID_SHOOT_DATA:
                memcpy(&referee_ins->shoot_data, &rx_data[7], sizeof(shoot_data_t));
                break;
            case CMD_ID_PROJECTILE_ALLOWANCE:
                memcpy(&referee_ins->projectile_allowance, &rx_data[7], sizeof(projectile_allowance_t));
                break;
            case CMD_ID_RFID_STATUS:
                memcpy(&referee_ins->rfid_status, &rx_data[7], sizeof(rfid_status_t));
                break;
            case CMD_ID_DART_CLIENT_CMD:
                memcpy(&referee_ins->dart_client_cmd, &rx_data[7], sizeof(dart_client_cmd_t));
                break;
            case CMD_ID_ROBOT_POS_EXT:
                memcpy(&referee_ins->robot_pos_ext, &rx_data[7], sizeof(robot_pos_ext_t));
                break;
            case CMD_ID_MARK_PROGRESS:
                memcpy(&referee_ins->mark_progress, &rx_data[7], sizeof(mark_progress_t));
                break;
            case CMD_ID_SENTRY_INFO:
                memcpy(&referee_ins->sentry_info, &rx_data[7], sizeof(sentry_info_t));
                break;
                // case CMD_ID_RADAR_INFO :
                //     memcpy(&referee_ins.radar_info, &rx_data[7], sizeof(radar_info_t));
                // break;
            default :
                break;
            }
            NUC_Referee_Tran(rx_data, data_length + 9);
        }
    }
}

void RefereeCmd_PackPacket(const referee_t* referee_ptr, const uint8_t *data_in, const uint16_t data_len, const uint16_t cmd_id)
{
    const uint16_t total_len = FRAME_HEADER_LEN + CMD_ID_LEN + data_len + 2;
    uint8_t data_out[total_len];
    uint32_t offset = 0;
    data_out[offset++] = SOF_VALUE;
    data_out[offset++] = data_len & 0xFF;
    data_out[offset++] = (data_len >> 8) & 0xFF;

    data_out[offset++] = 0;
    data_out[offset++] = 0;
    data_out[offset++] = cmd_id & 0xFF;
    data_out[offset++] = (cmd_id >> 8) & 0xFF;
    if (data_in != NULL && data_len > 0) {
        memcpy(&data_out[offset], data_in, data_len);
        offset += data_len;
    }
    Append_CRC8_Check_Sum(data_out, FRAME_HEADER_LEN);
    Append_CRC16_Check_Sum(data_out, total_len);

    BSP_UART_Transmit_To_Mail(&referee_ptr->uart_instance, data_out, sizeof(data_out), 100);
}

void Referee_SendSentryCmd(const referee_t* referee_ptr,
    const uint8_t progress_revive, // 读条复活
    const uint8_t exchange_revive, // 兑换复活
    const uint16_t exchange_projectile, // 兑换发弹量
    const uint8_t remote_projectile_req, // 兑换发弹量的请求次数
    const uint8_t remote_hp_req, // 兑换血量的请求次数
    const uint8_t posture, // 修改当前姿态指令
    const uint8_t trigger_energy // 是否确认使能量机关进入正在激活状态
    )
{
    sentry_cmd_t sentry_cmd = {0};

    // bit 0
    if (progress_revive) sentry_cmd.sentry_cmd |= (1 << 0);
    // bit 1
    if (exchange_revive) sentry_cmd.sentry_cmd |= (1 << 1);
    // bit 2-12 (11 bits)
    sentry_cmd.sentry_cmd |= ((uint32_t)(exchange_projectile & 0x7FF) << 2);
    // bit 13-16 (4 bits)
    sentry_cmd.sentry_cmd |= ((uint32_t)(remote_projectile_req & 0xF) << 13);
    // bit 17-20 (4 bits)
    sentry_cmd.sentry_cmd |= ((uint32_t)(remote_hp_req & 0xF) << 17);
    // bit 21-22 (2 bits)
    sentry_cmd.sentry_cmd |= ((uint32_t)(posture & 0x3) << 21);
    // bit 23 (1 bit)
    if (trigger_energy) sentry_cmd.sentry_cmd |= (1 << 23);
    // bit 24-31 are reserved and default to 0

    RefereeCmd_PackPacket(referee_ptr, (uint8_t*)&sentry_cmd, sizeof(sentry_cmd), CMD_ID_SENTRY_SELFDECISION);
}