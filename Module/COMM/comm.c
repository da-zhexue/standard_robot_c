/* 通信板仅做哨兵临时方案，希望后续能够去掉 */

#include "comm.h"
#include "crc.h"
#include "user_lib.h"
#include "usart.h"

void cmd_move_handler(const uint8_t* data);
void cmd_rotate_handler(const uint8_t* data);
void cmd_shart_handler(const uint8_t* data);
void cmd_mode_handler(const uint8_t* data);
void cmd_imu_s_handler(const uint8_t* data);
void cmd_imu_l_handler(const uint8_t* data);
void cmd_buffer_handler(const uint8_t* data);
void cmd_debug_handler(const uint8_t* data);
void cmd_gimbal_rotate_handler(const uint8_t* data);
void cmd_onlinecb_handler(const uint8_t* data);

void comm_send_start();
void comm_send_onlinecb();

void comm_decode(uint8_t *data, uint16_t len);

static comm_t* comm_ins = NULL;
void comm_init(comm_t* comm_instance, UART_HandleTypeDef* comm_uart)
{
	if (comm_instance == NULL)
		return;
	comm_ins = comm_instance;
	memset(comm_instance, 0, sizeof(comm_t));

	BSP_UART_Init(&comm_instance->uart_instance, comm_uart, comm_decode, NULL, 128, 128);
}

void comm_decode(uint8_t *data, const uint16_t len)
{
	// if(!upc_ptr.start_upc_flag)
	// 	return 0;
	if(data[0] != SOF_VALUE || data[3] != 0)
		return ;
	const uint16_t data_len = (data[2] << 8) | data[1];
	if (len < data_len + 2)
		return ;
	if(!Verify_CRC8_Check_Sum(data, FRAME_HEADER_LEN) || !Verify_CRC16_Check_Sum(data , data_len+9))
		return ;

	const uint16_t cmd_id = (data[6] << 8) | data[5];

	switch(cmd_id)
	{
		case CMD_MOVE:
			cmd_move_handler(&data[FRAME_HEADER_LEN+CMD_ID_LEN]);
			break;
		case CMD_ROTATE:
			cmd_rotate_handler(&data[FRAME_HEADER_LEN+CMD_ID_LEN]);
			break;
		case CMD_START:
			cmd_shart_handler(&data[FRAME_HEADER_LEN+CMD_ID_LEN]);
			break;
		case CMD_MODE:
			cmd_mode_handler(&data[FRAME_HEADER_LEN+CMD_ID_LEN]);
			break;
		case CMD_IMU_L_INFO:
			cmd_imu_l_handler(&data[FRAME_HEADER_LEN+CMD_ID_LEN]);
			break;
		case CMD_ONLINECB:
			cmd_onlinecb_handler(&data[FRAME_HEADER_LEN+CMD_ID_LEN]);
			break;
		case CMD_POWER:
			cmd_buffer_handler(&data[FRAME_HEADER_LEN+CMD_ID_LEN]);
			break;
		case CMD_GIMBAL_ROTATE:
			cmd_gimbal_rotate_handler(&data[FRAME_HEADER_LEN+CMD_ID_LEN]);
			break;
//		case CMD_DEBUG:
//			cmd_debug_handler(&rx_data[FRAME_HEADER_LEN+CMD_ID_LEN]);
//			break;
		default:
			break;
	}

}

uint16_t last_bag = 0;
uint16_t loss_bag = 0;
float loss_rate = 0.0f;
void cmd_debug_handler(const uint8_t* data)
{
	const uint16_t bag = (data[0] << 8) | data[1];
	loss_bag += (bag - last_bag - 1);
	loss_rate = (float)loss_bag / (float)bag * 1.0f;
	last_bag = bag;
}

void cmd_move_handler(const uint8_t* data)
{
	unpack_4bytes_to_floats(&data[0], &comm_ins->comm_ctrl_param.vx);
	unpack_4bytes_to_floats(&data[4], &comm_ins->comm_ctrl_param.vy);
	unpack_4bytes_to_floats(&data[8], &comm_ins->comm_ctrl_param.vw);
	//LOG_INFO("Get move cmd vx: %.2f, vy: %.2f, vw: %.2f", upc_ptr->vx, upc_ptr->vy, upc_ptr->vw);
}

void cmd_rotate_handler(const uint8_t* data)
{
	unpack_4bytes_to_floats(&data[0], &comm_ins->comm_ctrl_param.chassis_yaw);
	unpack_4bytes_to_floats(&data[4], &comm_ins->comm_ctrl_param.gimbal_yaw);
	//LOG_INFO("Get rotate cmd chassis: %.2f, gimbal: %.2f", upc_ptr->chassis_yaw, upc_ptr->gimbal_yaw);
}

void cmd_mode_handler(const uint8_t* data)
{
	comm_ins->comm_ctrl_param.chassis_spin = data[12];
}

void cmd_imu_l_handler(const uint8_t* data)
{
	unpack_4bytes_to_floats(&data[0], &comm_ins->big_gimbal_angle[0]);
	unpack_4bytes_to_floats(&data[4], &comm_ins->big_gimbal_angle[1]);
	unpack_4bytes_to_floats(&data[8], &comm_ins->big_gimbal_angle[2]);
}

void cmd_shart_handler(const uint8_t* data)
{
	if (data[0] == 1)
	{
		comm_ins->game_start = 1;
		comm_send_start();
	}
}

void cmd_onlinecb_handler(const uint8_t* data)
{
	if (data[0] == 1)
	{
		comm_send_onlinecb();
	}
}

void cmd_buffer_handler(const uint8_t* data)
{
	static fp32 buffer_ptr, power_max;
	unpack_4bytes_to_floats(&data[0], &buffer_ptr);
	unpack_4bytes_to_floats(&data[4], &power_max);
}

void cmd_gimbal_rotate_handler(const uint8_t* data)
{
	comm_ins->comm_ctrl_param.gimbal_scan = data[0];
}

void CMD_PackPacket(const uint8_t *data_in, const uint16_t data_len, const uint16_t cmd_id)
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

	BSP_UART_Transmit_To_Mail(&comm_ins->uart_instance, data_out, sizeof(data_out), 100);
}

void comm_send_attitude(const fp32 chassis_yaw)
{
	static uint8_t send_yaw[4] = {0};
	pack_float_to_4bytes(chassis_yaw, &send_yaw[0]);
	//LOG_INFO("send chassis yaw: %.2f", SEND_ATTITUDE, tf_ptr->Chassis_angle.yaw_deg);
	CMD_PackPacket(send_yaw, sizeof(send_yaw), SEND_ATTITUDE);
}
//
// void comm_send_onlinestate()
// {
// 	static uint8_t send_online[1] = {0};
// 	send_online[0] |= IS_ONLINE(M3508_0_ONLINE) < 0;
// 	send_online[0] |= IS_ONLINE(M3508_1_ONLINE) < 1;
// 	send_online[0] |= IS_ONLINE(M3508_2_ONLINE) < 2;
// 	send_online[0] |= IS_ONLINE(M3508_3_ONLINE) < 3;
// 	send_online[0] |= IS_ONLINE(M9025_ONLINE) < 4;
//
// 	CMD_PackPacket(send_online, sizeof(send_online), SEND_ONLINE);
// }

void comm_send_start()
{
	static uint8_t send_start[1] = {1};
	CMD_PackPacket(send_start, sizeof(send_start), SEND_START);
	//LOG_INFO("START!");
}

void comm_send_onlinecb()
{
	static uint8_t send_onlinecb[1] = {1};
	CMD_PackPacket(send_onlinecb, sizeof(send_onlinecb), SEND_ONLINECB);
}

void comm_send_cap()
{
	static uint8_t send_cap[12]; // 未完成
	CMD_PackPacket(send_cap, sizeof(send_cap), SEND_SUPERCAP);
}