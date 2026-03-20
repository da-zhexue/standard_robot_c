#include "mf9025.h"
#include "user_lib.h"

void mf9025_callback(const uint8_t* rx_data, void* arg);
MF9025_Status_t mf9025_init(mf9025_instance* mf9025_ins, CAN_HandleTypeDef *hcan, const uint32_t txid)
{
    if (txid < MF9025_TX_MIN || txid > MF9025_TX_MAX)
        return MF9025_ERROR_INVALID_PARAM;
    mf9025_ins->can_ins = BSP_CAN_Init(hcan);
    mf9025_ins->txid = txid;
    BSP_CAN_RegisterCallback(mf9025_ins->can_ins, txid, CAN_ID_STD, mf9025_callback, &mf9025_ins->ecd);
    return MF9025_OK;
}

// mf9025有非常多种控制方式，但是还是速度控制最好用，其他的不想写了
MF9025_Status_t mf9025_ctrl_speed(const mf9025_instance* mf9025_ins, uint16_t iqControl, uint32_t speedControl)
{
    if (mf9025_ins == NULL)
        return MF9025_ERROR;

    const CAN_Instance_t* can_ins = mf9025_ins->can_ins;
    uint8_t txdata[8];

    txdata[0] = CMD_9025_SPEED_CONTROL;
    txdata[1] = 0x00;
    txdata[2] = ((uint8_t *)&iqControl)[0];
    txdata[3] = ((uint8_t *)&iqControl)[1];
    txdata[4] = ((uint8_t *)&speedControl)[0];
    txdata[5] = ((uint8_t *)&speedControl)[1];
    txdata[6] = ((uint8_t *)&speedControl)[2];
    txdata[7] = ((uint8_t *)&speedControl)[3];

    if (BSP_CAN_Transmit(can_ins, mf9025_ins->txid, CAN_ID_STD, txdata, 8) != CAN_OK)
        return MF9025_ERROR;

    return MF9025_OK;
}

// param: CONTROL_PARAM_9025_ANGLE_PID CONTROL_PARAM_9025_SPEED_PID CONTROL_PARAM_9025_IQ_PID
MF9025_Status_t mf9025_set_pid(const mf9025_instance* mf9025_ins, const uint8_t param, uint16_t kp, uint16_t ki, uint16_t kd)
{
    if (mf9025_ins == NULL)
        return MF9025_ERROR;

    const CAN_Instance_t* can_ins = mf9025_ins->can_ins;
    uint8_t txdata[8];

    txdata[0] = CMD_9025_WRITE_CONTROL_PARAM;
    txdata[1] = param;
    txdata[2] = ((uint8_t *)&kp)[0];
    txdata[3] = ((uint8_t *)&kp)[1];
    txdata[4] = * (uint8_t*)(&ki);
    txdata[5] = *((uint8_t*)(&ki)+1);
    txdata[6] = *((uint8_t*)(&kd)+0);
    txdata[7] = *((uint8_t*)(&kd)+1);

    if (BSP_CAN_Transmit(can_ins, mf9025_ins->txid, CAN_ID_STD, txdata, 8) != CAN_OK)
        return MF9025_ERROR;

    return MF9025_OK;


}

void get_mf9025_measure(const uint8_t* rx_data, void* arg);
void mf9025_callback(const uint8_t* rx_data, void* arg)
{
    if(rx_data == NULL || arg == NULL)
        return;

    switch(rx_data[0])
    {
        case CMD_9025_READ_MEASURE:
        case CMD_9025_SPEED_CONTROL:
        case CMD_9025_ANGLE_CONTROL:
        case CMD_9025_INCREMENT_ANGLE_CONTROL:
            get_mf9025_measure(&rx_data[0], arg);
            break;
        case CMD_9025_READ_ENCODER:
            // get_motor_9025_ecd_data(motor_9025->motor_9025_ecd_data, rx_data);
            // 大概率用不到，不想写了
        case CMD_9025_READ_CONTROL_PARAM:
            // get_motor_9025_control_param(motor_9025->motor_9025_pid, rx_data);
            // 可以用上位机读取
        default:
            break;
    }
}

void get_mf9025_measure(const uint8_t* rx_data, void* arg)
{
    if(rx_data == NULL || arg == NULL)
        return;

    mf9025_ecd_t *ecd = (mf9025_ecd_t*)arg;
    ecd->temperature = (int8_t)   (rx_data)[1];
    ecd->iq        = (int16_t)  ((rx_data)[3]<<8 | (rx_data)[2]);
    ecd->speed     = (int16_t)  ((rx_data)[5]<<8 | (rx_data)[4]);
    ecd->ecd       = (uint16_t) ((rx_data)[7]<<8 | (rx_data)[6]);
    if (ecd->ecd_offset == 0)
    {
        ecd->ecd_offset = ecd->ecd;
        ecd->zero_offset = (ecd->ecd > MF9025_ECD_IN_ZERO) ? (ecd->ecd - MF9025_ECD_IN_ZERO) : (ecd->ecd + MF9025_ECD_MAX - MF9025_ECD_IN_ZERO);
    }
}
