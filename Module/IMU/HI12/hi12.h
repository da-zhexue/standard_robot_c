#ifndef STANDARD_ROBOT_C_HI12_H
#define STANDARD_ROBOT_C_HI12_H
#include "typedef.h"
#include "usart/bsp_uart.h"

#pragma pack(push, 1)
typedef struct {
    uint8_t header[6];
    uint8_t tag;
    uint16_t main_status;
    int8_t temperature;
    float air_pressure;
    uint32_t system_time;
    float acc_b[3];
    float gyr_b[3];
    float mag_b[3];
    float roll;
    float pitch;
    float yaw;
    float quat[4];
} SensorDataPacket;
#pragma pack(pop)

typedef struct
{
    UART_Instance_t imu_uart;
    SensorDataPacket imu_data;
} hi12_t;

void HI12_Init(hi12_t* hi12_ptr, UART_HandleTypeDef* uart_handle);

#endif //STANDARD_ROBOT_C_HI12_H