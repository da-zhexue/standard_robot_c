/**
 ******************************************************************************
 * @file    ins_task.h
 * @author  Wang Hongxi
 * @version V2.0.0
 * @date    2022/2/23
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#ifndef __INS_TASK_H
#define __INS_TASK_H

#include "IMU/BMI088/BMI088driver.h"

#define INS_TASK_PERIOD 1

typedef struct
{
    float q[4]; // 四元数估计值

    float Gyro[3];  // 角速度
    float Accel[3]; // 加速度
    float MotionAccel_b[3]; // 机体坐标加速度
    float MotionAccel_n[3]; // 绝对系加速度

    float AccelLPF; // 加速度低通滤波系数

    // 加速度在绝对系的向量表示
    float xn[3];
    float yn[3];
    float zn[3];

    float atanxz;
    float atanyz;

    // 位姿
    float Roll;
    float Pitch;
    float Yaw;
    float YawTotalAngle;
} INS_data_t;

typedef struct
{
    INS_data_t ins;
    IMU_Data_t imu;
} INS_t;

void INS_Init(INS_t *INS);
void INS_Update(INS_t *ins_ins);
void IMU_Temperature_Ctrl(const INS_t *ins_ins);

void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, const float *q);
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, const float *q);
#endif
