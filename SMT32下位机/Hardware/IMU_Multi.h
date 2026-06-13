#ifndef __IMU_MULTI_H
#define __IMU_MULTI_H

#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "misc.h"
#include "AHRS.h"

typedef struct
{
    int16_t ac1;
    int16_t ac2;
    int16_t ac3;
    uint16_t ac4;
    uint16_t ac5;
    uint16_t ac6;
    int16_t b1;
    int16_t b2;
    int16_t mb;
    int16_t mc;
    int16_t md;
    int32_t b5;
    uint8_t oss;
} IMU_BMP180CalibTypeDef;

typedef struct
{
    uint8_t online;
    uint8_t magOnline;
    uint8_t baroOnline;
    int16_t accRaw[3];
    int16_t gyroRaw[3];
    int16_t magRaw[3];
    float accG[3];
    float gyroDps[3];
    float magCal[3];
    float temperatureC;
    int32_t pressurePa;
    float altitudeM;
    float roll;
    float pitch;
    float yaw;
    IMU_BMP180CalibTypeDef bmpCalib;
    float magMin[3];
    float magMax[3];
    uint8_t bmpDivCounter;
    AHRS_StateTypeDef ahrs;
} IMU_PoseDataTypeDef;

void IMU_Multi_Init(void);
void IMU_Multi_Update(float dt);
const IMU_PoseDataTypeDef* IMU_Multi_Get(uint8_t index);

#endif
