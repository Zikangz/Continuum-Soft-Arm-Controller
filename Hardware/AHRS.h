#ifndef __AHRS_H
#define __AHRS_H

#include "stm32f10x.h"

typedef struct
{
    float roll;
    float pitch;
    float yaw;
    float q0;
    float q1;
    float q2;
    float q3;
    float twoKp;
    float twoKi;
    float integralFBx;
    float integralFBy;
    float integralFBz;
} AHRS_StateTypeDef;

void AHRS_Init(AHRS_StateTypeDef *State, float GyroWeight);
void AHRS_Update(AHRS_StateTypeDef *State,
    float GxDps, float GyDps, float GzDps,
    float AxG, float AyG, float AzG,
    float Mx, float My, float Mz,
    float Dt);
void AHRS_GetEulerDeg(const AHRS_StateTypeDef *State, float *RollDeg, float *PitchDeg, float *YawDeg);

#endif
