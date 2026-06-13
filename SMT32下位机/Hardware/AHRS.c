#include "AHRS.h"
#include <math.h>

#define AHRS_PI 3.1415926535f

static float AHRS_Limit(float Value, float Min, float Max)
{
    if (Value < Min)
    {
        return Min;
    }
    if (Value > Max)
    {
        return Max;
    }
    return Value;
}

static float AHRS_InvSqrt(float X)
{
    if (X <= 0.0f)
    {
        return 0.0f;
    }
    return 1.0f / sqrtf(X);
}

static float AHRS_WrapPi(float Angle)
{
    while (Angle > AHRS_PI)
    {
        Angle -= 2.0f * AHRS_PI;
    }
    while (Angle < -AHRS_PI)
    {
        Angle += 2.0f * AHRS_PI;
    }
    return Angle;
}

/*
 * 姿态解算初始化：
 * 四元数初始化为单位四元数，Kp/Ki 采用上传工程中验证过的一组参数。
 */
void AHRS_Init(AHRS_StateTypeDef *State, float GyroWeight)
{
    float Kp;
    float Ki;
    (void)GyroWeight;

    State->roll = 0.0f;
    State->pitch = 0.0f;
    State->yaw = 0.0f;
    State->q0 = 1.0f;
    State->q1 = 0.0f;
    State->q2 = 0.0f;
    State->q3 = 0.0f;

    Kp = 1.0f;
    Ki = 0.53f;
    State->twoKp = 2.0f * Kp;
    State->twoKi = 2.0f * Ki;
    State->integralFBx = 0.0f;
    State->integralFBy = 0.0f;
    State->integralFBz = 0.0f;
}

/*
 * Mahony 姿态融合核心：
 * 输入陀螺仪(°/s)、加速度(g)、磁场向量，输出更新后的四元数与欧拉角。
 */
void AHRS_Update(AHRS_StateTypeDef *State,
    float GxDps, float GyDps, float GzDps,
    float AxG, float AyG, float AzG,
    float Mx, float My, float Mz,
    float Dt)
{
    float Gx;
    float Gy;
    float Gz;
    float Q0;
    float Q1;
    float Q2;
    float Q3;
    float Norm;
    float Hx;
    float Hy;
    float Bx;
    float Bz;
    float Vx;
    float Vy;
    float Vz;
    float Wx;
    float Wy;
    float Wz;
    float HalfEx;
    float HalfEy;
    float HalfEz;
    float HalfT;
    float Qa;
    float Qb;
    float Qc;

    if (Dt <= 0.0f)
    {
        return;
    }

    Gx = GxDps * (AHRS_PI / 180.0f);
    Gy = GyDps * (AHRS_PI / 180.0f);
    Gz = GzDps * (AHRS_PI / 180.0f);

    Q0 = State->q0;
    Q1 = State->q1;
    Q2 = State->q2;
    Q3 = State->q3;

    Norm = AHRS_InvSqrt(AxG * AxG + AyG * AyG + AzG * AzG);
    if (Norm > 0.0f)
    {
        AxG *= Norm;
        AyG *= Norm;
        AzG *= Norm;
    }

    Norm = AHRS_InvSqrt(Mx * Mx + My * My + Mz * Mz);
    if (Norm > 0.0f)
    {
        Mx *= Norm;
        My *= Norm;
        Mz *= Norm;
    }

    Hx = 2.0f * (Mx * (0.5f - Q2 * Q2 - Q3 * Q3) + My * (Q1 * Q2 - Q0 * Q3) + Mz * (Q1 * Q3 + Q0 * Q2));
    Hy = 2.0f * (Mx * (Q1 * Q2 + Q0 * Q3) + My * (0.5f - Q1 * Q1 - Q3 * Q3) + Mz * (Q2 * Q3 - Q0 * Q1));
    Bx = sqrtf(Hx * Hx + Hy * Hy);
    Bz = 2.0f * (Mx * (Q1 * Q3 - Q0 * Q2) + My * (Q2 * Q3 + Q0 * Q1) + Mz * (0.5f - Q1 * Q1 - Q2 * Q2));

    Vx = 2.0f * (Q1 * Q3 - Q0 * Q2);
    Vy = 2.0f * (Q0 * Q1 + Q2 * Q3);
    Vz = Q0 * Q0 - Q1 * Q1 - Q2 * Q2 + Q3 * Q3;
    Wx = 2.0f * Bx * (0.5f - Q2 * Q2 - Q3 * Q3) + 2.0f * Bz * (Q1 * Q3 - Q0 * Q2);
    Wy = 2.0f * Bx * (Q1 * Q2 - Q0 * Q3) + 2.0f * Bz * (Q0 * Q1 + Q2 * Q3);
    Wz = 2.0f * Bx * (Q0 * Q2 + Q1 * Q3) + 2.0f * Bz * (0.5f - Q1 * Q1 - Q2 * Q2);

    HalfEx = (AyG * Vz - AzG * Vy) + (My * Wz - Mz * Wy);
    HalfEy = (AzG * Vx - AxG * Vz) + (Mz * Wx - Mx * Wz);
    HalfEz = (AxG * Vy - AyG * Vx) + (Mx * Wy - My * Wx);

    if ((State->twoKi > 0.0f) && ((fabsf(HalfEx) + fabsf(HalfEy) + fabsf(HalfEz)) > 1e-7f))
    {
        State->integralFBx += State->twoKi * HalfEx * Dt;
        State->integralFBy += State->twoKi * HalfEy * Dt;
        State->integralFBz += State->twoKi * HalfEz * Dt;
        Gx += State->integralFBx;
        Gy += State->integralFBy;
        Gz += State->integralFBz;
    }
    else
    {
        State->integralFBx = 0.0f;
        State->integralFBy = 0.0f;
        State->integralFBz = 0.0f;
    }

    Gx += State->twoKp * HalfEx;
    Gy += State->twoKp * HalfEy;
    Gz += State->twoKp * HalfEz;

    HalfT = 0.5f * Dt;
    Qa = Q0;
    Qb = Q1;
    Qc = Q2;
    Q0 += (-Qb * Gx - Qc * Gy - Q3 * Gz) * HalfT;
    Q1 += (Qa * Gx + Qc * Gz - Q3 * Gy) * HalfT;
    Q2 += (Qa * Gy - Qb * Gz + Q3 * Gx) * HalfT;
    Q3 += (Qa * Gz + Qb * Gy - Qc * Gx) * HalfT;

    Norm = AHRS_InvSqrt(Q0 * Q0 + Q1 * Q1 + Q2 * Q2 + Q3 * Q3);
    if (Norm > 0.0f)
    {
        Q0 *= Norm;
        Q1 *= Norm;
        Q2 *= Norm;
        Q3 *= Norm;
    }

    State->q0 = Q0;
    State->q1 = Q1;
    State->q2 = Q2;
    State->q3 = Q3;

    State->roll = atan2f(2.0f * (Q0 * Q1 + Q2 * Q3), 1.0f - 2.0f * (Q1 * Q1 + Q2 * Q2));
    State->pitch = asinf(AHRS_Limit(2.0f * (Q0 * Q2 - Q3 * Q1), -1.0f, 1.0f));
    State->yaw = atan2f(2.0f * (Q0 * Q3 + Q1 * Q2), 1.0f - 2.0f * (Q2 * Q2 + Q3 * Q3));
    State->yaw = AHRS_WrapPi(State->yaw);
}

void AHRS_GetEulerDeg(const AHRS_StateTypeDef *State, float *RollDeg, float *PitchDeg, float *YawDeg)
{
    *RollDeg = State->roll * 180.0f / AHRS_PI;
    *PitchDeg = State->pitch * 180.0f / AHRS_PI;
    *YawDeg = State->yaw * 180.0f / AHRS_PI;
}
