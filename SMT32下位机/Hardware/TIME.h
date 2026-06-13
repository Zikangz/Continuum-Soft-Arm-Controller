#ifndef __TIME_H
#define __TIME_H	
#include "sys.h"

/* TIM1: PWM输出，TIM2/TIM3/TIM4: 三路编码器输入 */
// void TIM1_Getsample_Int(u16 arr,u16 psc);
void TIM4_PWM_Init(u16 arr,u16 psc);
void Encoder_Init(void);
int16_t Encoder_Read1(void);
int16_t Encoder_Read2(void);
int16_t Encoder_Read3(void);
void Encoder_ResetTotals(void);
void Encoder_GetTotals(int32_t *c1, int32_t *c2, int32_t *c3);
void Encoder_GetVelocity(int16_t *v1, int16_t *v2, int16_t *v3);
void ClosedLoop_Reset(void);
void ClosedLoop_Update(int target1, int target2, int target3);

#endif
