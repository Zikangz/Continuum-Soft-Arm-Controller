#ifndef __IMUSERIAL_H
#define __IMUSERIAL_H

#include "stm32f10x.h"

void IMUSerial_Init(uint32_t BaudRate);
void IMUSerial_SendByte(uint8_t Byte);
void IMUSerial_SendString(const char *String);
void IMUSerial_Printf(const char *format, ...);
void IMUSerial_OnRxByte(uint8_t Byte);
void IMUSerial_PollCommand(void);
void IMUSerial_Tick10ms(void);
uint8_t IMUSerial_GetHostAxes(int8_t *feed, int8_t *x, int8_t *y);
char IMUSerial_ConsumeHostPattern(void);
int8_t IMUSerial_ConsumeHostEnable(void);
uint8_t IMUSerial_ConsumeHostEstop(void);
int16_t IMUSerial_ConsumeHostGripper(void);
uint8_t IMUSerial_ConsumeHostHome(void);
uint8_t IMUSerial_ConsumeHostStatusRequest(void);
uint8_t IMUSerial_ConsumeHostRodTarget(float *l1_mm, float *l2_mm, float *l3_mm);

#endif
