#ifndef _APP_H_
#define _APP_H_
#include "include.h"

extern u32 gSystemTickCount;	//系统从启动到现在的毫秒数

void InitDelay(u8 SYSCLK);
void DelayMs(u16 nms);
void DelayUs(u32 nus);

//void Usart1_Init(void);
//void InitTimer2(void);
//void USART_SendString(USART_TypeDef* USARTx,char *str);

void TaskRun(u8 ps2_ok, u8 L_data[], u8 R_data[]);
void HandleControl(u8 L_data[], u8 R_data[]);

#endif
