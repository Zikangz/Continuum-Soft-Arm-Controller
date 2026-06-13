#include "stm32f10x.h"
#include "Delay.h"
#include "TIME.h"

/*
 * TB6612方向引脚接线：
 * - 电机1方向: PA2/PA3
 * - 电机2方向: PA4/PA5
 * - 电机3方向: PB0/PB1
 * PWM占空比由TIM1_CH1/CH2/CH3在TIME.c中输出。
 */
void MOTOR_GPIO_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
	/*GPIO初始化*/
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5;  //电机1/2方向控制：PA2,PA3,PA4,PA5
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;  //电机3方向控制：PB0,PB1
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}	
 
/*
	函数功能：控制三个电机的速度和正反转（适配TB6612）

	参数说明：
				PWM1、PWM2、PWM3
				范围：-7200到7200（与PWM周期匹配）
				参数正负不同表示电机不同的转向
*/
void Set_Car_Speed(int PWM1,int PWM2,int PWM3)
{
	if(PWM1>0)
	{
		TIM_SetCompare1(TIM1,PWM1);
		GPIO_SetBits(GPIOA, GPIO_Pin_2);
		GPIO_ResetBits(GPIOA, GPIO_Pin_3);
	}
	
	if(PWM2>0)
	{
		TIM_SetCompare2(TIM1,PWM2);
		GPIO_SetBits(GPIOA, GPIO_Pin_4);
		GPIO_ResetBits(GPIOA, GPIO_Pin_5);
	}
	
	if(PWM3>0)
	{
		TIM_SetCompare3(TIM1,PWM3);
		GPIO_SetBits(GPIOB, GPIO_Pin_0);
		GPIO_ResetBits(GPIOB, GPIO_Pin_1);
	}
	
	if(PWM1<0)
	{
		TIM_SetCompare1(TIM1,-PWM1);
		GPIO_SetBits(GPIOA, GPIO_Pin_3);
		GPIO_ResetBits(GPIOA, GPIO_Pin_2);
	}
	
	if(PWM2<0)
	{
		TIM_SetCompare2(TIM1,-PWM2);
		GPIO_SetBits(GPIOA, GPIO_Pin_5);
		GPIO_ResetBits(GPIOA, GPIO_Pin_4);
	}
	
	if(PWM3<0)
	{
		TIM_SetCompare3(TIM1,-PWM3);
		GPIO_SetBits(GPIOB, GPIO_Pin_1);
		GPIO_ResetBits(GPIOB, GPIO_Pin_0);
	}
	
	if(PWM1==0)
	{
		TIM_SetCompare1(TIM1,0);
		GPIO_ResetBits(GPIOA, GPIO_Pin_2|GPIO_Pin_3);
	}
	
	if(PWM2==0)
	{
		TIM_SetCompare2(TIM1,0);
		GPIO_ResetBits(GPIOA, GPIO_Pin_4|GPIO_Pin_5);
	}
	
	if(PWM3==0)
	{
		TIM_SetCompare3(TIM1,0);
		GPIO_ResetBits(GPIOB, GPIO_Pin_0|GPIO_Pin_1);
	}
	
}
