/*****************************************************************************
 ** File			: include.h
 ** Author			: 
 ** Date			: 	
 ** Function		: 公共头文件
/
*****************************************************************************/
#ifndef _INCLUDE_H_
#define _INCLUDE_H_

//常用的类型定义
typedef unsigned char	bool, BOOL;

typedef unsigned char	u8, U8, uint8, UINT8, BYTE;
typedef signed char		s8, S8, int8, INT8;

typedef unsigned short	u16, U16, uint16, UINT16, WORD;
typedef signed short	s16, S16, int16, INT16;

typedef unsigned long	 U32, uint32, UINT32, DWORD;
typedef signed long		 S32, int32, INT32;


typedef unsigned short	string;

/* 添加死区处理，避免摇杆微小漂移 */
#define JOYSTICK_DEADZONE 10

/* 摇杆中心值定义（PS2手柄的标准中心值） */
#define CENTER_X 127         // X轴的中心值（摇杆左右移动）
#define CENTER_Y 128         // Y轴的中心值（摇杆上下移动）

/* 限制速度范围（PWM占空比）*/
#define MAX_SPEED 7200    // 根据PWM分辨率调整
#define MIN_SPEED -7200   // 负值表示反转	

//常用的宏定义
#define BIT(n) (1<<(n))

#define     BYTE0(n)            ((unsigned char)((unsigned short)(n)))
#define     BYTE1(n)            ((unsigned char)(((unsigned short)(n))>>8))
#define     BYTE2(n)            ((unsigned char)(((unsigned short)(((unsigned long)(n))>>8))>>8))
#define     BYTE3(n)            ((unsigned char)(((unsigned short)(((unsigned long)(n))>>16))>>8))

#define TRUE   1
#define FALSE  0
#define NULL   0


#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2))
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr))
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum))
//IO口地址映射
#define GPIOA_ODR_Addr    (GPIOA_BASE+12) //0x4001080C 
#define GPIOB_ODR_Addr    (GPIOB_BASE+12) //0x40010C0C 
#define GPIOC_ODR_Addr    (GPIOC_BASE+12) //0x4001100C 
#define GPIOD_ODR_Addr    (GPIOD_BASE+12) //0x4001140C 
#define GPIOE_ODR_Addr    (GPIOE_BASE+12) //0x4001180C 
#define GPIOF_ODR_Addr    (GPIOF_BASE+12) //0x40011A0C    
#define GPIOG_ODR_Addr    (GPIOG_BASE+12) //0x40011E0C    

#define GPIOA_IDR_Addr    (GPIOA_BASE+8) //0x40010808 
#define GPIOB_IDR_Addr    (GPIOB_BASE+8) //0x40010C08 
#define GPIOC_IDR_Addr    (GPIOC_BASE+8) //0x40011008 
#define GPIOD_IDR_Addr    (GPIOD_BASE+8) //0x40011408 
#define GPIOE_IDR_Addr    (GPIOE_BASE+8) //0x40011808 
#define GPIOF_IDR_Addr    (GPIOF_BASE+8) //0x40011A08 
#define GPIOG_IDR_Addr    (GPIOG_BASE+8) //0x40011E08 

//IO口操作,只对单一的IO口!
//确保n的值小于16!
#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  //输出 
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  //输入 

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //输出 
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //输入 

#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  //输出 
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  //输入 

#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  //输出 
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)  //输入 

#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  //输出 
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  //输入

#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  //输出 
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  //输入

#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  //输出 
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  //输入


#include "stm32f10x.h"
#include "misc.h"
#include "App.h"
#include "PS2GamePad.h"
#include <stdio.h>
#include <stdlib.h>

#include "Delay.h"
#include "solve.h"
#include "TIME.h"
#include "myMOTOR.h"
#include "AHRS.h"
#include "IMU_Multi.h"
#include "IMUSerial.h"

#include "Filter.h"

#endif

