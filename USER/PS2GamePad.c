/*****************************************************************************
 ** File			: ApiPS2GamePad.c
 ** Author			: 
 ** Date			: 			
 ** Function		: PS2游戏手柄接收器驱动
 ** Wiring        : DAT->PB12, CMD->PB13, CS->PB14, CLK->PB15
*****************************************************************************/
#include "include.h"

u16 Handkey;  //当前手柄按键状态（16位，每一位代表一个按键）
u8 Comd[2]={0x01,0x42};	//SPI命令：0x01=开始，0x42=请求数据
u8 Data[9]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; // 存储从手柄读取的9字节数据包 
// Data[0]: 未使用 // Data[1]: 设备ID和模式 // Data[2]: 右摇杆X轴 (RX)
// Data[3]: 右摇杆Y轴 (RY) // Data[4]: 左摇杆X轴 (LX) // Data[5]: 左摇杆Y轴 (LY)
// Data[6]: 按键数据低字节 // Data[7]: 按键数据高字节 // Data[8]: 特殊按键
u16 MASK[]={  // 按键索引到按键位掩码的映射表
    PSB_SELECT,
    PSB_L3,
    PSB_R3 ,
    PSB_START,
    PSB_PAD_UP,
    PSB_PAD_RIGHT,
    PSB_PAD_DOWN,
    PSB_PAD_LEFT,
    PSB_L2,
    PSB_R2,
    PSB_L1,
    PSB_R1 ,
    PSB_TRIANGLE,
    PSB_CIRCLE,
    PSB_CROSS,
    PSB_SQUARE
	};	//按键值与按键明


/******* 在文件开头定义滤波器变量 *******/
static MovingAverageFilter filter_lx;  // 左摇杆X轴滤波器
static MovingAverageFilter filter_ly;  // 左摇杆Y轴滤波器
static MovingAverageFilter filter_rx;  // 右摇杆X轴滤波器
static MovingAverageFilter filter_ry;  // 右摇杆Y轴滤波器

u8 InitPS2(void)
{
	u8 mode;
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	//PB12(DAT), PB13(CMD), PB14(CS), PB15(CLK)
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;  //DAT (数据输入)
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; // 浮空输入，读取手柄数据
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14| GPIO_Pin_15;  //CMD (命令输出)；CS/SEL (片选)
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 	  // 推挽输出，控制手柄
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

    /******* 初始化4个移动平均滤波器，窗口大小设为5 *******/
    MovingAverage_Init(&filter_lx, 5);  // 初始化左摇杆X轴滤波器
    MovingAverage_Init(&filter_ly, 5);  // 初始化左摇杆Y轴滤波器
    MovingAverage_Init(&filter_rx, 5);  // 初始化右摇杆X轴滤波器
    MovingAverage_Init(&filter_ry, 5);  // 初始化右摇杆Y轴滤波器
	
	mode = PS2_SetInit();		 //配配置初始化,配置“红绿灯模式”，并选择是否可以修改		
	return mode;
}

/******* 新增一个函数：获取滤波后的摇杆值 *******/
uint8_t FilteredValue(uint8_t axis_id, uint8_t raw_value)
{
    /* axis_id: 0=左X, 1=左Y, 2=右X, 3=右Y */
    switch(axis_id)  // 根据轴ID选择对应的滤波器
    {
        case 0:  // 左摇杆X轴
            return MovingAverage_Update(&filter_lx, raw_value);
            
        case 1:  // 左摇杆Y轴
            return MovingAverage_Update(&filter_ly, raw_value);
            
        case 2:  // 右摇杆X轴
            return MovingAverage_Update(&filter_rx, raw_value);
            
        case 3:  // 右摇杆Y轴
            return MovingAverage_Update(&filter_ry, raw_value);
            
        default:  // 如果轴ID不对
            return raw_value;  // 直接返回原始值
    }
}

//向手柄发送命令
void PS2_Cmd(u8 CMD)
{
	volatile u16 ref=0x01;
	Data[1] = 0;
	for(ref=0x01;ref<0x0100;ref<<=1)
	{
		if(ref&CMD)
		{
			DO_H;                   //输出一位控制位
		}
		else DO_L;

		Delay_us(10);
		CLK_L;
		Delay_us(40);
		CLK_H;
		if(DI)
			Data[1] = ref|Data[1];
		Delay_us(10);
	}
}
//判断是否为红灯模式
//返回值；0，红灯模式
//		  其他，其他模式
u8 PS2_RedLight(void)
{
	CS_L;
	PS2_Cmd(Comd[0]);  //开始命令
	PS2_Cmd(Comd[1]);  //请求数据
	CS_H;
	if( Data[1] == 0X73)   return 0 ;
	else return 1;

}
//读取手柄数据，核心通信函数
void PS2_ReadData(void)
{
	volatile u8 byte;
	volatile u16 ref;
 
	CS_L;
	Delay_us(10);
	PS2_Cmd(Comd[0]);  //开始命令
	PS2_Cmd(Comd[1]);  //请求数据
	for(byte=2;byte<9;byte++)   // 循环读取7个数据字节 (Data[2]~Data[8])
	{
		for(ref=0x01;ref<0x100;ref<<=1)
		{

			CLK_L;
				Delay_us(50);
			CLK_H;
		      if(DI)
		      {
				Data[byte] = ref|Data[byte];
			  }
				Delay_us(45);
		     
		}
			Delay_us(10);
	}
	CS_H;
}

//对读出来的PS2的数据进行处理      只处理了按键部分         默认数据是红灯模式  只有一个按键按下时
//按下为0， 未按下为1

u16 LastHandkey = 0xFFFF;
u8 PS2_DataKey()  // 按键扫描函数
{
	u8 index;

	PS2_ClearData();  // 清空数据缓冲区
	PS2_ReadData();  // 读取最新手柄数据
	LastHandkey = Handkey;  // 保存上次按键状态（用于检测变化）
	Handkey=(Data[4]<<8)|Data[3];   // 组合按键数据：Data[4]为高字节，Data[3]为低字节
	for(index=0;index<16;index++)  //扫描16个按键  按下为0， 未按下为1 
	{	    
		if((Handkey&(1<<(MASK[index]-1)))==0)
		return index+1;   // 返回按键索引（1-16）
	}
	return 0;          //没有任何按键按下
}

bool PS2_NewButtonState( u16 button )
{
  button = 0x0001u << ( button - 1 );  //输入的button的值是 该按键在数据中所在bit的值+1， 例如 PSB_SELECT 宏的值 是 1， 在数据中的位是0位， 如此类推，
  return ( ( ( LastHandkey ^ Handkey ) & button ) > 0 );  //将上次的按键数据和这次的按键数据进行异或运算，结果就是两次不同的部分会是1，就得到了状态发生了变化的按键
	                                                    //然后在与我们想要检测的按键进行与运算，如果这个按键发生了变化，那么结果就是1， 大于0，所以返回就是true
}

bool PS2_Button( u16 button )
{
  button = 0x0001u << ( button - 1 );  //输入的button的值是 该按键在数据中所在bit的值+1， 例如 PSB_SELECT 宏的值 是 1， 在数据中的位是0位， 如此类推，
  return ( ( (~Handkey) & button ) > 0 );  //按键按下则对应位为0，没按下为1， 将按键数据取反之后，就变成了按键为1，没按下为0
	                                         //再与我们想要检测的按键做与运算，若这个按键被按下，对应位就是1，没按下就是0，返回与0比较的结果，
}

bool PS2_ButtonPressed( u16 button )
{
  return (PS2_NewButtonState( button ) && PS2_Button( button ));  //按键被按住，并且这个是按键的一个新的状态，那么就是按键刚被按下
}

bool PS2_ButtonReleased( u16 button )
{
  return ( PS2_NewButtonState( button ) && !PS2_Button( button )); //按键没被按住，并且这个是按键的一个新的状态，那么就是按键刚被松开
}

//得到一个摇杆的模拟量	 范围0~256
u8 PS2_AnologData(u8 button)
{
	uint8_t raw_value;
    uint8_t filtered_value;
	
	/* 第一步：获取原始值 */
	raw_value = Data[button];// 直接返回对应数据字节
    // button参数应为以下宏之一：
    // PSS_LX (5): 左摇杆X轴，0-255，128为中心
    // PSS_LY (6): 左摇杆Y轴，0-255，128为中心  
    // PSS_RX (7): 右摇杆X轴，0-255，128为中心
    // PSS_RY (8): 右摇杆Y轴，0-255，128为中心
	
	/* 第二步：根据button参数选择对应的滤波器 */
    switch(button)
    {
        case PSS_LX:  // 左摇杆X轴（Data[4]）
            filtered_value = FilteredValue(0, raw_value);  // axis_id=0
            break;
            
        case PSS_LY:  // 左摇杆Y轴（Data[5]）
            filtered_value = FilteredValue(1, raw_value);  // axis_id=1
            break;
            
        case PSS_RX:  // 右摇杆X轴（Data[2]）
            filtered_value = FilteredValue(2, raw_value);  // axis_id=2
            break;
            
        case PSS_RY:  // 右摇杆Y轴（Data[3]）
            filtered_value = FilteredValue(3, raw_value);  // axis_id=3
            break;
            
        default:  // 如果不是摇杆数据，直接返回原始值
            return raw_value;	
	}
    
    /* 第三步：返回滤波后的值 */
    return filtered_value;
}

//清除数据缓冲区
void PS2_ClearData()
{
	u8 a;
	for(a=0;a<9;a++)
		Data[a]=0x00;
}
/******************************************************
Function:    void PS2_Vibration(u8 motor1, u8 motor2)
Description: 手柄震动函数，
Calls:		 void PS2_Cmd(u8 CMD);
Input: motor1:右侧小震动电机 0x00关，其他开
	   motor2:左侧大震动电机 0x40~0xFF 电机开，值越大 震动越大
******************************************************/
void PS2_Vibration(u8 motor1, u8 motor2)
{
	CS_L;
	Delay_us(50);
    PS2_Cmd(0x01);  //开始命令
	PS2_Cmd(0x42);  //请求数据
	PS2_Cmd(0X00);
	PS2_Cmd(motor1);
	PS2_Cmd(motor2);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	CS_H;
	Delay_us(50);  
}
//short poll
void PS2_ShortPoll(void)
{
	CS_L;
	Delay_us(50);
	PS2_Cmd(0x01);  
	PS2_Cmd(0x42);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x00);
	PS2_Cmd(0x00);
	CS_H;
	Delay_us(50);	
}
//进入配置
void PS2_EnterConfing(void)
{
    CS_L;
	Delay_us(50);
	PS2_Cmd(0x01);  
	PS2_Cmd(0x43);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x01);
	PS2_Cmd(0x00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	CS_H;
	Delay_us(50);
}
//发送模式设置
void PS2_TurnOnAnalogMode(void)
{
	CS_L;
	Delay_us(50);
	PS2_Cmd(0x01);  
	PS2_Cmd(0x44);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x01); //analog=0x01;digital=0x00  软件设置发送模式
	PS2_Cmd(0x03); //Ox03锁存设置，即不可通过按键“MODE”设置模式。
				   //0xEE不锁存软件设置，可通过按键“MODE”设置模式。
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	CS_H;
	Delay_us(50);
}
//振动设置
void PS2_VibrationMode(void)
{
	CS_L;
	Delay_us(50);
	PS2_Cmd(0x01);  
	PS2_Cmd(0x4D);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x00);
	PS2_Cmd(0X01);
	CS_H;
	Delay_us(50);	
}
//完成并保存配置
void PS2_ExitConfing(void)
{
    CS_L;
	Delay_us(50);
	PS2_Cmd(0x01);  
	PS2_Cmd(0x43);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x00);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	CS_H;
	Delay_us(50);
}
//手柄配置初始化
u8 PS2_SetInit(void)
{
	Delay_ms(100);  //DelayMs(100);
	PS2_ShortPoll();
	PS2_ShortPoll();
	PS2_ShortPoll();
	PS2_EnterConfing();		//进入配置模式
	PS2_TurnOnAnalogMode();	//“红绿灯”配置模式，并选择是否保存
	PS2_VibrationMode();	//开启震动模式
	PS2_ExitConfing();		//完成并保存配置
	if (PS2_RedLight() == 0)
		return 0;
	else
		return 1;
}
