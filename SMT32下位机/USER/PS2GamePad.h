#ifndef _PS2_GAME_PAD_H_
#define _PS2_GAME_PAD_H_

/*
 * PS2手柄接收器接线与宏定义：
 * DAT->PB12, CMD->PB13, CS->PB14, CLK->PB15
 */

/* --- 输入引脚定义 --- */
#define DI   PBin(12)           //DAT输入 -> PB12

#define DO_H PBout(13)=1        //CMD命令位高 -> PB13
#define DO_L PBout(13)=0        //CMD命令位低

#define CS_H PBout(14)=1       //CS片选拉高 -> PB14
#define CS_L PBout(14)=0       //CS片选拉低

#define CLK_H PBout(15)=1      //CLK时钟拉高 -> PB15
#define CLK_L PBout(15)=0      //CLK时钟拉低


/* --- PS2按键常量定义（无需修改）--- */
#define PSB_SELECT      1
#define PSB_L3          2
#define PSB_R3          3
#define PSB_START       4
#define PSB_PAD_UP      5
#define PSB_PAD_RIGHT   6
#define PSB_PAD_DOWN    7
#define PSB_PAD_LEFT    8
#define PSB_L2			9
#define PSB_R2          10
#define PSB_L1          11
#define PSB_R1          12
#define PSB_TRIANGLE    13	//三角形
#define PSB_CIRCLE      14	//圆圈
#define PSB_CROSS       15	//叉叉
#define PSB_SQUARE      16	//方框

//#define WHAMMY_BAR		8

/* --- 摇杆数据索引（无需修改）--- */
#define PSS_RX 5                // 右摇杆X轴数据索引 (对应Data[5])
#define PSS_RY 6                // 右摇杆Y轴数据索引 (对应Data[6])
#define PSS_LX 7                // 左摇杆X轴数据索引 (对应Data[7])
#define PSS_LY 8                // 左摇杆Y轴数据索引 (对应Data[8])

/* --- 函数声明（无需修改）--- */
u8 InitPS2(void);
u8 PS2_RedLight(void);//判断是否为红灯模式
void PS2_ReadData(void);
void PS2_Cmd(u8 CMD);		  //
u8 PS2_DataKey(void);		  //键值读取
u8 PS2_AnologData(u8 button); //得到一个摇杆的模拟量
void PS2_ClearData(void);	  //清除数据缓冲区
void PS2_Vibration(u8 motor1, u8 motor2);//振动设置motor1  0xFF开，其他关，motor2  0x40~0xFF

void PS2_EnterConfing(void);	 //进入配置
void PS2_TurnOnAnalogMode(void); //发送模拟量
void PS2_VibrationMode(void);    //振动设置
void PS2_ExitConfing(void);	     //完成配置
u8 PS2_SetInit(void);		     //配置初始化


bool PS2_NewButtonState( u16 button );
bool PS2_Button( u16 button );
bool PS2_ButtonPressed( u16 button );
bool PS2_ButtonReleased( u16 button );

#endif
