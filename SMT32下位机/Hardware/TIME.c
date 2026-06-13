#include "TIME.h"
#include "myMOTOR.h"

/*
 * �ջ������������ڳ��������Ƽ�����˳��
 *
 * Step 0: ���ֿ������ڹ̶�
 * - CTRL_DT_SEC �������ѭ�� Delay_ms(10) һ�£��� 0.01s�������� Ki ʵ�������Ư�ơ�
 *
 * Step 1: �ȵ� Kp���̶� Ki=0, Kff=0��
 * - ��ʱ�޸ģ�CTRL_KI=0, CTRL_KFF=0��
 * - �ӽ�С CTRL_KP ��ʼ������ 20����ÿ������ 5~10����һ�ν�Ծ���ԣ��ֱ������Ƶ�ĳ�̶�λ�ã���
 * - �����֡���΢�񵴡���Ŀ�긽�����ز��� 2~3 ���ڣ�ʱ����¼�� Kp Ϊ Kp_crit��
 * - ���� Kp �������Լ 15%��Kp = 0.85 * Kp_crit��
 *
 * Step 2: �ټ� Ki���������
 * - �̶���һ���õ��� Kp������ Kff=0��
 * - �ӽ�С Ki ��ʼ������ 20����ÿ������ 10~20��
 * - Ŀ�꣺���������Ҿ���ӽ� 0�������ֵ�Ƶ�ڶ�/�ص���˵�� Ki ƫ��Ӧ���ˡ�
 *
 * Step 3: ���� Kff���������桰����С���
 * - �� Kp/Ki ���ȶ��󣬴� 0 ��ʼ�� Kff��ÿ�μ� 1~2����
 * - Ŀ�꣺������Ӧ���졢�ͺ��С����������������Kff ���ˡ�
 *
 * Step 4: ÿ�θĲ�������ִ�� ClosedLoop_Reset()
 * - �������״̬������ɻ���Ӱ���²����жϡ�
 */
#define CTRL_DT_SEC            (0.01f)    // ��������(s)��������ѭ��10msһ��
#define CTRL_PWM_LIMIT         (7200)     // PWM����޷�����TIM��װ��ֵͬ������
#define CTRL_KP                (55.0f)    // �������棬������Ӧ���������
#define CTRL_KI                (180.0f)   // �������棬����������̬���
#define CTRL_KFF               (12.0f)    // ǰ�����棬���������ٶ��롰����С�
#define CTRL_I_LIMIT           (18.0f)    // �������޷�����ֹ���ֹ�����

static float g_i1 = 0.0f;
static float g_i2 = 0.0f;
static float g_i3 = 0.0f;

/* Encoder total accumulation (for position / rod-length tracking) */
static int32_t g_encoder_total1 = 0L;
static int32_t g_encoder_total2 = 0L;
static int32_t g_encoder_total3 = 0L;
static int16_t g_encoder_vel1 = 0;
static int16_t g_encoder_vel2 = 0;
static int16_t g_encoder_vel3 = 0;

/*
 * ��ʱ����Դ����˵����
 * - TIM1 CH1/CH2/CH3 -> PA8/PA9/PA10����·PWM����TB6612
 * - TIM2 ������ģʽ -> PA0/PA1�����1����AB��
 * - TIM3 ������ģʽ -> PA6/PA7�����2����AB��
 * - TIM4 ������ģʽ -> PB6/PB7�����3����AB��
 */
//ʹ��TIM1�����·PWM��PA8/PA9/PA10������TB6612������PWM����
void TIM4_PWM_Init(u16 arr,u16 psc)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);	//ʹ�ܶ�ʱ��1ʱ��
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  //ʹ��GPIOAʱ��
	
	//��ʼ��TIM1��·PWM�������(CH1->PA8 CH2->PA9 CH3->PA10)
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //�����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIO
 
	//��ʼ��TIM1
	TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
	
	//��ʼ��TIM1 Channel1/2/3 PWMģʽ
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //ѡ��ʱ��ģʽ:TIM������ȵ���ģʽ1
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //�������:TIM����Ƚϼ��Ը�
	
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM1 OC1
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);
	
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);  //ʹ��TIM1��CCR1�ϵ�Ԥװ�ؼĴ���
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
	
	TIM_Cmd(TIM1, ENABLE);  //ʹ��TIM1
	TIM_CtrlPWMOutputs(TIM1, ENABLE); //�߼���ʱ������ʹ�������
}

static int limit_range(int value, int min_value, int max_value)
{
	if (value > max_value) return max_value;
	if (value < min_value) return min_value;
	return value;
}

void Encoder_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4, ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	// ������1��PA0/PA1 -> TIM2 CH1/CH2
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// ������2��PA6/PA7 -> TIM3 CH1/CH2
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// ������3��PB6/PB7 -> TIM4 CH1/CH2
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

	TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);

	TIM_SetCounter(TIM2, 0);
	TIM_SetCounter(TIM3, 0);
	TIM_SetCounter(TIM4, 0);

	TIM_Cmd(TIM2, ENABLE);
	TIM_Cmd(TIM3, ENABLE);
	TIM_Cmd(TIM4, ENABLE);
}

int16_t Encoder_Read1(void)
{
	int16_t cnt = (int16_t)TIM_GetCounter(TIM2);
	TIM_SetCounter(TIM2, 0);
	g_encoder_total1 += (int32_t)cnt;
	g_encoder_vel1 = cnt;
	return cnt;
}

int16_t Encoder_Read2(void)
{
	int16_t cnt = (int16_t)TIM_GetCounter(TIM3);
	TIM_SetCounter(TIM3, 0);
	g_encoder_total2 += (int32_t)cnt;
	g_encoder_vel2 = cnt;
	return cnt;
}

int16_t Encoder_Read3(void)
{
	int16_t cnt = (int16_t)TIM_GetCounter(TIM4);
	TIM_SetCounter(TIM4, 0);
	g_encoder_total3 += (int32_t)cnt;
	g_encoder_vel3 = cnt;
	return cnt;
}

void Encoder_ResetTotals(void)
{
	TIM_SetCounter(TIM2, 0);
	TIM_SetCounter(TIM3, 0);
	TIM_SetCounter(TIM4, 0);
	g_encoder_total1 = 0L;
	g_encoder_total2 = 0L;
	g_encoder_total3 = 0L;
	g_encoder_vel1 = 0;
	g_encoder_vel2 = 0;
	g_encoder_vel3 = 0;
}

void Encoder_GetTotals(int32_t *c1, int32_t *c2, int32_t *c3)
{
	if (c1) *c1 = g_encoder_total1;
	if (c2) *c2 = g_encoder_total2;
	if (c3) *c3 = g_encoder_total3;
}

void Encoder_GetVelocity(int16_t *v1, int16_t *v2, int16_t *v3)
{
	if (v1) *v1 = g_encoder_vel1;
	if (v2) *v2 = g_encoder_vel2;
	if (v3) *v3 = g_encoder_vel3;
}

void ClosedLoop_Reset(void)
{
	TIM_SetCounter(TIM2, 0);
	TIM_SetCounter(TIM3, 0);
	TIM_SetCounter(TIM4, 0);
	g_encoder_total1 = 0L;
	g_encoder_total2 = 0L;
	g_encoder_total3 = 0L;
	g_encoder_vel1 = 0;
	g_encoder_vel2 = 0;
	g_encoder_vel3 = 0;
	g_i1 = 0.0f;
	g_i2 = 0.0f;
	g_i3 = 0.0f;
}

void ClosedLoop_Update(int target1, int target2, int target3)
{
	/*
	 * ��ǰ�����ɣ�u = Kp*e + Ki*��e dt + Kff*target
	 * - e = target - feedback����λ��ÿ10ms������������
	 * - Kff*target �ṩ�ٶ�ǰ������С��������������λ�ͺ�
	 */
	int fb1 = Encoder_Read1();
	int fb2 = Encoder_Read2();
	int fb3 = Encoder_Read3();
	float e1 = (float)(target1 - fb1);
	float e2 = (float)(target2 - fb2);
	float e3 = (float)(target3 - fb3);
	float u1;
	float u2;
	float u3;
	int pwm1, pwm2, pwm3;

	if ((target1 == 0) && (target2 == 0) && (target3 == 0))
	{
		(void)Encoder_Read1();
		(void)Encoder_Read2();
		(void)Encoder_Read3();
		g_i1 = 0.0f;
		g_i2 = 0.0f;
		g_i3 = 0.0f;
		Set_Car_Speed(0, 0, 0);
		return;
	}

	/*
	 * �������ֿ����ͣ�������ѵ���/�����������������ͷ�������ʱ����ͣ���֡�
	 */
	u1 = CTRL_KP * e1 + CTRL_KI * g_i1 + CTRL_KFF * (float)target1;
	u2 = CTRL_KP * e2 + CTRL_KI * g_i2 + CTRL_KFF * (float)target2;
	u3 = CTRL_KP * e3 + CTRL_KI * g_i3 + CTRL_KFF * (float)target3;

	if (!((u1 >= CTRL_PWM_LIMIT && e1 > 0.0f) || (u1 <= -CTRL_PWM_LIMIT && e1 < 0.0f)))
	{
		g_i1 += e1 * CTRL_DT_SEC;
	}
	if (!((u2 >= CTRL_PWM_LIMIT && e2 > 0.0f) || (u2 <= -CTRL_PWM_LIMIT && e2 < 0.0f)))
	{
		g_i2 += e2 * CTRL_DT_SEC;
	}
	if (!((u3 >= CTRL_PWM_LIMIT && e3 > 0.0f) || (u3 <= -CTRL_PWM_LIMIT && e3 < 0.0f)))
	{
		g_i3 += e3 * CTRL_DT_SEC;
	}

	if (g_i1 > CTRL_I_LIMIT) g_i1 = CTRL_I_LIMIT;
	if (g_i1 < -CTRL_I_LIMIT) g_i1 = -CTRL_I_LIMIT;
	if (g_i2 > CTRL_I_LIMIT) g_i2 = CTRL_I_LIMIT;
	if (g_i2 < -CTRL_I_LIMIT) g_i2 = -CTRL_I_LIMIT;
	if (g_i3 > CTRL_I_LIMIT) g_i3 = CTRL_I_LIMIT;
	if (g_i3 < -CTRL_I_LIMIT) g_i3 = -CTRL_I_LIMIT;

	u1 = CTRL_KP * e1 + CTRL_KI * g_i1 + CTRL_KFF * (float)target1;
	u2 = CTRL_KP * e2 + CTRL_KI * g_i2 + CTRL_KFF * (float)target2;
	u3 = CTRL_KP * e3 + CTRL_KI * g_i3 + CTRL_KFF * (float)target3;

	pwm1 = (int)u1;
	pwm2 = (int)u2;
	pwm3 = (int)u3;

	pwm1 = limit_range(pwm1, -CTRL_PWM_LIMIT, CTRL_PWM_LIMIT);
	pwm2 = limit_range(pwm2, -CTRL_PWM_LIMIT, CTRL_PWM_LIMIT);
	pwm3 = limit_range(pwm3, -CTRL_PWM_LIMIT, CTRL_PWM_LIMIT);

	Set_Car_Speed(pwm1, pwm2, pwm3);
}

