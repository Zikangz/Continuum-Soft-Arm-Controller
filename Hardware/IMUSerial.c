#include "IMUSerial.h"
#include "TIME.h"
#include "misc.h"
#include <stdio.h>
#include <stdarg.h>

#define RX_LINE_MAX          (48u)
#define HOST_TIMEOUT_TICKS   (20u)

static volatile char g_rx_line[RX_LINE_MAX];
static volatile uint8_t g_rx_index = 0u;
static volatile uint8_t g_rx_line_ready = 0u;

static int8_t g_host_feed = 0;
static int8_t g_host_x = 0;
static int8_t g_host_y = 0;
static uint8_t g_host_active_ticks = 0u;
static char g_host_pattern = 0;
static int8_t g_host_enable_request = -1;
static uint8_t g_host_estop_request = 0u;
static int16_t g_host_gripper_request = -1;
static uint8_t g_host_home_request = 0u;
static uint8_t g_host_status_request = 0u;

/* L protocol: rod target position (mm) */
static float g_host_rod_l1_mm = 0.0f;
static float g_host_rod_l2_mm = 0.0f;
static float g_host_rod_l3_mm = 0.0f;
static uint8_t g_host_rod_request = 0u;

/* Extern variables from App.c (runtime-tunable by K/M/Z commands) */
extern float g_rod_kp_pos;
extern float g_rod_tolerance_mm;
extern int   g_rod_speed_limit;
extern float g_mm_per_count_1;
extern float g_mm_per_count_2;
extern float g_mm_per_count_3;
extern float g_home_L1_mm;
extern float g_home_L2_mm;
extern float g_home_L3_mm;

/* E command: encoder query request flag */
static uint8_t g_host_encoder_query = 0u;

/* M command: new mm_per_count values */
static float g_host_new_mm1 = 0.0f;
static float g_host_new_mm2 = 0.0f;
static float g_host_new_mm3 = 0.0f;
static uint8_t g_host_mm_request = 0u;

static int clamp_int(int value, int min_value, int max_value)
{
    if (value > max_value) return max_value;
    if (value < min_value) return min_value;
    return value;
}

/*
 * 串口3初始化：
 * TX -> PB10, RX -> PB11，默认用于与Jetson通信发送位姿数据。
 * 报文格式：
 * IMU3,t,o1,r1,p1,y1,temp1,press1,alt1,o2,r2,p2,y2,temp2,press2,alt2,o3,r3,p3,y3,temp3,press3,alt3\r\n
 */
void IMUSerial_Init(uint32_t BaudRate)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = BaudRate;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_Init(USART3, &USART_InitStructure);

    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_Cmd(USART3, ENABLE);
}

void IMUSerial_SendByte(uint8_t Byte)
{
    USART_SendData(USART3, Byte);
    while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
}

void IMUSerial_SendString(const char *String)
{
    uint16_t i;
    for (i = 0; String[i] != '\0'; i++)
    {
        IMUSerial_SendByte((uint8_t)String[i]);
    }
}

#pragma import(__use_no_semihosting)
struct __FILE { int handle; };
FILE __stdout;

void _sys_exit(int x)
{
    x = x;
}

int fputc(int ch, FILE *f)
{
    f = f;
    IMUSerial_SendByte((uint8_t)ch);
    return ch;
}

void IMUSerial_Printf(const char *format, ...)
{
    char String[256];
    va_list arg;
    va_start(arg, format);
    vsnprintf(String, sizeof(String), format, arg);
    va_end(arg);
    IMUSerial_SendString(String);
}

void IMUSerial_OnRxByte(uint8_t Byte)
{
    if (Byte == '\r')
    {
        return;
    }

    if (Byte == '\n')
    {
        if (g_rx_index > 0u)
        {
            g_rx_line[g_rx_index] = '\0';
            g_rx_line_ready = 1u;
            g_rx_index = 0u;
        }
        return;
    }

    if (g_rx_line_ready != 0u)
    {
        return;
    }

    if (g_rx_index < (RX_LINE_MAX - 1u))
    {
        g_rx_line[g_rx_index++] = (char)Byte;
    }
    else
    {
        g_rx_index = 0u;
    }
}

static void host_stop(void)
{
    g_host_feed = 0;
    g_host_x = 0;
    g_host_y = 0;
    g_host_active_ticks = 0u;
    g_host_rod_request = 0u;
}

static void parse_host_line(const char *line)
{
    int feed;
    int x;
    int y;
    int value;
    char pattern;

    if ((line == 0) || (line[0] == '\0'))
    {
        return;
    }

    if ((line[0] == 'S') || (line[0] == 's'))
    {
        host_stop();
        return;
    }

    if ((line[0] == 'E') || (line[0] == 'e'))
    {
        host_stop();
        g_host_estop_request = 1u;
        return;
    }

    if ((line[0] == 'H') || (line[0] == 'h'))
    {
        host_stop();
        g_host_home_request = 1u;
        return;
    }

    if ((line[0] == 'Q') || (line[0] == 'q'))
    {
        g_host_status_request = 1u;
        return;
    }

    if (sscanf(line, "A,%d", &value) == 1)
    {
        g_host_enable_request = (value != 0) ? 1 : 0;
        if (g_host_enable_request == 0)
        {
            host_stop();
        }
        return;
    }

    if (sscanf(line, "G,%d", &value) == 1)
    {
        g_host_gripper_request = (int16_t)clamp_int(value, 0, 100);
        return;
    }

    if (sscanf(line, "G,%c", &pattern) == 1)
    {
        if ((pattern == 'O') || (pattern == 'o'))
        {
            g_host_gripper_request = 100;
        }
        else if ((pattern == 'C') || (pattern == 'c'))
        {
            g_host_gripper_request = 0;
        }
        else if ((pattern == 'S') || (pattern == 's'))
        {
            g_host_gripper_request = 50;
        }
        return;
    }

    if (sscanf(line, "J,%d,%d,%d", &feed, &x, &y) == 3)
    {
        g_host_feed = (int8_t)clamp_int(feed, -100, 100);
        g_host_x = (int8_t)clamp_int(x, -100, 100);
        g_host_y = (int8_t)clamp_int(y, -100, 100);
        g_host_active_ticks = HOST_TIMEOUT_TICKS;
        /* J overrides L rod position mode */
        g_host_rod_request = 0u;
        return;
    }

    /* L protocol: rod target position in mm */
    {
        float l1;
        float l2;
        float l3;
        if (sscanf(line, "L,%f,%f,%f", &l1, &l2, &l3) == 3)
        {
            g_host_rod_l1_mm = l1;
            g_host_rod_l2_mm = l2;
            g_host_rod_l3_mm = l3;
            g_host_rod_request = 1u;
            g_host_active_ticks = HOST_TIMEOUT_TICKS;
            return;
        }
    }

    /* R command: query encoder totals -> prints ENC,total1,total2,total3 */
    if ((line[0] == 'R') && (line[1] == '\0' || line[1] == ','))
    {
        int32_t c1 = 0L, c2 = 0L, c3 = 0L;
        Encoder_GetTotals(&c1, &c2, &c3);
        IMUSerial_Printf("ENC,%ld,%ld,%ld\r\n", (long)c1, (long)c2, (long)c3);
        return;
    }

    /* K command: runtime PID config -> K,kp,tolerance,speed_limit */
    if ((line[0] == 'K') && (line[1] == ','))
    {
        const char *p = line + 2;
        float kp = 0.0f;
        float tol = 0.0f;
        float lim_f = 0.0f;
        float frac;
        int n = 0;
        int lim;
        /* Manual float parse — robust on all libc variants */
        while (*p == ' ') p++;
        while (*p >= '0' && *p <= '9') { kp = kp * 10.0f + (float)(*p - '0'); p++; }
        if (*p == '.') { p++; frac = 0.1f; while (*p >= '0' && *p <= '9') { kp += frac * (float)(*p - '0'); frac *= 0.1f; p++; } }
        if (*p == ',') { p++; n++; }
        while (*p == ' ') p++;
        while (*p >= '0' && *p <= '9') { tol = tol * 10.0f + (float)(*p - '0'); p++; }
        if (*p == '.') { p++; frac = 0.1f; while (*p >= '0' && *p <= '9') { tol += frac * (float)(*p - '0'); frac *= 0.1f; p++; } }
        if (*p == ',') { p++; n++; }
        while (*p == ' ') p++;
        while (*p >= '0' && *p <= '9') { lim_f = lim_f * 10.0f + (float)(*p - '0'); p++; }
        if (*p == '.') { p++; frac = 0.1f; while (*p >= '0' && *p <= '9') { lim_f += frac * (float)(*p - '0'); frac *= 0.1f; p++; } }
        if (n >= 2)
        {
            lim = (int)lim_f;
            if (kp > 0.0f && kp <= 1000.0f)  g_rod_kp_pos = kp;
            if (tol > 0.0f && tol <= 50.0f)   g_rod_tolerance_mm = tol;
            if (lim > 0 && lim <= 10000)       g_rod_speed_limit = lim;
            IMUSerial_Printf("K_OK,%.1f,%.1f,%d\r\n",
                (double)g_rod_kp_pos, (double)g_rod_tolerance_mm, g_rod_speed_limit);
            return;
        }
    }

    /* M command: set mm_per_count -> M,c1,c2,c3 */
    if ((line[0] == 'M') && (line[1] == ','))
    {
        const char *p = line + 2;
        float mc1 = 0.0f, mc2 = 0.0f, mc3 = 0.0f;
        float frac;
        int n = 0;
        while (*p == ' ') p++;
        while (*p >= '0' && *p <= '9') { mc1 = mc1 * 10.0f + (float)(*p - '0'); p++; }
        if (*p == '.') { p++; frac = 0.1f; while (*p >= '0' && *p <= '9') { mc1 += frac * (float)(*p - '0'); frac *= 0.1f; p++; } }
        if (*p == ',') { p++; n++; }
        while (*p == ' ') p++;
        while (*p >= '0' && *p <= '9') { mc2 = mc2 * 10.0f + (float)(*p - '0'); p++; }
        if (*p == '.') { p++; frac = 0.1f; while (*p >= '0' && *p <= '9') { mc2 += frac * (float)(*p - '0'); frac *= 0.1f; p++; } }
        if (*p == ',') { p++; n++; }
        while (*p == ' ') p++;
        while (*p >= '0' && *p <= '9') { mc3 = mc3 * 10.0f + (float)(*p - '0'); p++; }
        if (*p == '.') { p++; frac = 0.1f; while (*p >= '0' && *p <= '9') { mc3 += frac * (float)(*p - '0'); frac *= 0.1f; p++; } }
        if (n >= 2)
        {
            if (mc1 > 0.0f && mc1 <= 1.0f) g_mm_per_count_1 = mc1;
            if (mc2 > 0.0f && mc2 <= 1.0f) g_mm_per_count_2 = mc2;
            if (mc3 > 0.0f && mc3 <= 1.0f) g_mm_per_count_3 = mc3;
            IMUSerial_Printf("M_OK,%.6f,%.6f,%.6f\r\n",
                (double)g_mm_per_count_1,
                (double)g_mm_per_count_2,
                (double)g_mm_per_count_3);
            return;
        }
    }

    /* Z command: set home offset -> Z,l1,l2,l3 (physical rod length at encoder=0) */
    if ((line[0] == 'Z') && (line[1] == ','))
    {
        const char *p = line + 2;
        float z1 = 0.0f, z2 = 0.0f, z3 = 0.0f;
        float frac;
        int n = 0;
        while (*p == ' ') p++;
        while (*p >= '0' && *p <= '9') { z1 = z1 * 10.0f + (float)(*p - '0'); p++; }
        if (*p == '.') { p++; frac = 0.1f; while (*p >= '0' && *p <= '9') { z1 += frac * (float)(*p - '0'); frac *= 0.1f; p++; } }
        if (*p == ',') { p++; n++; }
        while (*p == ' ') p++;
        while (*p >= '0' && *p <= '9') { z2 = z2 * 10.0f + (float)(*p - '0'); p++; }
        if (*p == '.') { p++; frac = 0.1f; while (*p >= '0' && *p <= '9') { z2 += frac * (float)(*p - '0'); frac *= 0.1f; p++; } }
        if (*p == ',') { p++; n++; }
        while (*p == ' ') p++;
        while (*p >= '0' && *p <= '9') { z3 = z3 * 10.0f + (float)(*p - '0'); p++; }
        if (*p == '.') { p++; frac = 0.1f; while (*p >= '0' && *p <= '9') { z3 += frac * (float)(*p - '0'); frac *= 0.1f; p++; } }
        if (n >= 2)
        {
            if (z1 >= 0.0f && z1 <= 500.0f) g_home_L1_mm = z1;
            if (z2 >= 0.0f && z2 <= 500.0f) g_home_L2_mm = z2;
            if (z3 >= 0.0f && z3 <= 500.0f) g_home_L3_mm = z3;
            IMUSerial_Printf("Z_OK,%.1f,%.1f,%.1f\r\n",
                (double)g_home_L1_mm, (double)g_home_L2_mm, (double)g_home_L3_mm);
            return;
        }
    }

    if (sscanf(line, "P,%c", &pattern) == 1)
    {
        g_host_pattern = pattern;
    }
}

void IMUSerial_PollCommand(void)
{
    char line[RX_LINE_MAX];
    uint8_t i;

    if (g_rx_line_ready == 0u)
    {
        return;
    }

    USART_ITConfig(USART3, USART_IT_RXNE, DISABLE);
    for (i = 0u; i < RX_LINE_MAX; i++)
    {
        line[i] = (char)g_rx_line[i];
        if (line[i] == '\0')
        {
            break;
        }
    }
    line[RX_LINE_MAX - 1u] = '\0';
    g_rx_line_ready = 0u;
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

    parse_host_line(line);
}

void IMUSerial_Tick10ms(void)
{
    if (g_host_active_ticks > 0u)
    {
        g_host_active_ticks--;
    }
}

uint8_t IMUSerial_GetHostAxes(int8_t *feed, int8_t *x, int8_t *y)
{
    if ((feed == 0) || (x == 0) || (y == 0))
    {
        return 0u;
    }

    if (g_host_active_ticks == 0u)
    {
        return 0u;
    }

    *feed = g_host_feed;
    *x = g_host_x;
    *y = g_host_y;
    return 1u;
}

char IMUSerial_ConsumeHostPattern(void)
{
    char pattern = g_host_pattern;
    g_host_pattern = 0;
    return pattern;
}

int8_t IMUSerial_ConsumeHostEnable(void)
{
    int8_t request = g_host_enable_request;
    g_host_enable_request = -1;
    return request;
}

uint8_t IMUSerial_ConsumeHostEstop(void)
{
    uint8_t request = g_host_estop_request;
    g_host_estop_request = 0u;
    return request;
}

int16_t IMUSerial_ConsumeHostGripper(void)
{
    int16_t request = g_host_gripper_request;
    g_host_gripper_request = -1;
    return request;
}

uint8_t IMUSerial_ConsumeHostHome(void)
{
    uint8_t request = g_host_home_request;
    g_host_home_request = 0u;
    return request;
}

uint8_t IMUSerial_ConsumeHostStatusRequest(void)
{
    uint8_t request = g_host_status_request;
    g_host_status_request = 0u;
    return request;
}

uint8_t IMUSerial_ConsumeHostRodTarget(float *l1_mm, float *l2_mm, float *l3_mm)
{
    if ((l1_mm == 0) || (l2_mm == 0) || (l3_mm == 0))
    {
        return 0u;
    }

    if (g_host_rod_request == 0u)
    {
        return 0u;
    }

    *l1_mm = g_host_rod_l1_mm;
    *l2_mm = g_host_rod_l2_mm;
    *l3_mm = g_host_rod_l3_mm;
    g_host_rod_request = 0u;
    return 1u;
}
