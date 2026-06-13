#include "IMU_Multi.h"
#include "Delay.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "misc.h"
#include <float.h>
#include <math.h>

/*
 * 三路独立软件I2C接线：
 * - IMU1: SCL->PB8,  SDA->PB9
 * - IMU2: SCL->PA11, SDA->PA12
 * - IMU3: SCL->PB4,  SDA->PB5
 * 说明：PB4默认受JTAG占用，初始化中通过关闭JTAG释放该引脚。
 */

#define IMU_COUNT                3
#define MPU6050_ADDR_8BIT        0xD0
#define HMC5883L_ADDR_8BIT       0x3C
#define BMP180_ADDR_8BIT         0xEE

#define MPU6050_SMPLRT_DIV       0x19
#define MPU6050_CONFIG           0x1A
#define MPU6050_GYRO_CONFIG      0x1B
#define MPU6050_ACCEL_CONFIG     0x1C
#define MPU6050_INT_PIN_CFG      0x37
#define MPU6050_PWR_MGMT_1       0x6B
#define MPU6050_PWR_MGMT_2       0x6C
#define MPU6050_WHO_AM_I         0x75
#define MPU6050_ACCEL_XOUT_H     0x3B

#define HMC5883L_CONFIG_A        0x00
#define HMC5883L_CONFIG_B        0x01
#define HMC5883L_MODE            0x02
#define HMC5883L_DATA_X_MSB      0x03
#define HMC5883L_ID_A            0x0A

#define BMP180_CALIB_BASE        0xAA
#define BMP180_CONTROL           0xF4
#define BMP180_OUT_MSB           0xF6
#define BMP180_CMD_TEMP          0x2E
#define BMP180_CMD_PRESSURE      0x34

#define BMP180_DIV               10

typedef struct
{
    GPIO_TypeDef *sclPort;
    uint16_t sclPin;
    GPIO_TypeDef *sdaPort;
    uint16_t sdaPin;
} SoftI2CBusTypeDef;

static const SoftI2CBusTypeDef g_buses[IMU_COUNT] = {
    {GPIOB, GPIO_Pin_8,  GPIOB, GPIO_Pin_9},   /* IMU1 */
    {GPIOA, GPIO_Pin_11, GPIOA, GPIO_Pin_12},  /* IMU2 */
    {GPIOB, GPIO_Pin_4,  GPIOB, GPIO_Pin_5}    /* IMU3 */
};

static IMU_PoseDataTypeDef g_imuData[IMU_COUNT];

static void I2C_Delay(void)
{
    Delay_us(4);
}

static void I2C_SCL_Write(uint8_t bus, uint8_t level)
{
    GPIO_WriteBit(g_buses[bus].sclPort, g_buses[bus].sclPin, (BitAction)level);
}

static void I2C_SDA_Write(uint8_t bus, uint8_t level)
{
    GPIO_WriteBit(g_buses[bus].sdaPort, g_buses[bus].sdaPin, (BitAction)level);
}

static uint8_t I2C_SDA_Read(uint8_t bus)
{
    return GPIO_ReadInputDataBit(g_buses[bus].sdaPort, g_buses[bus].sdaPin);
}

static void I2C_Start(uint8_t bus)
{
    I2C_SDA_Write(bus, 1);
    I2C_SCL_Write(bus, 1);
    I2C_Delay();
    I2C_SDA_Write(bus, 0);
    I2C_Delay();
    I2C_SCL_Write(bus, 0);
    I2C_Delay();
}

static void I2C_Stop(uint8_t bus)
{
    I2C_SDA_Write(bus, 0);
    I2C_Delay();
    I2C_SCL_Write(bus, 1);
    I2C_Delay();
    I2C_SDA_Write(bus, 1);
    I2C_Delay();
}

static uint8_t I2C_WriteByte(uint8_t bus, uint8_t data)
{
    uint8_t i;

    for (i = 0; i < 8; i++)
    {
        I2C_SDA_Write(bus, (data & 0x80) ? 1 : 0);
        I2C_Delay();
        I2C_SCL_Write(bus, 1);
        I2C_Delay();
        I2C_SCL_Write(bus, 0);
        I2C_Delay();
        data <<= 1;
    }

    I2C_SDA_Write(bus, 1);
    I2C_Delay();
    I2C_SCL_Write(bus, 1);
    I2C_Delay();
    i = I2C_SDA_Read(bus);
    I2C_SCL_Write(bus, 0);
    I2C_Delay();

    return (i == 0U);
}

static uint8_t I2C_ReadByte(uint8_t bus, uint8_t ack)
{
    uint8_t i;
    uint8_t data = 0;

    I2C_SDA_Write(bus, 1);
    for (i = 0; i < 8; i++)
    {
        data <<= 1;
        I2C_SCL_Write(bus, 1);
        I2C_Delay();
        if (I2C_SDA_Read(bus))
        {
            data |= 0x01;
        }
        I2C_SCL_Write(bus, 0);
        I2C_Delay();
    }

    I2C_SDA_Write(bus, ack ? 0 : 1);
    I2C_Delay();
    I2C_SCL_Write(bus, 1);
    I2C_Delay();
    I2C_SCL_Write(bus, 0);
    I2C_Delay();
    I2C_SDA_Write(bus, 1);

    return data;
}

static uint8_t I2C_WriteReg(uint8_t bus, uint8_t devAddr8, uint8_t reg, uint8_t data)
{
    I2C_Start(bus);
    if (!I2C_WriteByte(bus, devAddr8))
    {
        I2C_Stop(bus);
        return 0;
    }
    if (!I2C_WriteByte(bus, reg))
    {
        I2C_Stop(bus);
        return 0;
    }
    if (!I2C_WriteByte(bus, data))
    {
        I2C_Stop(bus);
        return 0;
    }
    I2C_Stop(bus);
    return 1;
}

static uint8_t I2C_ReadRegs(uint8_t bus, uint8_t devAddr8, uint8_t reg, uint8_t *buf, uint8_t len)
{
    uint8_t i;

    I2C_Start(bus);
    if (!I2C_WriteByte(bus, devAddr8))
    {
        I2C_Stop(bus);
        return 0;
    }
    if (!I2C_WriteByte(bus, reg))
    {
        I2C_Stop(bus);
        return 0;
    }

    I2C_Start(bus);
    if (!I2C_WriteByte(bus, devAddr8 | 0x01))
    {
        I2C_Stop(bus);
        return 0;
    }

    for (i = 0; i < len; i++)
    {
        buf[i] = I2C_ReadByte(bus, (i < (len - 1U)) ? 1U : 0U);
    }

    I2C_Stop(bus);
    return 1;
}

static uint8_t MPU_WriteReg(uint8_t bus, uint8_t reg, uint8_t data)
{
    return I2C_WriteReg(bus, MPU6050_ADDR_8BIT, reg, data);
}

static uint8_t MPU_ReadRegs(uint8_t bus, uint8_t reg, uint8_t *buf, uint8_t len)
{
    return I2C_ReadRegs(bus, MPU6050_ADDR_8BIT, reg, buf, len);
}

static uint8_t MPU_InitOne(uint8_t bus)
{
    uint8_t id;

    if (!MPU_WriteReg(bus, MPU6050_PWR_MGMT_1, 0x01)) return 0;
    if (!MPU_WriteReg(bus, MPU6050_PWR_MGMT_2, 0x00)) return 0;
    if (!MPU_WriteReg(bus, MPU6050_SMPLRT_DIV, 0x09)) return 0;
    if (!MPU_WriteReg(bus, MPU6050_CONFIG, 0x06)) return 0;
    if (!MPU_WriteReg(bus, MPU6050_GYRO_CONFIG, 0x10)) return 0;
    if (!MPU_WriteReg(bus, MPU6050_ACCEL_CONFIG, 0x08)) return 0;

    if (!MPU_ReadRegs(bus, MPU6050_WHO_AM_I, &id, 1)) return 0;
    return (id == 0x68U) ? 1U : 0U;
}

static uint8_t MPU_EnableBypass(uint8_t bus)
{
    /* INT_PIN_CFG: BYPASS_EN=1，让外部磁力计/气压计挂到同一I2C线上可直读 */
    return MPU_WriteReg(bus, MPU6050_INT_PIN_CFG, 0x02);
}

static uint8_t HMC_InitOne(uint8_t bus)
{
    uint8_t idA;
    if (!I2C_WriteReg(bus, HMC5883L_ADDR_8BIT, HMC5883L_CONFIG_A, 0x70)) return 0;
    if (!I2C_WriteReg(bus, HMC5883L_ADDR_8BIT, HMC5883L_CONFIG_B, 0x40)) return 0;
    if (!I2C_WriteReg(bus, HMC5883L_ADDR_8BIT, HMC5883L_MODE, 0x00)) return 0;
    if (!I2C_ReadRegs(bus, HMC5883L_ADDR_8BIT, HMC5883L_ID_A, &idA, 1)) return 0;
    return (idA == 'H') ? 1U : 0U;
}

static uint8_t HMC_ReadRawOne(uint8_t bus, IMU_PoseDataTypeDef *imu)
{
    uint8_t b[6];
    if (!I2C_ReadRegs(bus, HMC5883L_ADDR_8BIT, HMC5883L_DATA_X_MSB, b, 6))
    {
        return 0;
    }

    imu->magRaw[0] = (int16_t)((b[0] << 8) | b[1]);
    imu->magRaw[2] = (int16_t)((b[2] << 8) | b[3]);
    imu->magRaw[1] = (int16_t)((b[4] << 8) | b[5]);
    return 1;
}

static int16_t BMP_ReadS16(uint8_t bus, uint8_t reg)
{
    uint8_t b[2];
    if (!I2C_ReadRegs(bus, BMP180_ADDR_8BIT, reg, b, 2))
    {
        return 0;
    }
    return (int16_t)((b[0] << 8) | b[1]);
}

static uint16_t BMP_ReadU16(uint8_t bus, uint8_t reg)
{
    uint8_t b[2];
    if (!I2C_ReadRegs(bus, BMP180_ADDR_8BIT, reg, b, 2))
    {
        return 0;
    }
    return (uint16_t)((b[0] << 8) | b[1]);
}

static uint8_t BMP_ReadCalibration(uint8_t bus, IMU_BMP180CalibTypeDef *calib)
{
    calib->oss = 2;
    calib->ac1 = BMP_ReadS16(bus, BMP180_CALIB_BASE + 0);
    calib->ac2 = BMP_ReadS16(bus, BMP180_CALIB_BASE + 2);
    calib->ac3 = BMP_ReadS16(bus, BMP180_CALIB_BASE + 4);
    calib->ac4 = BMP_ReadU16(bus, BMP180_CALIB_BASE + 6);
    calib->ac5 = BMP_ReadU16(bus, BMP180_CALIB_BASE + 8);
    calib->ac6 = BMP_ReadU16(bus, BMP180_CALIB_BASE + 10);
    calib->b1 = BMP_ReadS16(bus, BMP180_CALIB_BASE + 12);
    calib->b2 = BMP_ReadS16(bus, BMP180_CALIB_BASE + 14);
    calib->mb = BMP_ReadS16(bus, BMP180_CALIB_BASE + 16);
    calib->mc = BMP_ReadS16(bus, BMP180_CALIB_BASE + 18);
    calib->md = BMP_ReadS16(bus, BMP180_CALIB_BASE + 20);

    if ((calib->ac1 == 0) || (calib->ac1 == -1))
    {
        return 0;
    }
    return 1;
}

static uint16_t BMP_ReadUT(uint8_t bus)
{
    uint8_t b[2];
    if (!I2C_WriteReg(bus, BMP180_ADDR_8BIT, BMP180_CONTROL, BMP180_CMD_TEMP))
    {
        return 0;
    }
    Delay_ms(5);
    if (!I2C_ReadRegs(bus, BMP180_ADDR_8BIT, BMP180_OUT_MSB, b, 2))
    {
        return 0;
    }
    return (uint16_t)((b[0] << 8) | b[1]);
}

static uint32_t BMP_ReadUP(uint8_t bus, uint8_t oss)
{
    uint8_t b[3];
    uint32_t up;

    if (!I2C_WriteReg(bus, BMP180_ADDR_8BIT, BMP180_CONTROL, (uint8_t)(BMP180_CMD_PRESSURE + (oss << 6))))
    {
        return 0;
    }
    Delay_ms((uint16_t)(2 + (3 << oss)));
    if (!I2C_ReadRegs(bus, BMP180_ADDR_8BIT, BMP180_OUT_MSB, b, 3))
    {
        return 0;
    }

    up = ((uint32_t)b[0] << 16) | ((uint32_t)b[1] << 8) | b[2];
    up >>= (8U - oss);
    return up;
}

static float BMP_GetTemperature(IMU_BMP180CalibTypeDef *calib, uint16_t ut)
{
    int32_t x1, x2, t;

    x1 = (((int32_t)ut - (int32_t)calib->ac6) * (int32_t)calib->ac5) >> 15;
    x2 = ((int32_t)calib->mc << 11) / (x1 + calib->md);
    calib->b5 = x1 + x2;
    t = (calib->b5 + 8) >> 4;
    return t / 10.0f;
}

static int32_t BMP_GetPressure(IMU_BMP180CalibTypeDef *calib, uint32_t up)
{
    int32_t x1, x2, x3, b3, b6, p;
    uint32_t b4, b7;

    b6 = calib->b5 - 4000;
    x1 = (calib->b2 * ((b6 * b6) >> 12)) >> 11;
    x2 = (calib->ac2 * b6) >> 11;
    x3 = x1 + x2;
    b3 = (((((int32_t)calib->ac1) * 4 + x3) << calib->oss) + 2) >> 2;

    x1 = (calib->ac3 * b6) >> 13;
    x2 = (calib->b1 * ((b6 * b6) >> 12)) >> 16;
    x3 = (x1 + x2 + 2) >> 2;
    b4 = (calib->ac4 * (uint32_t)(x3 + 32768)) >> 15;
    if (b4 == 0)
    {
        return 0;
    }

    b7 = (uint32_t)(up - b3) * (50000U >> calib->oss);
    if (b7 < 0x80000000U)
    {
        p = (int32_t)((b7 << 1) / b4);
    }
    else
    {
        p = (int32_t)((b7 / b4) << 1);
    }

    x1 = (p >> 8) * (p >> 8);
    x1 = (x1 * 3038) >> 16;
    x2 = (-77357 * p) >> 16;
    p += (x1 + x2 + 3791) >> 4;
    return p;
}

static float BMP_GetAltitude(float pressurePa)
{
    float ratio;
    if (pressurePa <= 100.0f)
    {
        pressurePa = 100.0f;
    }
    ratio = pressurePa / 101325.0f;
    return 44330.0f * (powf(ratio, 0.19029496f) - 1.0f);
}

static void Mag_CalibrateOne(IMU_PoseDataTypeDef *imu)
{
    float mx = (float)imu->magRaw[0];
    float my = (float)imu->magRaw[1];
    float mz = (float)imu->magRaw[2];
    float offX, offY, offZ;
    float dx, dy, dz;
    float avg;

    if (mx < imu->magMin[0]) imu->magMin[0] = mx;
    if (my < imu->magMin[1]) imu->magMin[1] = my;
    if (mz < imu->magMin[2]) imu->magMin[2] = mz;
    if (mx > imu->magMax[0]) imu->magMax[0] = mx;
    if (my > imu->magMax[1]) imu->magMax[1] = my;
    if (mz > imu->magMax[2]) imu->magMax[2] = mz;

    offX = 0.5f * (imu->magMax[0] + imu->magMin[0]);
    offY = 0.5f * (imu->magMax[1] + imu->magMin[1]);
    offZ = 0.5f * (imu->magMax[2] + imu->magMin[2]);

    dx = 0.5f * (imu->magMax[0] - imu->magMin[0]);
    dy = 0.5f * (imu->magMax[1] - imu->magMin[1]);
    dz = 0.5f * (imu->magMax[2] - imu->magMin[2]);
    if (dx < 1.0f) dx = 1.0f;
    if (dy < 1.0f) dy = 1.0f;
    if (dz < 1.0f) dz = 1.0f;

    avg = (dx + dy + dz) / 3.0f;
    imu->magCal[0] = (mx - offX) * (avg / dx);
    imu->magCal[1] = (my - offY) * (avg / dy);
    imu->magCal[2] = (mz - offZ) * (avg / dz);
}

static uint8_t MPU_ReadDataOne(uint8_t bus, IMU_PoseDataTypeDef *imu)
{
    uint8_t b[14];

    if (!MPU_ReadRegs(bus, MPU6050_ACCEL_XOUT_H, b, 14))
    {
        return 0;
    }

    imu->accRaw[0] = (int16_t)((b[0] << 8) | b[1]);
    imu->accRaw[1] = (int16_t)((b[2] << 8) | b[3]);
    imu->accRaw[2] = (int16_t)((b[4] << 8) | b[5]);

    imu->gyroRaw[0] = (int16_t)((b[8] << 8) | b[9]);
    imu->gyroRaw[1] = (int16_t)((b[10] << 8) | b[11]);
    imu->gyroRaw[2] = (int16_t)((b[12] << 8) | b[13]);

    imu->accG[0] = (float)imu->accRaw[0] / 8192.0f;
    imu->accG[1] = (float)imu->accRaw[1] / 8192.0f;
    imu->accG[2] = (float)imu->accRaw[2] / 8192.0f;

    imu->gyroDps[0] = (float)imu->gyroRaw[0] / 32.8f;
    imu->gyroDps[1] = (float)imu->gyroRaw[1] / 32.8f;
    imu->gyroDps[2] = (float)imu->gyroRaw[2] / 32.8f;

    return 1;
}

void IMU_Multi_Init(void)
{
    uint8_t i;
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

    /* 释放 PB4 给 IMU3_SCL 使用，保留 SWD 调试 */
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    /* IMU1: PB8/PB9 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* IMU2: PA11/PA12 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* IMU3: PB4/PB5 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    for (i = 0; i < IMU_COUNT; i++)
    {
        I2C_SCL_Write(i, 1);
        I2C_SDA_Write(i, 1);
        g_imuData[i].online = 0;
        g_imuData[i].magOnline = 0;
        g_imuData[i].baroOnline = 0;
        g_imuData[i].temperatureC = 0.0f;
        g_imuData[i].pressurePa = 101325;
        g_imuData[i].altitudeM = 0.0f;
        g_imuData[i].bmpDivCounter = 0;
        g_imuData[i].magMin[0] = FLT_MAX;
        g_imuData[i].magMin[1] = FLT_MAX;
        g_imuData[i].magMin[2] = FLT_MAX;
        g_imuData[i].magMax[0] = -FLT_MAX;
        g_imuData[i].magMax[1] = -FLT_MAX;
        g_imuData[i].magMax[2] = -FLT_MAX;
        g_imuData[i].magCal[0] = 1.0f;
        g_imuData[i].magCal[1] = 0.0f;
        g_imuData[i].magCal[2] = 0.0f;
        AHRS_Init(&g_imuData[i].ahrs, 0.98f);
    }

    Delay_ms(50);

    for (i = 0; i < IMU_COUNT; i++)
    {
        g_imuData[i].online = MPU_InitOne(i);
        if (g_imuData[i].online)
        {
            MPU_EnableBypass(i);
            g_imuData[i].magOnline = HMC_InitOne(i);
            g_imuData[i].baroOnline = BMP_ReadCalibration(i, &g_imuData[i].bmpCalib);
        }
    }
}

void IMU_Multi_Update(float dt)
{
    uint8_t i;

    for (i = 0; i < IMU_COUNT; i++)
    {
        if (!g_imuData[i].online)
        {
            continue;
        }

        if (!MPU_ReadDataOne(i, &g_imuData[i]))
        {
            continue;
        }

        if (g_imuData[i].magOnline && HMC_ReadRawOne(i, &g_imuData[i]))
        {
            Mag_CalibrateOne(&g_imuData[i]);
        }

        if (g_imuData[i].baroOnline)
        {
            if (g_imuData[i].bmpDivCounter == 0U)
            {
                uint16_t ut = BMP_ReadUT(i);
                uint32_t up = BMP_ReadUP(i, g_imuData[i].bmpCalib.oss);
                g_imuData[i].temperatureC = BMP_GetTemperature(&g_imuData[i].bmpCalib, ut);
                g_imuData[i].pressurePa = BMP_GetPressure(&g_imuData[i].bmpCalib, up);
                g_imuData[i].altitudeM = BMP_GetAltitude((float)g_imuData[i].pressurePa);
            }
            g_imuData[i].bmpDivCounter++;
            if (g_imuData[i].bmpDivCounter >= BMP180_DIV)
            {
                g_imuData[i].bmpDivCounter = 0;
            }
        }

        /* 使用实测磁场量做AHRS融合，得到更稳定的绝对航向 */
        AHRS_Update(&g_imuData[i].ahrs,
            g_imuData[i].gyroDps[0], g_imuData[i].gyroDps[1], g_imuData[i].gyroDps[2],
            g_imuData[i].accG[0], g_imuData[i].accG[1], g_imuData[i].accG[2],
            g_imuData[i].magCal[0], g_imuData[i].magCal[1], g_imuData[i].magCal[2],
            dt);

        AHRS_GetEulerDeg(&g_imuData[i].ahrs,
            &g_imuData[i].roll,
            &g_imuData[i].pitch,
            &g_imuData[i].yaw);
    }
}

const IMU_PoseDataTypeDef* IMU_Multi_Get(uint8_t index)
{
    if (index >= IMU_COUNT)
    {
        return 0;
    }
    return &g_imuData[index];
}
