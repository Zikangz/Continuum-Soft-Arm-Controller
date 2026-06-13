#include "include.h"
#include "misc.h"

#define MAIN_LOOP_DT_SEC (0.01f)
#define IMU_REPORT_PERIOD_TICKS (2u)
#define ARM_STATUS_PERIOD_TICKS (10u)

extern void ArmControl_Init(void);
extern void ArmControl_Tick10ms(void);
extern void ArmControl_ReportStatus(void);

static void PrintImuHeader(void)
{
	IMUSerial_Printf("IMU1_HEADER,t,o,r,p,y,temp,press,alt\r\n");
	IMUSerial_Printf("IMU2_HEADER,t,o,r,p,y,temp,press,alt\r\n");
	IMUSerial_Printf("IMU3_HEADER,t,o,r,p,y,temp,press,alt\r\n");
	IMUSerial_Printf("CTRL_HEADER,cmd,format\r\n");
	IMUSerial_Printf("CTRL,J,feed,x,y; P,T/X/O/S; S; E; A,0/1; G,0..100/O/C/S; H; Q\r\n");
	IMUSerial_Printf("ARM_HEADER,enabled,estop,gripper,gripper_dir,home_pending\r\n");
}

static void PrintImuPose(float imu_time)
{
	const IMU_PoseDataTypeDef *imu1 = IMU_Multi_Get(0);
	const IMU_PoseDataTypeDef *imu2 = IMU_Multi_Get(1);
	const IMU_PoseDataTypeDef *imu3 = IMU_Multi_Get(2);

	if ((imu1 == 0) || (imu2 == 0) || (imu3 == 0))
	{
		return;
	}

	IMUSerial_Printf("IMU1,%.3f,%d,%.2f,%.2f,%.2f,%.2f,%ld,%.2f\r\n",
		imu_time, imu1->online, imu1->roll, imu1->pitch, imu1->yaw,
		imu1->temperatureC, (long)imu1->pressurePa, imu1->altitudeM);
	IMUSerial_Printf("IMU2,%.3f,%d,%.2f,%.2f,%.2f,%.2f,%ld,%.2f\r\n",
		imu_time, imu2->online, imu2->roll, imu2->pitch, imu2->yaw,
		imu2->temperatureC, (long)imu2->pressurePa, imu2->altitudeM);
	IMUSerial_Printf("IMU3,%.3f,%d,%.2f,%.2f,%.2f,%.2f,%ld,%.2f\r\n",
		imu_time, imu3->online, imu3->roll, imu3->pitch, imu3->yaw,
		imu3->temperatureC, (long)imu3->pressurePa, imu3->altitudeM);
}

int main(void)
{
	uint8 ps_ok;
	float imu_time = 0.0f;
	uint8 report_divider = 0u;
	uint8 status_divider = 0u;
	u8 L_data[2] = {CENTER_X, CENTER_Y};
	u8 R_data[2] = {CENTER_X, CENTER_Y};

	SystemInit();
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	ps_ok = InitPS2();
	TIM4_PWM_Init(7199, 0);
	MOTOR_GPIO_Init();
	Encoder_Init();
	ClosedLoop_Reset();
	Set_Car_Speed(0, 0, 0);
	IMUSerial_Init(115200);
	IMU_Multi_Init();
	ArmControl_Init();

	PrintImuHeader();
	ArmControl_ReportStatus();

	while (1)
	{
		IMUSerial_PollCommand();
		TaskRun(ps_ok, L_data, R_data);
		HandleControl(L_data, R_data);

		IMU_Multi_Update(MAIN_LOOP_DT_SEC);
		imu_time += MAIN_LOOP_DT_SEC;

		report_divider++;
		if (report_divider >= IMU_REPORT_PERIOD_TICKS)
		{
			report_divider = 0u;
			PrintImuPose(imu_time);
		}

		status_divider++;
		if (status_divider >= ARM_STATUS_PERIOD_TICKS)
		{
			status_divider = 0u;
			ArmControl_ReportStatus();
		}

		Delay_ms(10);
		IMUSerial_Tick10ms();
		ArmControl_Tick10ms();
	}
}
