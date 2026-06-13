#include "include.h"

#include <math.h>

#define AXIS_FULL_SCALE           (127.0f)
#define AXIS_DEADZONE_NORM        (0.08f)
#define AXIS_EXPO                 (0.55f)
#define PLANAR_SPEED_SCALE        (4200.0f)
#define FEED_SPEED_SCALE          (2400.0f)
#define MIX_COEFF_SIN60           (0.8660254f)
#define MIX_COEFF_HALF            (0.5f)
#define MIX_INV_2SIN60            (0.57735027f)
#define MIX_INV_SIN60             (1.15470054f)
#define PWM_TO_TARGET_SCALE       (1.0f / 80.0f)
#define TARGET_LPF_ALPHA          (0.35f)
#define MAIN_LOOP_DT_SEC          (0.01f)
#define PI_F                      (3.1415926f)
#define INV_SQRT2_F               (0.70710678f)
#define PATTERN_SPEED_NORM        (0.65f)
#define PATTERN_TRIANGLE_SEG_SEC  (0.9f)
#define PATTERN_SQUARE_SEG_SEC    (0.9f)
#define PATTERN_CROSS_SEG_SEC     (0.6f)
#define PATTERN_CIRCLE_PERIOD_SEC (4.0f)
#define GRIPPER_OPEN_RCC          RCC_APB2Periph_GPIOB
#define GRIPPER_CLOSE_RCC         RCC_APB2Periph_GPIOB
#define GRIPPER_OPEN_PORT         GPIOB
#define GRIPPER_CLOSE_PORT        GPIOB
#define GRIPPER_OPEN_PIN          GPIO_Pin_2
#define GRIPPER_CLOSE_PIN         GPIO_Pin_3
#define GRIPPER_PULSE_TICKS       (50u)

/* Rod position control parameters (runtime-tunable via Z/K/M serial commands) */
#define ROD_SIGN_1             (-1.0f)    /* direction sign, fixed */
#define ROD_SIGN_2             (-1.0f)
#define ROD_SIGN_3             (-1.0f)
#define ROD_TELEMETRY_PERIOD   (50u)      /* print ROD telemetry every N ticks */

/* Runtime-tunable globals (modified by Z/K/M serial commands) */
float g_home_L1_mm         = 157.2f;  /* rod length at encoder=0 */
float g_home_L2_mm         = 157.2f;
float g_home_L3_mm         = 157.2f;
float g_rod_kp_pos       = 80.0f;    /* position loop P gain */
float g_rod_tolerance_mm = 2.0f;     /* mm, arrival deadband */
int   g_rod_speed_limit  = 3000;     /* max speed counts per motor */
float g_mm_per_count_1   = 0.002448f; /* mm per encoder count, motor 1 */
float g_mm_per_count_2   = 0.002388f; /* mm per encoder count, motor 2 */
float g_mm_per_count_3   = 0.002424f; /* mm per encoder count, motor 3 */

typedef enum
{
	PATTERN_NONE = 0,
	PATTERN_TRIANGLE,
	PATTERN_CROSS,
	PATTERN_CIRCLE,
	PATTERN_SQUARE
} PatternType;

typedef struct
{
	PatternType type;
	float t;
	float duration;
} PatternState;

static uint8 g_ps2_online = 0u;
static uint8 g_arm_enabled = 0u;  /* start disabled for safety */
static uint8 g_estop_latched = 0u;
static uint8 g_home_pending = 0u;
static uint8 g_gripper_percent = 50u;
static uint8 g_gripper_pulse_ticks = 0u;
static int8 g_gripper_direction = 0;

/* Rod position control state */
static uint8 g_rod_position_mode = 0u;
static float g_target_L1_mm = 0.0f;
static float g_target_L2_mm = 0.0f;
static float g_target_L3_mm = 0.0f;
static float g_current_L1_mm = 0.0f;
static float g_current_L2_mm = 0.0f;
static float g_current_L3_mm = 0.0f;
static uint16_t g_rod_telemetry_divider = 0u;

static uint8 ClampJoystickCenter(uint8 value)
{
	if (value >= 126u && value <= 130u)
	{
		return 128u;
	}

	return value;
}

static int clamp_int(int value, int min_value, int max_value)
{
	if (value > max_value) return max_value;
	if (value < min_value) return min_value;
	return value;
}

static float clampf(float value, float min_value, float max_value)
{
	if (value > max_value) return max_value;
	if (value < min_value) return min_value;
	return value;
}

static float normalize_delta(int delta)
{
	float v = (float)delta / AXIS_FULL_SCALE;
	return clampf(v, -1.0f, 1.0f);
}

static float apply_deadzone_expo(float x, float deadzone, float expo)
{
	float sign = 1.0f;
	float ax = x;
	float t;
	float curved;

	if (x < 0.0f)
	{
		sign = -1.0f;
		ax = -x;
	}

	if (ax <= deadzone)
	{
		return 0.0f;
	}

	t = (ax - deadzone) / (1.0f - deadzone);
	curved = (1.0f - expo) * t + expo * t * t * t;
	return sign * curved;
}

static int round_to_int(float v)
{
	if (v >= 0.0f)
	{
		return (int)(v + 0.5f);
	}
	return (int)(v - 0.5f);
}

static void Gripper_StopOutput(void)
{
	GPIO_ResetBits(GRIPPER_OPEN_PORT, GRIPPER_OPEN_PIN);
	GPIO_ResetBits(GRIPPER_CLOSE_PORT, GRIPPER_CLOSE_PIN);
	g_gripper_pulse_ticks = 0u;
	g_gripper_direction = 0;
}

static void Gripper_CommandPercent(uint8 percent)
{
	g_gripper_percent = (uint8)clamp_int((int)percent, 0, 100);
	Gripper_StopOutput();

	if (g_gripper_percent >= 60u)
	{
		GPIO_SetBits(GRIPPER_OPEN_PORT, GRIPPER_OPEN_PIN);
		g_gripper_direction = 1;
		g_gripper_pulse_ticks = GRIPPER_PULSE_TICKS;
	}
	else if (g_gripper_percent <= 40u)
	{
		GPIO_SetBits(GRIPPER_CLOSE_PORT, GRIPPER_CLOSE_PIN);
		g_gripper_direction = -1;
		g_gripper_pulse_ticks = GRIPPER_PULSE_TICKS;
	}
}

static void Arm_ForceStop(void)
{
	g_arm_enabled = 0u;
	g_estop_latched = 1u;
	g_home_pending = 0u;
	Gripper_StopOutput();
	ClosedLoop_Reset();
	Set_Car_Speed(0, 0, 0);
}

void ArmControl_Init(void)
{
	GPIO_InitTypeDef gpio;

	RCC_APB2PeriphClockCmd(GRIPPER_OPEN_RCC | GRIPPER_CLOSE_RCC | RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

	gpio.GPIO_Mode = GPIO_Mode_Out_PP;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	gpio.GPIO_Pin = GRIPPER_OPEN_PIN;
	GPIO_Init(GRIPPER_OPEN_PORT, &gpio);
	gpio.GPIO_Pin = GRIPPER_CLOSE_PIN;
	GPIO_Init(GRIPPER_CLOSE_PORT, &gpio);

	Gripper_StopOutput();
}

void ArmControl_Tick10ms(void)
{
	if (g_gripper_pulse_ticks > 0u)
	{
		g_gripper_pulse_ticks--;
		if (g_gripper_pulse_ticks == 0u)
		{
			Gripper_StopOutput();
		}
	}
}

void ArmControl_ReportStatus(void)
{
	IMUSerial_Printf("ARM,%d,%d,%d,%d,%d\r\n",
		g_arm_enabled,
		g_estop_latched,
		g_gripper_percent,
		(int)g_gripper_direction,
		g_home_pending);
}

/* ------------------------------------------------------------------
 * Compute current rod lengths from encoder totals
 * L_i_mm = home_L_i_mm + total_i * mm_per_count_i * sign_i
 * ------------------------------------------------------------------ */
static void UpdateRodLengths(void)
{
	int32_t c1 = 0L, c2 = 0L, c3 = 0L;
	Encoder_GetTotals(&c1, &c2, &c3);
	g_current_L1_mm = g_home_L1_mm + (float)c1 * g_mm_per_count_1 * ROD_SIGN_1;
	g_current_L2_mm = g_home_L2_mm + (float)c2 * g_mm_per_count_2 * ROD_SIGN_2;
	g_current_L3_mm = g_home_L3_mm + (float)c3 * g_mm_per_count_3 * ROD_SIGN_3;
}

/* ------------------------------------------------------------------
 * Process L,<l1>,<l2>,<l3> rod position target
 * Returns 1 while actively moving to target, 0 when idle.
 * ------------------------------------------------------------------ */
static uint8 ProcessRodPosition(void)
{
	float err1, err2, err3;
	int speed1, speed2, speed3;

	if (g_rod_position_mode == 0u)
	{
		return 0u;
	}

	UpdateRodLengths();

	err1 = g_target_L1_mm - g_current_L1_mm;
	err2 = g_target_L2_mm - g_current_L2_mm;
	err3 = g_target_L3_mm - g_current_L3_mm;

	/* Per-rod arrival: zero speed for rods within tolerance */
	if (fabsf(err1) < g_rod_tolerance_mm) err1 = 0.0f;
	if (fabsf(err2) < g_rod_tolerance_mm) err2 = 0.0f;
	if (fabsf(err3) < g_rod_tolerance_mm) err3 = 0.0f;

	/* All rods arrived -> stop position mode */
	if ((err1 == 0.0f) && (err2 == 0.0f) && (err3 == 0.0f))
	{
		ClosedLoop_Update(0, 0, 0);
		g_rod_position_mode = 0u;
		return 0u;
	}

	/* P-only position loop: each rod independent */
	speed1 = (int)(g_rod_kp_pos * err1);
	speed2 = (int)(g_rod_kp_pos * err2);
	speed3 = (int)(g_rod_kp_pos * err3);

	speed1 = clamp_int(speed1, -g_rod_speed_limit, g_rod_speed_limit);
	speed2 = clamp_int(speed2, -g_rod_speed_limit, g_rod_speed_limit);
	speed3 = clamp_int(speed3, -g_rod_speed_limit, g_rod_speed_limit);

	ClosedLoop_Update(speed1, speed2, speed3);
	return 1u;
}

/* ------------------------------------------------------------------
 * Print rod telemetry for host (ROD,t,L1_mm,L2_mm,L3_mm,V1,V2,V3,mode)
 * ------------------------------------------------------------------ */
static void PrintRodTelemetry(void)
{
	int16_t v1, v2, v3;

	g_rod_telemetry_divider++;
	if (g_rod_telemetry_divider < ROD_TELEMETRY_PERIOD)
	{
		return;
	}
	g_rod_telemetry_divider = 0u;

	UpdateRodLengths();
	Encoder_GetVelocity(&v1, &v2, &v3);
	IMUSerial_Printf("ROD,%.2f,%.1f,%.1f,%.1f,%d,%d,%d,%d\r\n",
		0.0f, /* t placeholder - can add timer later */
		g_current_L1_mm, g_current_L2_mm, g_current_L3_mm,
		(int)v1, (int)v2, (int)v3,
		(int)g_rod_position_mode);
}

static uint8 axis_to_ps2_byte(int axis, int center, int sign)
{
	int value;

	axis = clamp_int(axis, -100, 100);
	value = center + sign * ((axis * 127) / 100);
	return (uint8)clamp_int(value, 0, 255);
}

static float wrap_time(float t, float period)
{
	if (period <= 0.0f)
	{
		return 0.0f;
	}
	while (t >= period)
	{
		t -= period;
	}
	return t;
}

static float pattern_duration(PatternType type)
{
	switch (type)
	{
		case PATTERN_TRIANGLE:
			return 3.0f * PATTERN_TRIANGLE_SEG_SEC;
		case PATTERN_CROSS:
			return 4.0f * PATTERN_CROSS_SEG_SEC;
		case PATTERN_CIRCLE:
			return PATTERN_CIRCLE_PERIOD_SEC;
		case PATTERN_SQUARE:
			return 4.0f * PATTERN_SQUARE_SEG_SEC;
		default:
			return 0.0f;
	}
}

static PatternType pattern_from_host_key(char key)
{
	switch (key)
	{
		case 'T':
		case 't':
			return PATTERN_TRIANGLE;
		case 'X':
		case 'x':
			return PATTERN_CROSS;
		case 'O':
		case 'o':
		case 'C':
		case 'c':
			return PATTERN_CIRCLE;
		case 'S':
		case 's':
		case 'Q':
		case 'q':
			return PATTERN_SQUARE;
		default:
			return PATTERN_NONE;
	}
}

static void pattern_start(PatternState *state, PatternType type)
{
	if (state == 0)
	{
		return;
	}
	state->type = type;
	state->t = 0.0f;
	state->duration = pattern_duration(type);
}

static void pattern_stop(PatternState *state)
{
	if (state == 0)
	{
		return;
	}
	state->type = PATTERN_NONE;
	state->t = 0.0f;
	state->duration = 0.0f;
}

static void pattern_axes(const PatternState *state, float *x_axis, float *y_axis, float *f_axis)
{
	float phase;
	float angle;
	float w;
	float seg;
	float speed = PATTERN_SPEED_NORM;

	if ((state == 0) || (x_axis == 0) || (y_axis == 0) || (f_axis == 0))
	{
		return;
	}

	*f_axis = 0.0f;

	switch (state->type)
	{
		case PATTERN_TRIANGLE:
			seg = PATTERN_TRIANGLE_SEG_SEC;
			phase = wrap_time(state->t, 3.0f * seg);
			if (phase < seg)
			{
				angle = 0.0f;
			}
			else if (phase < 2.0f * seg)
			{
				angle = 2.0f * PI_F / 3.0f;
			}
			else
			{
				angle = 4.0f * PI_F / 3.0f;
			}
			*x_axis = speed * cosf(angle);
			*y_axis = speed * sinf(angle);
			break;

		case PATTERN_CROSS:
			seg = PATTERN_CROSS_SEG_SEC;
			phase = wrap_time(state->t, 4.0f * seg);
			if (phase < seg)
			{
				*x_axis = speed * INV_SQRT2_F;
				*y_axis = speed * INV_SQRT2_F;
			}
			else if (phase < 2.0f * seg)
			{
				*x_axis = -speed * INV_SQRT2_F;
				*y_axis = -speed * INV_SQRT2_F;
			}
			else if (phase < 3.0f * seg)
			{
				*x_axis = -speed * INV_SQRT2_F;
				*y_axis = speed * INV_SQRT2_F;
			}
			else
			{
				*x_axis = speed * INV_SQRT2_F;
				*y_axis = -speed * INV_SQRT2_F;
			}
			break;

		case PATTERN_CIRCLE:
			w = 2.0f * PI_F / PATTERN_CIRCLE_PERIOD_SEC;
			*x_axis = -speed * sinf(w * state->t);
			*y_axis = speed * cosf(w * state->t);
			break;

		case PATTERN_SQUARE:
			seg = PATTERN_SQUARE_SEG_SEC;
			phase = wrap_time(state->t, 4.0f * seg);
			if (phase < seg)
			{
				*x_axis = speed;
				*y_axis = 0.0f;
			}
			else if (phase < 2.0f * seg)
			{
				*x_axis = 0.0f;
				*y_axis = speed;
			}
			else if (phase < 3.0f * seg)
			{
				*x_axis = -speed;
				*y_axis = 0.0f;
			}
			else
			{
				*x_axis = 0.0f;
				*y_axis = -speed;
			}
			break;

		default:
			*x_axis = 0.0f;
			*y_axis = 0.0f;
			break;
	}
}

static void mix_planar_axes(float x_axis, float y_axis, float *v1, float *v2, float *v3)
{
	if ((v1 == 0) || (v2 == 0) || (v3 == 0))
	{
		return;
	}

	if (y_axis >= (MIX_COEFF_HALF * x_axis) && y_axis > (-MIX_COEFF_HALF * x_axis))
	{
		*v1 = 0.0f;
		*v2 = y_axis + x_axis * MIX_INV_2SIN60;
		*v3 = y_axis - x_axis * MIX_INV_2SIN60;
	}
	else if (y_axis <= (-MIX_COEFF_HALF * x_axis) && x_axis < 0.0f)
	{
		*v3 = -x_axis * MIX_INV_SIN60;
		*v2 = 0.0f;
		*v1 = MIX_COEFF_HALF * (*v3) - y_axis;
	}
	else
	{
		*v2 = x_axis * MIX_INV_SIN60;
		*v3 = 0.0f;
		*v1 = MIX_COEFF_HALF * (*v2) - y_axis;
	}
}

static void HandleSafetyAndAuxInputs(void)
{
	int8_t enable_request;
	int16_t gripper_request;

	if (IMUSerial_ConsumeHostEstop() != 0u)
	{
		Arm_ForceStop();
	}

	enable_request = IMUSerial_ConsumeHostEnable();
	if (enable_request == 0)
	{
		g_arm_enabled = 0u;
		ClosedLoop_Reset();
		Set_Car_Speed(0, 0, 0);
	}
	else if ((enable_request > 0) && (g_estop_latched == 0u))
	{
		g_arm_enabled = 1u;
	}
	else if (enable_request > 0)
	{
		g_estop_latched = 0u;
		g_arm_enabled = 1u;
		ClosedLoop_Reset();
	}

	gripper_request = IMUSerial_ConsumeHostGripper();
	if ((gripper_request >= 0) && (g_estop_latched == 0u))
	{
		Gripper_CommandPercent((uint8)gripper_request);
	}

	if (IMUSerial_ConsumeHostHome() != 0u)
	{
		g_home_pending = 1u;
	}

	if (IMUSerial_ConsumeHostStatusRequest() != 0u)
	{
		ArmControl_ReportStatus();
	}

	if (g_ps2_online == 0u)
	{
		return;
	}

	if (PS2_Button(PSB_SELECT) && PS2_Button(PSB_START))
	{
		Arm_ForceStop();
		return;
	}

	if (PS2_Button(PSB_START) && PS2_ButtonPressed(PSB_R2))
	{
		g_estop_latched = 0u;
		g_arm_enabled = 1u;
		ClosedLoop_Reset();
		return;
	}

	if (PS2_ButtonPressed(PSB_L2))
	{
		g_arm_enabled = 0u;
		ClosedLoop_Reset();
		Set_Car_Speed(0, 0, 0);
	}
	else if ((PS2_ButtonPressed(PSB_R2)) && (g_estop_latched == 0u))
	{
		g_arm_enabled = 1u;
	}

	if (g_estop_latched == 0u)
	{
		if (PS2_ButtonPressed(PSB_L1))
		{
			Gripper_CommandPercent(100u);
		}
		else if (PS2_ButtonPressed(PSB_R1))
		{
			Gripper_CommandPercent(0u);
		}

		if (PS2_Button(PSB_L3) && PS2_Button(PSB_R3))
		{
			g_home_pending = 1u;
		}
	}
}

void TaskRun(u8 ps2_ok, u8 L_data[], u8 R_data[])
{
	uint8 PS2KeyValue = 0u;
	int8_t host_feed = 0;
	int8_t host_x = 0;
	int8_t host_y = 0;

	if ((L_data == 0) || (R_data == 0))
	{
		return;
	}

	L_data[0] = CENTER_X;
	L_data[1] = CENTER_Y;
	R_data[0] = CENTER_X;
	R_data[1] = CENTER_Y;
	g_ps2_online = (ps2_ok == 0u) ? 1u : 0u;

	if (g_ps2_online != 0u)
	{
		PS2KeyValue = PS2_DataKey();
		L_data[0] = ClampJoystickCenter(PS2_AnologData(PSS_LX));
		L_data[1] = ClampJoystickCenter(PS2_AnologData(PSS_LY));
		R_data[0] = ClampJoystickCenter(PS2_AnologData(PSS_RX));
		R_data[1] = ClampJoystickCenter(PS2_AnologData(PSS_RY));

		switch (PS2KeyValue)
		{
			case PSB_PAD_LEFT:
				R_data[0] = 0u;
				break;
			case PSB_PAD_RIGHT:
				R_data[0] = 255u;
				break;
			case PSB_PAD_UP:
				R_data[1] = 0u;
				break;
			case PSB_PAD_DOWN:
				R_data[1] = 255u;
				break;
			default:
				break;
		}
	}

	if (IMUSerial_GetHostAxes(&host_feed, &host_x, &host_y) != 0u)
	{
		L_data[0] = CENTER_X;
		L_data[1] = axis_to_ps2_byte((int)host_feed, CENTER_Y, -1);
		R_data[0] = axis_to_ps2_byte((int)host_x, CENTER_X, 1);
		R_data[1] = axis_to_ps2_byte((int)host_y, CENTER_Y, -1);
	}

	HandleSafetyAndAuxInputs();

	if ((g_arm_enabled == 0u) || (g_estop_latched != 0u))
	{
		L_data[0] = CENTER_X;
		L_data[1] = CENTER_Y;
		R_data[0] = CENTER_X;
		R_data[1] = CENTER_Y;
	}
}

void HandleControl(u8 L_data[], u8 R_data[])
{
	static PatternState pattern = {PATTERN_NONE, 0.0f, 0.0f};
	static float target1_f = 0.0f;
	static float target2_f = 0.0f;
	static float target3_f = 0.0f;
	PatternType request = PATTERN_NONE;
	char host_pattern;
	int speed1;
	int speed2;
	int speed3;
	int target1;
	int target2;
	int target3;
	int l_y_delta;
	int r_x_delta;
	int r_y_delta;
	float f_axis;
	float x_axis;
	float y_axis;
	float v1 = 0.0f;
	float v2 = 0.0f;
	float v3 = 0.0f;
	float speed1_f;
	float speed2_f;
	float speed3_f;
	float max_abs_v;
	float max_abs_speed;
	float speed_scale;
	uint8 sticks_idle;

	if ((L_data == 0) || (R_data == 0))
	{
		return;
	}

	if (g_home_pending != 0u)
	{
		g_home_pending = 0u;
		target1_f = 0.0f;
		target2_f = 0.0f;
		target3_f = 0.0f;
		g_rod_position_mode = 0u;
		pattern_stop(&pattern);
		ClosedLoop_Reset();
		Set_Car_Speed(0, 0, 0);
		ArmControl_ReportStatus();
		return;
	}

	if ((g_arm_enabled == 0u) || (g_estop_latched != 0u))
	{
		target1_f = 0.0f;
		target2_f = 0.0f;
		target3_f = 0.0f;
		g_rod_position_mode = 0u;
		pattern_stop(&pattern);
		ClosedLoop_Update(0, 0, 0);
		return;
	}

	/* -- Rod position mode (L protocol) -- */
	l_y_delta = CENTER_Y - (int)L_data[1];
	r_x_delta = (int)R_data[0] - CENTER_X;
	r_y_delta = CENTER_Y - (int)R_data[1];

	f_axis = apply_deadzone_expo(normalize_delta(l_y_delta), AXIS_DEADZONE_NORM, AXIS_EXPO);
	x_axis = apply_deadzone_expo(normalize_delta(r_x_delta), AXIS_DEADZONE_NORM, AXIS_EXPO);
	y_axis = apply_deadzone_expo(normalize_delta(r_y_delta), AXIS_DEADZONE_NORM, AXIS_EXPO);

	sticks_idle = ((f_axis == 0.0f) && (x_axis == 0.0f) && (y_axis == 0.0f)) ? 1u : 0u;

	/* Consume new L target even when sticks are idle */
	{
		float hl1, hl2, hl3;
		if (IMUSerial_ConsumeHostRodTarget(&hl1, &hl2, &hl3) != 0u)
		{
			g_target_L1_mm = hl1;
			g_target_L2_mm = hl2;
			g_target_L3_mm = hl3;
			g_rod_position_mode = 1u;
		}
	}

	/* If sticks are active, cancel rod position mode (manual override) */
	if (sticks_idle == 0u)
	{
		g_rod_position_mode = 0u;
	}

	/* Execute rod position control if active and sticks are idle */
	if (g_rod_position_mode != 0u)
	{
		ProcessRodPosition();
		PrintRodTelemetry();
		return;
	}

	/* Print telemetry periodically even in manual/speed mode */
	PrintRodTelemetry();

	host_pattern = IMUSerial_ConsumeHostPattern();

	if (sticks_idle != 0u)
	{
		request = pattern_from_host_key(host_pattern);
		if ((request == PATTERN_NONE) && (g_ps2_online != 0u))
		{
			if (PS2_ButtonPressed(PSB_TRIANGLE))
			{
				request = PATTERN_TRIANGLE;
			}
			else if (PS2_ButtonPressed(PSB_CROSS))
			{
				request = PATTERN_CROSS;
			}
			else if (PS2_ButtonPressed(PSB_CIRCLE))
			{
				request = PATTERN_CIRCLE;
			}
			else if (PS2_ButtonPressed(PSB_SQUARE))
			{
				request = PATTERN_SQUARE;
			}
		}
	}

	if (request != PATTERN_NONE)
	{
		pattern_start(&pattern, request);
	}

	if (pattern.type != PATTERN_NONE)
	{
		if (sticks_idle == 0u)
		{
			pattern_stop(&pattern);
		}
		else
		{
			pattern_axes(&pattern, &x_axis, &y_axis, &f_axis);
			pattern.t += MAIN_LOOP_DT_SEC;
			if (pattern.t >= pattern.duration)
			{
				pattern_stop(&pattern);
			}
		}
	}

	mix_planar_axes(x_axis, y_axis, &v1, &v2, &v3);

	max_abs_v = fabsf(v1);
	if (fabsf(v2) > max_abs_v) max_abs_v = fabsf(v2);
	if (fabsf(v3) > max_abs_v) max_abs_v = fabsf(v3);
	if (max_abs_v > 1.0f)
	{
		v1 /= max_abs_v;
		v2 /= max_abs_v;
		v3 /= max_abs_v;
	}

	speed1_f = v1 * PLANAR_SPEED_SCALE + f_axis * FEED_SPEED_SCALE;
	speed2_f = v2 * PLANAR_SPEED_SCALE + f_axis * FEED_SPEED_SCALE;
	speed3_f = v3 * PLANAR_SPEED_SCALE + f_axis * FEED_SPEED_SCALE;

	max_abs_speed = fabsf(speed1_f);
	if (fabsf(speed2_f) > max_abs_speed) max_abs_speed = fabsf(speed2_f);
	if (fabsf(speed3_f) > max_abs_speed) max_abs_speed = fabsf(speed3_f);
	if (max_abs_speed > (float)MAX_SPEED)
	{
		speed_scale = (float)MAX_SPEED / max_abs_speed;
		speed1_f *= speed_scale;
		speed2_f *= speed_scale;
		speed3_f *= speed_scale;
	}

	speed1 = round_to_int(speed1_f);
	speed2 = round_to_int(speed2_f);
	speed3 = round_to_int(speed3_f);

	speed1 = clamp_int(speed1, MIN_SPEED, MAX_SPEED);
	speed2 = clamp_int(speed2, MIN_SPEED, MAX_SPEED);
	speed3 = clamp_int(speed3, MIN_SPEED, MAX_SPEED);

	if ((speed1 == 0) && (speed2 == 0) && (speed3 == 0))
	{
		target1_f = 0.0f;
		target2_f = 0.0f;
		target3_f = 0.0f;
		ClosedLoop_Update(0, 0, 0);
	}
	else
	{
		target1_f += TARGET_LPF_ALPHA * (((float)speed1 * PWM_TO_TARGET_SCALE) - target1_f);
		target2_f += TARGET_LPF_ALPHA * (((float)speed2 * PWM_TO_TARGET_SCALE) - target2_f);
		target3_f += TARGET_LPF_ALPHA * (((float)speed3 * PWM_TO_TARGET_SCALE) - target3_f);
		target1 = round_to_int(target1_f);
		target2 = round_to_int(target2_f);
		target3 = round_to_int(target3_f);
		ClosedLoop_Update(target1, target2, target3);
	}
}
