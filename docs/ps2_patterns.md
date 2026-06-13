# PS2 Button Mapping and Shape Patterns

## Button usage in this project

- Used by default
- `SELECT + START`: latches emergency stop.
  - `PAD_UP/DOWN/LEFT/RIGHT`: overrides right stick X/Y when pressed.
- Shape drawing (added)
  - `PSB_TRIANGLE`: draw triangle
  - `PSB_CROSS`: draw X
  - `PSB_CIRCLE`: draw circle
  - `PSB_SQUARE`: draw square
- Safety and gripper
  - `PSB_L1`: open gripper
  - `PSB_R1`: close gripper
  - `PSB_L2`: disable motor output
  - `PSB_R2`: enable motor output
  - `SELECT + START`: latch emergency stop
  - `START + R2`: clear emergency stop and enable
  - `L3 + R3`: software home/reset speed loop

## How shape drawing works

1. Keep both sticks centered (inside the deadzone).
2. Press one of the four shape buttons.
3. The firmware generates a time-based virtual stick command and feeds it into the same mixing logic used for manual control.
4. Moving either stick cancels the pattern and returns to manual control.

This logic is implemented in `USER/App.c`; `USER/main.c` only runs initialization, scheduling, and telemetry.

## Tuning knobs

You can tune the speed and duration here:

- `PATTERN_SPEED_NORM`: overall pattern speed (0..1)
- `PATTERN_TRIANGLE_SEG_SEC`, `PATTERN_SQUARE_SEG_SEC`, `PATTERN_CROSS_SEG_SEC`: segment times
- `PATTERN_CIRCLE_PERIOD_SEC`: circle period
- `MAIN_LOOP_DT_SEC` must match the loop delay (`Delay_ms(10)`)

## Notes

- These patterns are open-loop velocity commands. The actual path depends on mechanical load, friction, and calibration.
- For precise shape tracking, add position feedback and/or kinematics-based position control.
