# Mechanical Arm Control Notes

## Firmware responsibility

- STM32 keeps the deterministic low-level loop: PS2/host input -> virtual joystick axes -> 3-motor velocity mix -> encoder speed loop -> TB6612 PWM.
- Jetson/PC keeps high-level work: IMU visualization, pose/force estimate, keyboard teleoperation, depth-camera autonomous target reaching.
- This keeps the MCU safe and simple; the host can change estimation and camera logic without changing motor timing.

## USART3 control protocol

Serial settings: `115200 8N1`, PB10 TX, PB11 RX.

Commands sent from host to STM32 end with `\n`:

- `J,<feed>,<x>,<y>`: virtual joystick command, each value in `-100..100`.
  - `feed`: same as PS2 left stick Y.
  - `x`: same as PS2 right stick X / left-right bending.
  - `y`: same as PS2 right stick Y / up-down bending.
- `P,T`: draw triangle.
- `P,X`: draw cross.
- `P,O`: draw circle.
- `P,S`: draw square.
- `S`: stop host command and return to neutral/PS2 control after timeout.
- `E`: latch emergency stop, stop motors and gripper outputs.
- `A,0`: disable arm motor output.
- `A,1`: clear emergency latch and enable arm motor output after safety confirmation.
- `G,<open_percent>`: command gripper, `0` close, `100` open, `50` stop output.
- `G,O`, `G,C`, `G,S`: gripper open, close, stop aliases.
- `H`: reset speed-loop state and encoder counters as a software home/reset action.
- `Q`: request one `ARM` status frame.

STM32 also publishes:

- `ARM,enabled,estop,gripper,gripper_dir,home_pending`

`gripper_dir` is `1` opening, `-1` closing, `0` idle.

Host joystick commands time out after about 200 ms if the host stops sending packets.

## Host keyboard mapping

Run:

```powershell
python jetson_imu3_realtime_plot.py --port COM5 --baud 115200
```

Keys:

- `W/S`: feed forward/back.
- `A/D` or left/right arrows: bend left/right.
- up/down arrows: bend up/down.
- `T` or `1`: triangle.
- `X` or `2`: cross.
- `O` or `3`: circle.
- `Q` or `4`: square.
- Space: stop.
- `Esc` or `E`: emergency stop.
- `Enter` or `R`: clear emergency stop and enable.
- `Backspace` or `F`: disable arm motor output.
- `[` / `]`: gripper close/open.
- `\`: stop gripper output.
- `H`: software home/reset.
- `/` or `?`: request one `ARM` status frame.

## Pose and force estimate

The host uses the latest roll/pitch/yaw of IMU1..IMU3 as three segment orientations and draws a 3D chain.

Force is an equivalent bending-force estimate:

```text
force = equivalent_stiffness * relative_angle / equivalent_lever_arm
```

The default values are only placeholders. Calibrate `--stiffness` and `--lever-arm` with known loads before using force values for decisions.

## Autonomous target reaching

Run on Jetson with Intel RealSense support installed:

```powershell
python jetson_autonomous_grasp.py --port COM5 --baud 115200 --target-distance 0.18
```

The script:

- reads STM32 IMU telemetry,
- reads depth frames,
- selects the nearest valid target in the center ROI,
- closes the loop on image X/Y error and depth error,
- uses IMU tip direction as an attitude bias,
- sends `J,feed,x,y` commands back to STM32.

Tune signs and gains on the real arm:

- `--pixel-gain`
- `--depth-gain`
- `--imu-gain`
- `--max-speed`
- `--target-distance`

The firmware drives the arm toward the target position and now defines `G,<open_percent>` for gripper open/close. Verify the actual gripper driver polarity and travel limit before powering the actuator.
