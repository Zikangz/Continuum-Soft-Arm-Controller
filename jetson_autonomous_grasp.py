# -*- coding: utf-8 -*-
import argparse
import math
import time

import numpy as np
import serial


IMU_TAGS = ("IMU1", "IMU2", "IMU3")


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def parse_imu_line(line):
    parts = [x.strip() for x in line.split(",")]
    if len(parts) not in (6, 9):
        return None
    if parts[0] not in IMU_TAGS:
        return None
    try:
        return parts[0], {
            "t": float(parts[1]),
            "online": int(parts[2]),
            "roll": float(parts[3]),
            "pitch": float(parts[4]),
            "yaw": float(parts[5]),
        }
    except ValueError:
        return None


def matmul(a, b):
    return [
        [sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)]
        for i in range(3)
    ]


def matvec(a, v):
    return [sum(a[i][k] * v[k] for k in range(3)) for i in range(3)]


def euler_zyx_deg(roll, pitch, yaw):
    cr = math.cos(math.radians(roll))
    sr = math.sin(math.radians(roll))
    cp = math.cos(math.radians(pitch))
    sp = math.sin(math.radians(pitch))
    cy = math.cos(math.radians(yaw))
    sy = math.sin(math.radians(yaw))
    rz = [[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]]
    ry = [[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]]
    rx = [[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]]
    return matmul(matmul(rz, ry), rx)


class SerialArmLink:
    def __init__(self, port, baud):
        self.ser = serial.Serial(port, baud, timeout=0.002)
        self.ser.reset_input_buffer()
        self.latest_imu = {}

    def close(self):
        try:
            self.stop()
            self.ser.close()
        except serial.SerialException:
            pass

    def write_line(self, text):
        self.ser.write((text + "\n").encode("ascii"))

    def command(self, feed, x_axis, y_axis):
        feed = int(clamp(feed, -100, 100))
        x_axis = int(clamp(x_axis, -100, 100))
        y_axis = int(clamp(y_axis, -100, 100))
        self.write_line(f"J,{feed},{x_axis},{y_axis}")

    def stop(self):
        self.write_line("S")

    def poll_imu(self):
        while self.ser.in_waiting:
            raw = self.ser.readline()
            line = raw.decode("utf-8", errors="ignore").strip()
            rec = parse_imu_line(line)
            if rec is None:
                continue
            tag, data = rec
            self.latest_imu[tag] = data

    def imu_ready(self):
        return all(tag in self.latest_imu and self.latest_imu[tag]["online"] for tag in IMU_TAGS)

    def tip_direction(self):
        if not self.imu_ready():
            return [1.0, 0.0, 0.0]
        imu3 = self.latest_imu["IMU3"]
        rotation = euler_zyx_deg(imu3["roll"], imu3["pitch"], imu3["yaw"])
        direction = matvec(rotation, [1.0, 0.0, 0.0])
        length = math.sqrt(sum(v * v for v in direction)) or 1.0
        return [v / length for v in direction]


class RealSenseTarget:
    def __init__(self, width, height, fps, roi_scale, min_depth, max_depth):
        try:
            import pyrealsense2 as rs
        except ImportError as exc:
            raise RuntimeError("pyrealsense2 is required for depth-camera autonomous mode") from exc

        self.rs = rs
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        profile = self.pipeline.start(self.config)
        sensor = profile.get_device().first_depth_sensor()
        self.depth_scale = sensor.get_depth_scale()
        self.roi_scale = roi_scale
        self.min_depth = min_depth
        self.max_depth = max_depth
        self.width = width
        self.height = height

    def close(self):
        self.pipeline.stop()

    def nearest_target(self):
        frames = self.pipeline.wait_for_frames(timeout_ms=1000)
        depth_frame = frames.get_depth_frame()
        if not depth_frame:
            return None

        depth = np.asanyarray(depth_frame.get_data()).astype(np.float32) * self.depth_scale
        h, w = depth.shape
        cx = w // 2
        cy = h // 2
        half_w = max(8, int(w * self.roi_scale * 0.5))
        half_h = max(8, int(h * self.roi_scale * 0.5))
        x0 = max(0, cx - half_w)
        x1 = min(w, cx + half_w)
        y0 = max(0, cy - half_h)
        y1 = min(h, cy + half_h)
        roi = depth[y0:y1, x0:x1]
        valid = np.isfinite(roi) & (roi > self.min_depth) & (roi < self.max_depth)
        if not np.any(valid):
            return None

        masked = np.where(valid, roi, np.inf)
        local_v, local_u = np.unravel_index(np.argmin(masked), masked.shape)
        u = x0 + int(local_u)
        v = y0 + int(local_v)
        patch = depth[max(0, v - 2):min(h, v + 3), max(0, u - 2):min(w, u + 3)]
        patch_valid = patch[(patch > self.min_depth) & (patch < self.max_depth)]
        z = float(np.median(patch_valid)) if patch_valid.size else float(depth[v, u])
        return {"u": u, "v": v, "z": z, "width": w, "height": h}


class AutonomousController:
    def __init__(self, link, camera, target_distance, max_speed, pixel_gain, depth_gain, imu_gain, tolerance_px, tolerance_z):
        self.link = link
        self.camera = camera
        self.target_distance = target_distance
        self.max_speed = max_speed
        self.pixel_gain = pixel_gain
        self.depth_gain = depth_gain
        self.imu_gain = imu_gain
        self.tolerance_px = tolerance_px
        self.tolerance_z = tolerance_z
        self.last_print = 0.0

    def step(self):
        self.link.poll_imu()
        if not self.link.imu_ready():
            self.link.stop()
            self.report("waiting for all IMUs online")
            return

        target = self.camera.nearest_target()
        if target is None:
            self.link.stop()
            self.report("no depth target in ROI")
            return

        cx = target["width"] * 0.5
        cy = target["height"] * 0.5
        x_err = (target["u"] - cx) / cx
        y_err = (cy - target["v"]) / cy
        z_err = target["z"] - self.target_distance

        tip = self.link.tip_direction()
        imu_x_bias = -tip[1]
        imu_y_bias = tip[2]

        x_cmd = (self.pixel_gain * x_err + self.imu_gain * imu_x_bias) * self.max_speed
        y_cmd = (self.pixel_gain * y_err + self.imu_gain * imu_y_bias) * self.max_speed
        feed_cmd = self.depth_gain * z_err * self.max_speed

        if abs(x_err) < self.tolerance_px and abs(y_err) < self.tolerance_px and abs(z_err) < self.tolerance_z:
            self.link.stop()
            self.report(f"target reached z={target['z']:.3f}m")
            return

        self.link.command(feed_cmd, x_cmd, y_cmd)
        self.report(
            f"err x={x_err:+.3f} y={y_err:+.3f} z={z_err:+.3f} "
            f"cmd feed/x/y={feed_cmd:+.0f}/{x_cmd:+.0f}/{y_cmd:+.0f}"
        )

    def report(self, text):
        now = time.time()
        if now - self.last_print > 0.5:
            print(text)
            self.last_print = now


def main():
    parser = argparse.ArgumentParser(description="Depth-camera + IMU autonomous target reaching")
    parser.add_argument("--port", required=True, help="STM32 serial port")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--roi-scale", type=float, default=0.55)
    parser.add_argument("--min-depth", type=float, default=0.12)
    parser.add_argument("--max-depth", type=float, default=1.20)
    parser.add_argument("--target-distance", type=float, default=0.18)
    parser.add_argument("--max-speed", type=float, default=55.0)
    parser.add_argument("--pixel-gain", type=float, default=0.9)
    parser.add_argument("--depth-gain", type=float, default=2.0)
    parser.add_argument("--imu-gain", type=float, default=0.25)
    parser.add_argument("--tolerance-px", type=float, default=0.045)
    parser.add_argument("--tolerance-z", type=float, default=0.020)
    args = parser.parse_args()

    link = SerialArmLink(args.port, args.baud)
    camera = RealSenseTarget(
        width=args.width,
        height=args.height,
        fps=args.fps,
        roi_scale=clamp(args.roi_scale, 0.1, 1.0),
        min_depth=args.min_depth,
        max_depth=args.max_depth,
    )
    controller = AutonomousController(
        link=link,
        camera=camera,
        target_distance=args.target_distance,
        max_speed=args.max_speed,
        pixel_gain=args.pixel_gain,
        depth_gain=args.depth_gain,
        imu_gain=args.imu_gain,
        tolerance_px=args.tolerance_px,
        tolerance_z=args.tolerance_z,
    )

    try:
        while True:
            controller.step()
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("stopping")
    finally:
        link.close()
        camera.close()


if __name__ == "__main__":
    main()
