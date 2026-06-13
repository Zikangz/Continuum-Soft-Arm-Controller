# -*- coding: utf-8 -*-
import argparse
import csv
import math
import os
from collections import deque
import tkinter as tk
from tkinter import ttk

import matplotlib as mpl
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
import serial
from serial.tools import list_ports


mpl.rcParams["font.sans-serif"] = [
    "Microsoft YaHei",
    "SimHei",
    "Noto Sans CJK SC",
    "WenQuanYi Zen Hei",
    "Source Han Sans CN",
    "DejaVu Sans",
]
mpl.rcParams["axes.unicode_minus"] = False


IMU_TAGS = ("IMU1", "IMU2", "IMU3")
IMU_KEYS = ("t", "o", "r", "p", "y", "temp", "press", "alt")
CSV_FIELDS = ["imu"] + list(IMU_KEYS)
VIEW_MODES = ("ALL", "IMU1", "IMU2", "IMU3")
ARM_STATUS_KEYS = ("enabled", "estop", "gripper", "gripper_dir", "home_pending")
PATTERN_KEYS = {
    "t": "T",
    "1": "T",
    "x": "X",
    "2": "X",
    "o": "O",
    "3": "O",
    "q": "S",
    "4": "S",
}
AUX_KEY_COMMANDS = {
    "Escape": ("E", "estop", True),
    "Return": ("A,1", "enable", False),
    "BackSpace": ("A,0", "disable", True),
}
AUX_CHAR_COMMANDS = {
    "e": ("E", "estop", True),
    "r": ("A,1", "enable", False),
    "f": ("A,0", "disable", True),
    "[": ("G,0", "gripper close", False),
    "]": ("G,100", "gripper open", False),
    "\\": ("G,50", "gripper stop", False),
    "h": ("H", "home", True),
    "/": ("Q", "status", False),
    "?": ("Q", "status", False),
}


def parse_line(line: str):
    if not line:
        return []

    if line.startswith(("IMU1_HEADER", "IMU2_HEADER", "IMU3_HEADER", "CTRL_HEADER", "CTRL,")):
        return []

    parts = [x.strip() for x in line.split(",")]
    if not parts:
        return []

    prefix = parts[0]
    if prefix in IMU_TAGS:
        try:
            if len(parts) == 9:
                return [
                    (prefix, {
                        "t": float(parts[1]),
                        "o": int(parts[2]),
                        "r": float(parts[3]),
                        "p": float(parts[4]),
                        "y": float(parts[5]),
                        "temp": float(parts[6]),
                        "press": float(parts[7]),
                        "alt": float(parts[8]),
                    })
                ]
            if len(parts) == 6:
                return [
                    (prefix, {
                        "t": float(parts[1]),
                        "o": int(parts[2]),
                        "r": float(parts[3]),
                        "p": float(parts[4]),
                        "y": float(parts[5]),
                        "temp": math.nan,
                        "press": math.nan,
                        "alt": math.nan,
                    })
                ]
        except ValueError:
            return []

    if prefix != "IMU3":
        return []

    try:
        if len(parts) == 14:
            t = float(parts[1])
            return [
                ("IMU1", {"t": t, "o": int(parts[2]), "r": float(parts[3]), "p": float(parts[4]), "y": float(parts[5]),
                          "temp": math.nan, "press": math.nan, "alt": math.nan}),
                ("IMU2", {"t": t, "o": int(parts[6]), "r": float(parts[7]), "p": float(parts[8]), "y": float(parts[9]),
                          "temp": math.nan, "press": math.nan, "alt": math.nan}),
                ("IMU3", {"t": t, "o": int(parts[10]), "r": float(parts[11]), "p": float(parts[12]), "y": float(parts[13]),
                          "temp": math.nan, "press": math.nan, "alt": math.nan}),
            ]
        if len(parts) == 23:
            t = float(parts[1])
            return [
                ("IMU1", {"t": t, "o": int(parts[2]), "r": float(parts[3]), "p": float(parts[4]), "y": float(parts[5]),
                          "temp": float(parts[6]), "press": float(parts[7]), "alt": float(parts[8])}),
                ("IMU2", {"t": t, "o": int(parts[9]), "r": float(parts[10]), "p": float(parts[11]), "y": float(parts[12]),
                          "temp": float(parts[13]), "press": float(parts[14]), "alt": float(parts[15])}),
                ("IMU3", {"t": t, "o": int(parts[16]), "r": float(parts[17]), "p": float(parts[18]), "y": float(parts[19]),
                          "temp": float(parts[20]), "press": float(parts[21]), "alt": float(parts[22])}),
            ]
    except ValueError:
        return []

    return []


def parse_arm_status(line: str):
    parts = [x.strip() for x in line.split(",")]
    if len(parts) != 6 or parts[0] != "ARM":
        return None
    try:
        return {
            "enabled": int(parts[1]),
            "estop": int(parts[2]),
            "gripper": int(parts[3]),
            "gripper_dir": int(parts[4]),
            "home_pending": int(parts[5]),
        }
    except ValueError:
        return None


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def parse_lengths(text):
    values = [float(x.strip()) for x in text.split(",") if x.strip()]
    if len(values) != 3:
        raise argparse.ArgumentTypeError("--lengths must contain 3 comma-separated numbers")
    return values


def matmul(a, b):
    return [
        [sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)]
        for i in range(3)
    ]


def matvec(a, v):
    return [sum(a[i][k] * v[k] for k in range(3)) for i in range(3)]


def transpose(a):
    return [[a[j][i] for j in range(3)] for i in range(3)]


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


def rotation_angle(a, b):
    rel = matmul(transpose(a), b)
    trace = rel[0][0] + rel[1][1] + rel[2][2]
    c = clamp((trace - 1.0) * 0.5, -1.0, 1.0)
    return math.acos(c)


class HostKeyboardControl:
    def __init__(self, serial_port, speed_percent):
        self.serial = serial_port
        self.speed = int(clamp(speed_percent, 1, 100))
        self.keys = set()
        self.last_axes = None
        self.enabled = True
        self.last_aux = ""

    def attach(self, root):
        self.root = root
        root.bind("<KeyPress>", self.on_key_press)
        root.bind("<KeyRelease>", self.on_key_release)
        root.after(50, self.periodic_send)

    def write_line(self, text):
        if not self.enabled:
            return
        try:
            self.serial.write((text + "\n").encode("ascii"))
        except serial.SerialException:
            self.enabled = False

    def axes(self):
        feed = 0
        x_axis = 0
        y_axis = 0

        if "w" in self.keys:
            feed += self.speed
        if "s" in self.keys:
            feed -= self.speed
        if "d" in self.keys or "Right" in self.keys:
            x_axis += self.speed
        if "a" in self.keys or "Left" in self.keys:
            x_axis -= self.speed
        if "Up" in self.keys:
            y_axis += self.speed
        if "Down" in self.keys:
            y_axis -= self.speed

        return (
            int(clamp(feed, -100, 100)),
            int(clamp(x_axis, -100, 100)),
            int(clamp(y_axis, -100, 100)),
        )

    def send_axes(self, force=False):
        axes = self.axes()
        if axes == (0, 0, 0):
            if force or self.last_axes not in (None, (0, 0, 0)):
                self.write_line("S")
            self.last_axes = axes
            return

        if force or axes != self.last_axes:
            self.write_line(f"J,{axes[0]},{axes[1]},{axes[2]}")
        self.last_axes = axes

    def send_aux(self, command, label, clear_motion):
        if clear_motion:
            self.keys.clear()
            self.write_line("S")
            self.last_axes = (0, 0, 0)
        self.write_line(command)
        self.last_aux = label

    def periodic_send(self):
        if self.axes() != (0, 0, 0):
            self.send_axes(force=True)
        if hasattr(self, "root"):
            self.root.after(50, self.periodic_send)

    def on_key_press(self, event):
        key = event.keysym
        char = (event.char or "").lower()

        aux_command = AUX_KEY_COMMANDS.get(key) or AUX_CHAR_COMMANDS.get(char)
        if aux_command is not None:
            command, label, clear_motion = aux_command
            self.send_aux(command, label, clear_motion)
            return

        if key == "space":
            self.keys.clear()
            self.write_line("S")
            self.last_axes = (0, 0, 0)
            self.last_aux = "stop"
            return

        if char in PATTERN_KEYS:
            self.keys.clear()
            self.write_line("S")
            self.write_line(f"P,{PATTERN_KEYS[char]}")
            self.last_axes = (0, 0, 0)
            self.last_aux = f"pattern {PATTERN_KEYS[char]}"
            return

        self.keys.add(char if char in ("w", "a", "s", "d") else key)
        self.send_axes(force=True)

    def on_key_release(self, event):
        key = event.keysym
        char = (event.char or "").lower()
        self.keys.discard(char if char in ("w", "a", "s", "d") else key)
        self.send_axes(force=True)


class IMUArmConsole:
    def __init__(self, port, baud, csv_path, window_size, lengths, stiffness, lever_arm, key_speed):
        self.port = port
        self.baud = baud
        self.csv_path = csv_path
        self.window_size = window_size
        self.lengths = lengths
        self.stiffness = stiffness
        self.lever_arm = max(lever_arm, 1e-4)
        self.view_mode = "ALL"
        self.closed = False

        self.ser = serial.Serial(self.port, self.baud, timeout=0.02)
        self.ser.reset_input_buffer()
        self.keyboard = HostKeyboardControl(self.ser, key_speed)

        self.data = {tag: {k: deque(maxlen=self.window_size) for k in IMU_KEYS} for tag in IMU_TAGS}
        self.force = {"joint12": 0.0, "joint23": 0.0, "tip": 0.0}
        self.points = [[0.0, 0.0, 0.0] for _ in range(4)]
        self.arm_status = None

        self.csv_f = open(self.csv_path, "w", newline="", encoding="utf-8")
        self.writer = csv.writer(self.csv_f)
        self.writer.writerow(CSV_FIELDS)

        self.root = None
        self.canvas = None
        self.mode_combo = None
        self.status_var = None

        self.fig = Figure(figsize=(13, 8), dpi=100)
        grid = self.fig.add_gridspec(3, 2, width_ratios=(1.05, 1.25), hspace=0.34, wspace=0.28)
        self.ax_arm = self.fig.add_subplot(grid[:, 0], projection="3d")
        self.ax_r = self.fig.add_subplot(grid[0, 1])
        self.ax_p = self.fig.add_subplot(grid[1, 1])
        self.ax_y = self.fig.add_subplot(grid[2, 1])
        self.setup_axes()

    @property
    def csv_abs_path(self):
        return os.path.abspath(self.csv_path)

    def setup_axes(self):
        self.ax_r.set_ylabel("Roll (deg)")
        self.ax_p.set_ylabel("Pitch (deg)")
        self.ax_y.set_ylabel("Yaw (deg)")
        self.ax_y.set_xlabel("Time (s)")
        for ax in (self.ax_r, self.ax_p, self.ax_y):
            ax.grid(alpha=0.25)

    def on_mode_changed(self, _event=None):
        if self.mode_combo is None:
            return
        self.view_mode = self.mode_combo.get()

    def latest_records(self):
        records = {}
        for tag in IMU_TAGS:
            if self.data[tag]["t"]:
                records[tag] = {k: self.data[tag][k][-1] for k in IMU_KEYS}
        return records

    def estimate_arm(self):
        records = self.latest_records()
        if len(records) < 3:
            return

        rotations = []
        for tag in IMU_TAGS:
            rec = records[tag]
            if int(rec["o"]) == 0:
                return
            rotations.append(euler_zyx_deg(rec["r"], rec["p"], rec["y"]))

        points = [[0.0, 0.0, 0.0]]
        for rotation, length in zip(rotations, self.lengths):
            segment = matvec(rotation, [length, 0.0, 0.0])
            last = points[-1]
            points.append([last[i] + segment[i] for i in range(3)])
        self.points = points

        theta12 = rotation_angle(rotations[0], rotations[1])
        theta23 = rotation_angle(rotations[1], rotations[2])
        self.force["joint12"] = self.stiffness * theta12 / self.lever_arm
        self.force["joint23"] = self.stiffness * theta23 / self.lever_arm
        self.force["tip"] = self.force["joint12"] + self.force["joint23"]

    def draw_arm(self):
        self.ax_arm.clear()
        xs = [p[0] for p in self.points]
        ys = [p[1] for p in self.points]
        zs = [p[2] for p in self.points]
        self.ax_arm.plot(xs, ys, zs, "-o", linewidth=3.0, markersize=5)
        self.ax_arm.scatter([xs[-1]], [ys[-1]], [zs[-1]], s=55)
        total = max(sum(self.lengths), 0.1)
        self.ax_arm.set_xlim(-0.15 * total, total)
        self.ax_arm.set_ylim(-0.6 * total, 0.6 * total)
        self.ax_arm.set_zlim(-0.6 * total, 0.6 * total)
        self.ax_arm.set_xlabel("X (m)")
        self.ax_arm.set_ylabel("Y (m)")
        self.ax_arm.set_zlabel("Z (m)")
        self.ax_arm.set_title(
            f"Arm pose | F12={self.force['joint12']:.2f}N "
            f"F23={self.force['joint23']:.2f}N Tip={self.force['tip']:.2f}N"
        )

    def draw_all_mode(self):
        axes = [self.ax_r, self.ax_p, self.ax_y]
        colors = {"r": "#d62728", "p": "#1f77b4", "y": "#2ca02c"}
        names = {"r": "Roll", "p": "Pitch", "y": "Yaw"}

        for idx, tag in enumerate(IMU_TAGS):
            ax = axes[idx]
            if len(self.data[tag]["t"]) < 2:
                ax.set_title(f"{tag} waiting data")
                ax.set_ylabel("Angle (deg)")
                ax.grid(alpha=0.25)
                continue

            t = list(self.data[tag]["t"])
            for k in ("r", "p", "y"):
                ax.plot(t, list(self.data[tag][k]), linestyle="--", linewidth=1.25, color=colors[k], label=names[k])

            ax.set_title(f"{tag} attitude")
            ax.set_ylabel("Angle (deg)")
            ax.grid(alpha=0.25)
            ax.legend(loc="upper right")

        self.ax_y.set_xlabel("Time (s)")

    def draw_single_mode(self, tag):
        if len(self.data[tag]["t"]) < 2:
            return

        t = list(self.data[tag]["t"])
        self.ax_r.plot(t, list(self.data[tag]["r"]), linewidth=1.4, label=tag)
        self.ax_p.plot(t, list(self.data[tag]["p"]), linewidth=1.4, label=tag)
        self.ax_y.plot(t, list(self.data[tag]["y"]), linewidth=1.4, label=tag)
        self.ax_r.legend(loc="upper right")
        self.ax_p.legend(loc="upper right")
        self.ax_y.legend(loc="upper right")

    def redraw(self):
        self.estimate_arm()
        self.draw_arm()

        for ax in (self.ax_r, self.ax_p, self.ax_y):
            ax.clear()
            ax.grid(alpha=0.25)
        self.ax_r.set_ylabel("Roll (deg)")
        self.ax_p.set_ylabel("Pitch (deg)")
        self.ax_y.set_ylabel("Yaw (deg)")
        self.ax_y.set_xlabel("Time (s)")

        if self.view_mode == "ALL":
            self.draw_all_mode()
        else:
            self.draw_single_mode(self.view_mode)

        if self.status_var is not None:
            axes = self.keyboard.axes()
            if self.arm_status is None:
                arm_text = "ARM unknown"
            else:
                arm_text = (
                    f"ARM en={self.arm_status['enabled']} estop={self.arm_status['estop']} "
                    f"grip={self.arm_status['gripper']}% dir={self.arm_status['gripper_dir']}"
                )
            self.status_var.set(
                f"Port {self.port} @ {self.baud} | {arm_text} | keys feed/x/y={axes} | "
                f"last={self.keyboard.last_aux or '-'} | "
                f"Space stop, Esc/E estop, Enter/R enable, Backspace/F disable, "
                f"[/] gripper, H home, / status"
            )

    def read_serial_once(self):
        updated = False
        while self.ser.in_waiting:
            raw = self.ser.readline()
            line = raw.decode("utf-8", errors="ignore").strip()
            arm_status = parse_arm_status(line)
            if arm_status is not None:
                self.arm_status = arm_status
                updated = True
                continue

            recs = parse_line(line)
            if not recs:
                continue

            for tag, rec in recs:
                if tag not in self.data:
                    continue
                for k in IMU_KEYS:
                    self.data[tag][k].append(rec[k])
                self.writer.writerow([tag] + [rec[k] for k in IMU_KEYS])
                updated = True

        return updated

    def tick(self):
        if self.closed:
            return

        updated = self.read_serial_once()
        if updated:
            self.redraw()
            if self.canvas is not None:
                self.canvas.draw_idle()

        if self.root is not None:
            self.root.after(50, self.tick)

    def close(self):
        self.closed = True
        self.keyboard.write_line("S")
        try:
            self.csv_f.close()
        except Exception:
            pass
        try:
            self.ser.close()
        except Exception:
            pass
        if self.root is not None:
            self.root.destroy()

    def run(self):
        self.root = tk.Tk()
        self.keyboard.root = self.root
        self.root.title("IMU Arm Console")
        self.root.geometry("1320x860")
        self.root.option_add("*Font", "{Microsoft YaHei} 10")

        style = ttk.Style(self.root)
        try:
            style.theme_use("clam")
        except Exception:
            pass

        top_frame = ttk.Frame(self.root, padding=(10, 8, 10, 6))
        top_frame.pack(side=tk.TOP, fill=tk.X)

        ttk.Label(top_frame, text="View").pack(side=tk.LEFT, padx=(0, 8))
        self.mode_combo = ttk.Combobox(top_frame, state="readonly", width=12, values=list(VIEW_MODES))
        self.mode_combo.current(0)
        self.mode_combo.pack(side=tk.LEFT)
        self.mode_combo.bind("<<ComboboxSelected>>", self.on_mode_changed)

        self.status_var = tk.StringVar(value="")
        ttk.Label(top_frame, textvariable=self.status_var).pack(side=tk.LEFT, padx=(16, 0))

        canvas_frame = ttk.Frame(self.root, padding=(8, 0, 8, 8))
        canvas_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        self.canvas = FigureCanvasTkAgg(self.fig, master=canvas_frame)
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        toolbar = NavigationToolbar2Tk(self.canvas, canvas_frame)
        toolbar.update()
        toolbar.pack(side=tk.BOTTOM, fill=tk.X)

        self.keyboard.attach(self.root)
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.redraw()
        self.tick()
        self.root.mainloop()


def main():
    parser = argparse.ArgumentParser(description="Three-IMU arm visualization and keyboard control console")
    parser.add_argument("--port", help="Serial port, e.g. /dev/ttyUSB0 or COM5")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate")
    parser.add_argument("--window", type=int, default=500, help="Plot window size")
    parser.add_argument("--csv", default="imu_pose_log.csv", help="CSV output file")
    parser.add_argument("--lengths", type=parse_lengths, default=[0.18, 0.16, 0.12],
                        help="Arm segment lengths in meters, e.g. 0.18,0.16,0.12")
    parser.add_argument("--stiffness", type=float, default=0.35,
                        help="Equivalent bending stiffness in N*m/rad, calibrate for force estimate")
    parser.add_argument("--lever-arm", type=float, default=0.04,
                        help="Equivalent force lever arm in meters, calibrate for force estimate")
    parser.add_argument("--key-speed", type=int, default=75,
                        help="Keyboard command magnitude, 1..100")
    args = parser.parse_args()

    port = args.port
    if port is None:
        ports = list(list_ports.comports())
        if not ports:
            print("No serial ports detected. Use --port to specify one.")
            return
        port = ports[0].device
        print(f"Using serial port: {port}")

    csv_path = args.csv if os.path.isabs(args.csv) else os.path.join(os.path.dirname(os.path.abspath(__file__)), args.csv)
    console = IMUArmConsole(
        port=port,
        baud=args.baud,
        csv_path=csv_path,
        window_size=args.window,
        lengths=args.lengths,
        stiffness=args.stiffness,
        lever_arm=args.lever_arm,
        key_speed=args.key_speed,
    )
    print(f"CSV output: {console.csv_abs_path}")
    console.run()


if __name__ == "__main__":
    main()
