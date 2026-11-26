"""Slider window for ONE loose STS3215 servo: position, speed, acceleration, sweep (like the video's GUI).

usage: python tools\\servo_gui.py COM5 [--id 1]

- Starts with torque OFF. "Torque ON (here)" holds the motor where it is (no jump), then the position
  slider moves it. Speed / acceleration sliders apply to the next move.
- "Sweep" goes back and forth between the two marks A and B (set them from the position slider).
- The status line shows position, speed, load, voltage and temperature 5 times a second.
- Closing the window (or "Torque OFF") releases torque and puts the motor back in position mode.
Only tkinter (comes with python) and LeRobot are used. All serial access happens in the window's own
thread (after() timers), so there is never a second thread on the port.
"""

import argparse
import tkinter as tk

from servo_common import (
    check_answers,
    clamp,
    name_of,
    open_bus,
    read_status,
    refuse_if_arm,
    safe_close,
    set_caps,
    torque_on_here,
)

ap = argparse.ArgumentParser()
ap.add_argument("port")
ap.add_argument("--id", type=int, default=1)
args = ap.parse_args()

bus = open_bus(args.port, [args.id])
refuse_if_arm(bus)
check_answers(bus, [args.id])
safe_close_done = False
N = name_of(args.id)
MIN_POS, MAX_POS = 50, 4045

root = tk.Tk()
root.title(f"STS3215 id {args.id} on {args.port}")

pos_var = tk.IntVar(value=2048)
speed_var = tk.IntVar(value=800)
accel_var = tk.IntVar(value=20)
mark_a, mark_b = tk.IntVar(value=1500), tk.IntVar(value=2500)
torque_on = tk.BooleanVar(value=False)
sweeping = tk.BooleanVar(value=False)
status = tk.StringVar(value="torque OFF - press 'Torque ON (here)'")
sweep_goal = {"v": None}


def send_goal(*_):
    if torque_on.get() and not sweeping.get():
        set_caps(bus, args.id, speed_var.get(), accel_var.get())
        bus.write("Goal_Position", N, clamp(pos_var.get(), MIN_POS, MAX_POS), normalize=False)


def do_torque_on():
    here = torque_on_here(bus, args.id, speed_var.get(), accel_var.get())
    pos_var.set(here)  # slider jumps to the motor, not the motor to the slider
    torque_on.set(True)


def do_torque_off():
    sweeping.set(False)
    bus.disable_torque(N, num_retry=2)
    torque_on.set(False)


def toggle_sweep():
    if not torque_on.get():
        status.set("turn torque ON first")
        return
    sweeping.set(not sweeping.get())
    sweep_goal["v"] = None


def tick():
    try:
        st = read_status(bus, args.id)
        if sweeping.get():
            a, b = clamp(mark_a.get(), MIN_POS, MAX_POS), clamp(mark_b.get(), MIN_POS, MAX_POS)
            if sweep_goal["v"] is None or abs(st["position"] - sweep_goal["v"]) <= 15:
                sweep_goal["v"] = b if sweep_goal["v"] == a else a
                set_caps(bus, args.id, speed_var.get(), accel_var.get())
                bus.write("Goal_Position", N, sweep_goal["v"], normalize=False)
        status.set(f"pos {st['position']:>4}   speed {st['velocity']:>5}   load {st['load']:>5}   "
                   f"{st['volts']:.1f} V   {st['temp_c']} C   torque {'ON' if st['torque'] else 'OFF'}"
                   + ("   SWEEP" if sweeping.get() else ""))
        if not torque_on.get():
            pos_var.set(st["position"])  # follow the hand while torque is off
    except Exception as e:  # noqa: BLE001
        status.set(f"read error: {e}")
    root.after(200, tick)


def on_close():
    global safe_close_done
    if not safe_close_done:
        safe_close(bus, [args.id])
        safe_close_done = True
    root.destroy()


def slider(label, var, lo, hi, cmd=None):
    f = tk.Frame(root)
    tk.Label(f, text=label, width=14, anchor="w").pack(side="left")
    tk.Scale(f, variable=var, from_=lo, to=hi, orient="horizontal", length=420, command=cmd).pack(side="left")
    f.pack(fill="x", padx=8)


slider("position (steps)", pos_var, MIN_POS, MAX_POS, send_goal)
slider("speed (steps/s)", speed_var, 50, 3000)
slider("accel (x100/s2)", accel_var, 1, 254)
slider("sweep mark A", mark_a, MIN_POS, MAX_POS)
slider("sweep mark B", mark_b, MIN_POS, MAX_POS)
bf = tk.Frame(root)
tk.Button(bf, text="Torque ON (here)", command=do_torque_on).pack(side="left", padx=4)
tk.Button(bf, text="Torque OFF", command=do_torque_off).pack(side="left", padx=4)
tk.Button(bf, text="Sweep on/off", command=toggle_sweep).pack(side="left", padx=4)
bf.pack(pady=6)
tk.Label(root, textvariable=status, font=("Consolas", 10)).pack(padx=8, pady=6)
root.protocol("WM_DELETE_WINDOW", on_close)
root.after(200, tick)
try:
    root.mainloop()
finally:
    if not safe_close_done:
        safe_close(bus, [args.id])
