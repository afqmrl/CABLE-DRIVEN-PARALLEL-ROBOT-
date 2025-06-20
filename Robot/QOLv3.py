#!/usr/bin/env python
import sys
import time
import threading
import numpy as np
import matplotlib.pyplot as plt
import pygame
import os
from scservo_sdk import *
from scipy.spatial.transform import Rotation as R
import json

# ============ Configuration ============
BAUDRATE = 115200
DEVICENAME = '/dev/ttyUSB0'  
MOTOR_IDS = [8, 6, 4, 5, 11, 7, 1, 2]

ACCELERATION = 50
MOVE_SPEED = 1000
MANUAL_SPEED = 2400
ENCODER_TICKS_PER_REV = 4096 
POSITION_THRESHOLD = 100

absolute_positions = {m: 0 for m in MOTOR_IDS}
previous_encoders = {m: 0 for m in MOTOR_IDS}
target_positions = {m: 0 for m in MOTOR_IDS}
cable_lengths = {m: 0 for m in MOTOR_IDS}
initial_motor_pos = {}
initial_cable_length = {}

# PID states
pid_errors = {m: 0 for m in MOTOR_IDS}
pid_integrals = {m: 0 for m in MOTOR_IDS}
pid_derivatives = {m: 0 for m in MOTOR_IDS}
previous_pid_errors = {m: 0 for m in MOTOR_IDS}
pid_outputs_log = {m: [] for m in MOTOR_IDS}

# PID gains
Kp = 1.0
Ki = 0.0
Kd = 0.0

portHandler = PortHandler(DEVICENAME)
packetHandler = sms_sts(portHandler)

# Global Flags
manual_override = False
exit_program = False


def mode_watcher():
    global manual_override, exit_program
    while not exit_program:
        try:
            with open("input_mode.json") as f:
                mode = json.load(f).get("mode", "")
            manual_override = (mode == "manual")
        except:
            pass
        time.sleep(0.1)

def json_keyboard_loop():
    global exit_program
    prev_len = 0
    key_to_motor = {
        '1':8,'q':8,'2':6,'w':6,'3':4,'e':4,'4':5,'r':5,
        '5':11,'t':11,'6':7,'y':7,'7':1,'u':1,'8':2,'i':2
    }
    speeds = {
        '1':MANUAL_SPEED, 'q':-MANUAL_SPEED, '2':MANUAL_SPEED, 'w':-MANUAL_SPEED, 
        '3':MANUAL_SPEED, 'e':-MANUAL_SPEED, '4':MANUAL_SPEED, 'r':-MANUAL_SPEED,
        '5':MANUAL_SPEED, 't':-MANUAL_SPEED, '6':MANUAL_SPEED, 'y':-MANUAL_SPEED,
        '7':MANUAL_SPEED, 'u':-MANUAL_SPEED, '8':MANUAL_SPEED, 'i':-MANUAL_SPEED
    }

    while not exit_program:
        if manual_override:
            try:
                with open("keyboard_input.json") as f:
                    keys = json.load(f)
            except:
                keys = []
            # only process new keys
            new_keys = keys[prev_len:]
            for ch in new_keys:
                if ch == ' ':
                    for m in MOTOR_IDS:
                        write_speed(m, 0)
                elif ch in key_to_motor:
                    m, s = key_to_motor[ch], speeds[ch]
                    write_speed(m, s)
                # else ignore unmapped
            prev_len = len(keys)
        time.sleep(0.05)


# ============ SCServo Initialization ============
def init_motors():
    if not portHandler.openPort() or not portHandler.setBaudRate(BAUDRATE):
        sys.exit("Failed to initialize port")
    for motor_id in MOTOR_IDS:
        packetHandler.WheelMode(motor_id)
        enc = packetHandler.ReadPos(motor_id)
        initial_motor_pos[motor_id] = enc[0] if isinstance(enc, tuple) else enc

# ============ Encoder & Control ============
def update_absolute_position(motor_id):
    global initial_motor_pos, previous_encoders, absolute_positions

    with thread_lock:
        if motor_id not in initial_motor_pos:
            return
        cur = packetHandler.ReadPos(motor_id)[0]
        offset = cur - initial_motor_pos[motor_id]

        if offset - previous_encoders[motor_id] > 2000:
            absolute_positions[motor_id] -= ENCODER_TICKS_PER_REV
        elif offset - previous_encoders[motor_id] < -2000:
            absolute_positions[motor_id] += ENCODER_TICKS_PER_REV

        absolute_positions[motor_id] += offset - previous_encoders[motor_id]
        previous_encoders[motor_id] = offset

def synchronized_motor_control(dt=0.05):
    for m in MOTOR_IDS:
        err = target_positions[m] - absolute_positions[m]

        pid_integrals[m] += err * dt
        pid_derivatives[m] = (err - previous_pid_errors[m]) / dt
        previous_pid_errors[m] = err

        output = Kp * err + Ki * pid_integrals[m] + Kd * pid_derivatives[m]
        pid_outputs_log[m].append(output)

        if abs(err) < POSITION_THRESHOLD:
            speed = 0
        else:
            speed = int(np.clip(output, -MOVE_SPEED, MOVE_SPEED))

        write_speed(m, speed)

def write_speed(motor_id, speed):
    packetHandler.WriteSpec(motor_id, speed, ACCELERATION)

# ============ IK & Joystick ============
motor_positions_ik = np.array([
    [ 0.42, -0.18, 0.095], [ 0.42,  0.18, 0.095], [ 0.42,  0.18, 0.49], [ 0.42, -0.18, 0.49],
    [-0.42, -0.18, 0.095], [-0.42,  0.18, 0.095], [-0.42,  0.18, 0.49], [-0.42, -0.18, 0.49]
])
platform_anchors = np.array([
    [ 0.04, -0.04, 0], [ 0.04,  0.04, 0], [ 0.04,  0.04, 0], [ 0.04, -0.04, 0],
    [-0.04, -0.04, 0], [-0.04,  0.04, 0], [-0.04,  0.04, 0], [-0.04, -0.04, 0]
])

def calculate_cable_lengths(pos, rot):
    return [np.linalg.norm(pos + rot.dot(anchor) - motor)
            for motor, anchor in zip(motor_positions_ik, platform_anchors)]

pygame.init()
pygame.joystick.init()
joystick = pygame.joystick.Joystick(0)
joystick.init()

pos = np.array([0.0, 0.0, 0.3])
rot = R.from_euler('xyz', [0, 0, 0]).as_matrix()

def apply_deadzone(value, threshold=0.35):
    return value if abs(value) >= threshold else 0.0

def joystick_loop():
    global pos, rot, manual_override, exit_program
    while not exit_program:
        if not manual_override:
            pygame.event.pump()
            axes = [joystick.get_axis(i) for i in range(joystick.get_numaxes())]
            # print every axis index & value
            print("Axes:", ", ".join(f"{i}:{v:+.3f}" for i, v in enumerate(axes)), end="\r")
            lx = apply_deadzone(axes[0])
            ly = apply_deadzone(axes[1])
            rx = apply_deadzone(axes[3])
            ry = apply_deadzone(axes[4])
            tri, sqr = joystick.get_button(3), joystick.get_button(2)
            x_b, cir = joystick.get_button(0), joystick.get_button(1)

            pos += np.array([lx, -ly, tri - sqr]) * 0.00025
            pos = np.clip(pos, [-0.43, -0.15, 0.15], [0.43, 0.15, 0.6])

            # --- Rotation ---
            euler = R.from_matrix(rot).as_euler('xyz')  # get current rotation in radians
            euler += np.array([-ry, -rx, x_b - cir]) * 0.005  # apply joystick input

            # Limit rotation
            limit_rad = np.radians(300)  
            euler = np.clip(euler, -limit_rad, limit_rad)

            rot = R.from_euler('xyz', euler).as_matrix()

        time.sleep(0.01)

def update_targets():
    lengths = calculate_cable_lengths(pos, rot)
    for i, motor_id in enumerate(MOTOR_IDS):
        if motor_id not in initial_cable_length:
            initial_cable_length[motor_id] = lengths[i]
        delta_length = lengths[i] - initial_cable_length[motor_id]
        if motor_id in [6, 5, 11, 1]:
            delta_length *= -1
        target_positions[motor_id] = delta_length * (ENCODER_TICKS_PER_REV*2 / (np.pi*0.04))

# ============ Keyboard Manual Control ============
thread_lock = threading.Lock()

# def keyboard_loop():
#     global manual_override, initial_motor_pos, initial_cable_length
#     global previous_encoders, absolute_positions, pos, rot

#     # def getch():
#     #     import msvcrt
#     #     return msvcrt.getch().decode()

#     key_to_motor = {
#         '1':8,'q':8,'2':6,'w':6,'3':4,'e':4,'4':5,'r':5,
#         '5':11,'t':11,'6':7,'y':7,'7':1,'u':1,'8':2,'i':2
#     }
#     speeds = {
#         '1':MANUAL_SPEED, 'q':-MANUAL_SPEED, '2':MANUAL_SPEED, 'w':-MANUAL_SPEED, 
#         '3':MANUAL_SPEED, 'e':-MANUAL_SPEED, '4':MANUAL_SPEED, 'r':-MANUAL_SPEED,
#         '5':MANUAL_SPEED, 't':-MANUAL_SPEED, '6':MANUAL_SPEED, 'y':-MANUAL_SPEED,
#         '7':MANUAL_SPEED, 'u':-MANUAL_SPEED, '8':MANUAL_SPEED, 'i':-MANUAL_SPEED
#     }

#     print("Keyboard control ready: Press ESC to toggle joystick/manual, SPACE to stop motors.")

    # while not exit_program:
    #     ch = getch()

    #     if ch == chr(0x1b):
    #         manual_override = not manual_override
    #         mode = "MANUAL" if manual_override else "JOYSTICK"
    #         print(f"\n--- {mode} MODE ACTIVE ---\n")

    #         if not manual_override:
    #             with thread_lock:
    #                 for m in MOTOR_IDS:
    #                     write_speed(m, 0)
    #                 # pos[:] = np.array([0.0, 0.0, 0.3])
    #                 # rot[:] = R.from_euler('xyz', [0.0, 0.0, 0.0]).as_matrix()
    #                 initial_cable_length.clear()
    #                 initial_motor_pos.clear()

    #                 for motor_id in MOTOR_IDS:
    #                     enc = packetHandler.ReadPos(motor_id)
    #                     enc = enc[0] if isinstance(enc, tuple) else enc
    #                     if enc == -1:
    #                         print(f"Error reading encoder from motor {motor_id}, defaulting to zero.")
    #                         enc = 0
    #                     initial_motor_pos[motor_id] = enc
    #                     previous_encoders[motor_id] = 0
    #                     absolute_positions[motor_id] = 0

    #                 print("Joystick mode reinitialized at position [0, 0, 0.3].")

    #     elif manual_override:
    #         if ch == ' ':
    #             for m in MOTOR_IDS:
    #                 write_speed(m, 0)
    #             print("All motors stopped manually.")
    #         elif ch in key_to_motor:
    #             motor_id, speed = key_to_motor[ch], speeds[ch]
    #             write_speed(motor_id, speed)
    #             print(f"Motor {motor_id} set to speed {speed}")
    #         else:
    #             print(f"Key '{ch}' not mapped.")

# ============ Main Loop ============
def main():
    global exit_program, manual_override

    init_motors()
    threading.Thread(target=joystick_loop, daemon=True).start()
    threading.Thread(target=json_keyboard_loop, daemon=True).start()
    threading.Thread(target=mode_watcher,    daemon=True).start()

    last_print = time.time()
    while not exit_program:
        if not manual_override:
            update_targets()
            for m in MOTOR_IDS:
                update_absolute_position(m)
            synchronized_motor_control()

            current_time = time.time()
            if current_time - last_print >= 1.0:
                euler_deg = R.from_matrix(rot).as_euler('xyz', degrees=True)
                print(f"\nJoystick Mode Active")
                print(f"Position: x={pos[0]:.3f}, y={pos[1]:.3f}, z={pos[2]:.3f} m")
                print(f"Rotation: roll={euler_deg[0]:.2f}°, pitch={euler_deg[1]:.2f}°, yaw={euler_deg[2]:.2f}°\n")
                last_print = current_time
        time.sleep(0.05)

    for m in MOTOR_IDS:
        write_speed(m, 0)
    portHandler.closePort()
    pygame.quit()

    # Plot PID outputs
    plt.figure(figsize=(12, 6))
    for m in MOTOR_IDS:
        plt.plot(pid_outputs_log[m], label=f"Motor {m}")
    plt.title("PID Output Over Time")
    plt.xlabel("Iteration")
    plt.ylabel("Output Speed Command")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()

    sys.exit()

if __name__ == "__main__":
    main()

