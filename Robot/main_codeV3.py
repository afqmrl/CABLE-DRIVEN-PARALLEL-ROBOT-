#!/usr/bin/env python
import sys
import time
import threading
import numpy as np
import json
import os
import pygame
from scservo_sdk import *  # SC Servo SDK library
from scipy.spatial.transform import Rotation as R

# ============ Configuration ============
BAUDRATE = 115200
DEVICENAME = '/dev/ttyUSB0'  
MOTOR_IDS = [8, 6, 4, 5, 11, 7, 1, 2]

ACCELERATION = 255
MOVE_SPEED = 2400
MANUAL_SPEED = 2400
ENCODER_TICKS_PER_REV = 4096
POSITION_THRESHOLD = 200

# Global state variables
absolute_positions = {m: 0 for m in MOTOR_IDS}
previous_encoders = {m: 0 for m in MOTOR_IDS}
target_positions = {m: 0 for m in MOTOR_IDS}
cable_lengths = {m: 0 for m in MOTOR_IDS}
initial_motor_pos = {}
initial_cable_length = {}

# PID states
pid_integrals = {m: 0 for m in MOTOR_IDS}
pid_derivatives = {m: 0 for m in MOTOR_IDS}
previous_pid_errors = {m: 0 for m in MOTOR_IDS}
pid_outputs_log = {m: [] for m in MOTOR_IDS}

# New: dictionary to store current motor speeds
motor_speeds = {m: 0 for m in MOTOR_IDS}

# PID gains
Kp = 1.0
Ki = 0.0
Kd = 0.0

# Initialize PortHandler and PacketHandler from SC Servo SDK
portHandler = PortHandler(DEVICENAME)
packetHandler = sms_sts(portHandler)

# Global Flags
manual_override = False
exit_program = False

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
    BASE_SPEED = 1500
    MIN_SPEED = 100        # 🔧 Overcome friction
    DEADZONE_ERR = 200     # 🔕 Ignore tiny errors
    FEEDFORWARD_THRESHOLD = 700

    # Step 1: Compute cable deltas (position errors)
    position_errors = {m: target_positions[m] - absolute_positions[m] for m in MOTOR_IDS}

    # Step 2: Compute max error
    max_error = max(abs(err) for err in position_errors.values())
    if max_error == 0:
        max_error = 1e-6  # Avoid zero division

    for m in MOTOR_IDS:
        err = position_errors[m]
        pid_output = 0

        if abs(err) > FEEDFORWARD_THRESHOLD:
            # 🚀 Trajectory-style: proportional to delta
            ratio = err / max_error
            speed = int(BASE_SPEED * ratio)
            # 🔧 Enforce minimum movement
            if abs(speed) < MIN_SPEED:
                speed = MIN_SPEED * np.sign(speed)
        else:
            # 🧠 PID region
            pid_integrals[m] += err * dt
            pid_derivatives[m] = (err - previous_pid_errors[m]) / dt
            previous_pid_errors[m] = err

            pid_output = Kp * err + Ki * pid_integrals[m] + Kd * pid_derivatives[m]
            speed = int(pid_output)

        # ✅ Apply deadzone
        if abs(err) < DEADZONE_ERR:
            speed = 0

        # ✅ Clamp and send
        speed = int(np.clip(speed, -BASE_SPEED, BASE_SPEED))
        write_speed(m, speed)
        motor_speeds[m] = speed

        # 🔧 Store PID raw output (not final speed)
        pid_outputs_log[m].append(speed)




def get_channel_name(motor_id):
    mapping = {
        3: "Board_3_Channel_A",
        1: "Board_1_Channel_A",
        4: "Board_4_Channel_A",
        2: "Board_2_Channel_A",
        7: "Board_6_Channel_A",
        6: "Board_5_Channel_A",
        5: "Board_5_Channel_B",
        8: "Board_6_Channel_B"
    }
    return mapping.get(motor_id, "Unknown")


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

# Starting position and rotation
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

            pos += np.array([lx, -ly, tri - sqr]) * 0.0025
            pos = np.clip(pos, [-0.43, -0.15, 0.15], [0.43, 0.15, 0.6])

            # --- Rotation ---
            euler = R.from_matrix(rot).as_euler('xyz')  # get current rotation in radians
            euler += np.array([-ry, -rx, x_b - cir]) * 0.02  # apply joystick input

            # Limit rotation
            limit_rad = np.radians(300)  
            euler = np.clip(euler, -limit_rad, limit_rad)

            rot = R.from_euler('xyz', euler).as_matrix()

        time.sleep(0.1)

def update_targets():
    lengths = calculate_cable_lengths(pos, rot)
    for i, motor_id in enumerate(MOTOR_IDS):
        if motor_id not in initial_cable_length:
            initial_cable_length[motor_id] = lengths[i]
        delta_length = lengths[i] - initial_cable_length[motor_id]
        if motor_id in []:
            delta_length *= -1
        target_positions[motor_id] = delta_length * (ENCODER_TICKS_PER_REV  / (np.pi * 0.022))

def keyboard_loop():
    global manual_override
    def getch():
        import msvcrt
        return msvcrt.getch().decode()
    key_to_motor = {'1':8,'q':8,'2':6,'w':6,'3':4,'e':4,'4':5,'r':5,
                    '5':7,'t':7,'6':11,'y':11,'7':1,'u':1,'8':2,'i':2}
    speeds = {'1':MANUAL_SPEED, 'q':-MANUAL_SPEED, '2':MANUAL_SPEED, 'w':-MANUAL_SPEED, 
              '3':MANUAL_SPEED, 'e':-MANUAL_SPEED, '4':MANUAL_SPEED, 'r':-MANUAL_SPEED,
              '5':MANUAL_SPEED, 't':-MANUAL_SPEED, '6':MANUAL_SPEED, 'y':-MANUAL_SPEED,
              '7':MANUAL_SPEED, 'u':-MANUAL_SPEED, '8':MANUAL_SPEED, 'i':-MANUAL_SPEED}
    print("Keyboard ready: ESC toggles mode, SPACE stops all motors.")
    while not exit_program:
        ch = getch()
        if ch == chr(0x1b):
            manual_override = not manual_override
            print("\n--- {} MODE ACTIVE ---\n".format("MANUAL" if manual_override else "JOYSTICK"))
        elif manual_override:
            if ch == ' ':
                for m in MOTOR_IDS:
                    write_speed(m, 0)
                print("All motors stopped manually.")
            elif ch in key_to_motor:
                motor_id, speed = key_to_motor[ch], speeds[ch]
                write_speed(motor_id, speed)
                print(f"Motor {motor_id} set to speed {speed}")

def write_pid_data():
    # Update motor speeds by reading each motor's current speed using ReadPosSpeed.
    for m in MOTOR_IDS:
        pos_val, speed_val, comm_result, error = packetHandler.ReadPosSpeed(m)
        if comm_result == COMM_SUCCESS:
            motor_speeds[m] = speed_val
        else:
            motor_speeds[m] = 0  # or leave unchanged
    data = {
        "pid_outputs": {str(m): pid_outputs_log[m][-1000:] for m in MOTOR_IDS},
        "motor_speeds": {str(m): motor_speeds[m] for m in MOTOR_IDS}
    }
    try:
        with open("pid_data.json", "w") as f:
            json.dump(data, f)
    except Exception as e:
        print("Error writing pid_data.json:", e)



def manual_position_monitor():
    global pos, manual_override, exit_program
    while not exit_program:
        if os.path.exists("desired_positions.json"):
            try:
                with open("desired_positions.json", "r") as f:
                    desired_list = json.load(f)
                
                manual_override = True
                original_pos = pos.copy()

                for desired in desired_list:
                    target_pos = np.array([
                        desired.get("x", pos[0]),
                        desired.get("y", pos[1]),
                        desired.get("z", pos[2])
                    ])
                    print(f"Moving to {target_pos}")

                    pos[:] = target_pos  # Set new target position

                    reached = False
                    stable_start_time = None

                    while not reached and not exit_program:
                        all_within_threshold = True
                        for m in MOTOR_IDS:
                            error = abs(target_positions[m] - absolute_positions[m])
                            if error > 200:  # threshold
                                all_within_threshold = False
                                break

                        if all_within_threshold:
                            if stable_start_time is None:
                                stable_start_time = time.time()
                            elif time.time() - stable_start_time >= 1.0:
                                reached = True
                        else:
                            stable_start_time = None

                        time.sleep(0.1)

                    print(f"Reached {target_pos}")

                print("All manual commands completed.")
                pos[:] = original_pos
                os.remove("desired_positions.json")
                manual_override = False

            except Exception as e:
                print("Error processing manual commands:", e)

                print(f"Manual command: moving to {pos}")
                reached = False
                while not reached and not exit_program:
                    reached = True
                    for m in MOTOR_IDS:
                        error = abs(target_positions[m] - absolute_positions[m])
                        if error > POSITION_THRESHOLD:
                            reached = False
                            break
                    if not reached:
                        time.sleep(0.05)
                print("Desired position reached. Holding for 5 seconds.")
                time.sleep(5)
                pos = original_pos
                print(f"Returning to original position: {pos}")
                os.remove("desired_position.json")
                manual_override = False
            except Exception as e:
                print("Error processing manual command:", e)
        time.sleep(0.1)

def main():
    global exit_program
    init_motors()
    threading.Thread(target=joystick_loop, daemon=True).start()
    threading.Thread(target=keyboard_loop, daemon=True).start()
    threading.Thread(target=manual_position_monitor, daemon=True).start()
    last_print = time.time()
    last_data_write = time.time()
    while not exit_program:
        # === New Reset Checking ===
        if os.path.exists("reset_command.json"):
            try:
                with open("reset_command.json", "r") as f:
                    cmd = json.load(f)
                if cmd.get("reset"):
                    pos[:] = [0.0, 0.0, 0.3]  # Live reset joystick position
                    rot[:] = np.eye(3)  # Optional: reset rotation too
                    print("\n*** Reset to (0, 0, 0.3) triggered ***\n")
                os.remove("reset_command.json")
            except Exception as e:
                print("Error handling reset command:", e)
        # === End of New Reset Checking ===

        # Your normal code
    
        update_targets()
        for m in MOTOR_IDS:
            update_absolute_position(m)
        synchronized_motor_control()
        current_time = time.time()
        if current_time - last_print >= 1.0:
            euler_deg = R.from_matrix(rot).as_euler('xyz', degrees=True)
            print(f"\nMode: {'MANUAL' if manual_override else 'JOYSTICK'}")
            print(f"Position: x={pos[0]:.3f}, y={pos[1]:.3f}, z={pos[2]:.3f} m")
            print(f"Rotation: roll={euler_deg[0]:.2f}°, pitch={euler_deg[1]:.2f}°, yaw={euler_deg[2]:.2f}°\n")
            last_print = current_time
        if current_time - last_data_write >= 0.1:
            write_pid_data()
            last_data_write = current_time
        time.sleep(0.1)
    for m in MOTOR_IDS:
        write_speed(m, 0)
    portHandler.closePort()
    pygame.quit()
    sys.exit()


if __name__ == "__main__":
    main()


