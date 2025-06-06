import os
os.sched_setaffinity(0, {3})
import subprocess
import mujoco as mj
from mujoco.glfw import glfw
import numpy as np
import tkinter as tk
from tkinter import ttk
import time
from scipy.spatial.transform import Rotation as R  # Import for quaternion conversion



# Initialize GLFW and create the window
glfw.init()
window = glfw.create_window(1200, 900, "Cable Control with PID Adjustment GUI", None, None)
glfw.make_context_current(window)
glfw.swap_interval(1)

# Define PID Controller
class PIDController:
    def __init__(self, kp, ki, kd, setpoint):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.setpoint = setpoint
        self.prev_error = 0
        self.integral = 0

    def update(self, measurement):
        error = self.setpoint - measurement
        self.integral += error
        derivative = error - self.prev_error
        self.prev_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative

# Load MuJoCo model
xml_path = r'/home/group7/Desktop/CDPR/FINAL/8CDPR.4.xml'
model = mj.MjModel.from_xml_path(xml_path)
data = mj.MjData(model)
cam, opt = mj.MjvCamera(), mj.MjvOption()
scene, context = mj.MjvScene(model, maxgeom=10000), mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150.value)

# Global variables for orientation (Euler angles in radians)
desired_roll = 0.0
desired_pitch = 0.0
desired_yaw = 0.0

# Update the desired_rotation accordingly:
desired_rotation = R.from_euler('xyz', [desired_roll, desired_pitch, desired_yaw]).as_matrix()

# Motor positions (world frame) for 8 cables
motor_positions = np.array([
    [-0.5, -0.23, 0.6],  # Motor 1 (bottom left corner)
    [0.5, -0.23, 0.6],   # Motor 2 (bottom right corner)
    [-0.5, 0.23, 0.6],   # Motor 3 (top left corner)
    [0.5, 0.23, 0.6],    # Motor 4 (top right corner)
    [-0.5, -0.23, 0.0],    # Motor 5 (middle left)
    [0.5, -0.23, 0.0],     # Motor 6 (middle right)
    [-0.5, 0.23, 0.0],   # Motor 7 (bottom center)
    [0.5, 0.23, 0.0]     # Motor 8 (top center)
])

# Platform anchor points (local frame) for 8 cables
platform_anchors = np.array([
    [-0.025, -0.025,  0.025],  # Intended as top1 (top face)
    [ 0.025, -0.025,  0.025],  # Intended as top2 (top face)
    [-0.025,  0.025,  0.025],  # Intended as top3 (top face)
    [ 0.025,  0.025,  0.025],  # Intended as top4 (top face)
    [-0.025, -0.025, 0.025],  # Intended as bottom1 (bottom face)
    [ 0.025, -0.025, 0.025],  # Intended as bottom2 (bottom face)
    [-0.025,  0.025, 0.025],  # Intended as bottom3 (bottom face)
    [ 0.025,  0.025, 0.025]   # Intended as bottom4 (bottom face)
])



# Inverse kinematics to compute desired cable lengths
def calculate_cable_lengths(position, rotation):
    cable_lengths = []
    for motor, anchor in zip(motor_positions, platform_anchors):
        world_anchor = position + np.dot(rotation, anchor)
        length = np.linalg.norm(world_anchor - motor)
        cable_lengths.append(length)
    return cable_lengths

# Desired initial position and orientation - Adjusted to center the platform in the cage.
# (Even though the geometry might not be perfectly symmetric, we force uniform cable setpoints when centered.)
initial_position = np.array([0.0, 0.0, 0.3])  # Center in x,y and adjusted z
desired_position = initial_position.copy()


# Compute cable lengths from inverse kinematics at the desired center position
ik_lengths_initial = calculate_cable_lengths(desired_position, desired_rotation)
# When centered, force all cables to have the same setpoint by taking the average.
avg_length = np.mean(ik_lengths_initial)
print("IK-calculated cable lengths at center:", ik_lengths_initial)
print("Average cable length =", avg_length)

# Initialize PID controllers for each cable (8 cables) using the average length for the initial state.
pid_controllers = [PIDController(1.0, 0.01, 0.3, avg_length) for _ in range(8)]

# Adjust cable lengths based on inverse kinematics.
# For the initial (center) state, we force all setpoints to be equal.
# When moving, we update each cable’s setpoint based on the IK calculation.


def adjust_cables_for_target(target_position, target_rotation):
    new_lengths = calculate_cable_lengths(target_position, target_rotation)
    # If the platform is at the initial (center) position, force all setpoints to be the average.
    if np.allclose(target_position, initial_position):
        setpoints = [np.mean(new_lengths)] * 8
    else:
        setpoints = new_lengths
    for pid, sp in zip(pid_controllers, setpoints):
        pid.setpoint = sp

# Set GLFW callbacks for keyboard control (which will update the setpoints via IK if moved)
def keyboard(window, key, scancode, action, mods):
    global desired_position, desired_rotation, desired_roll, desired_pitch, desired_yaw
    angle_step = 0.3  # Increment (in radians) for orientation changes.
    pos_step = 0.01   # Increment (in meters) for position changes.
    
    if action == glfw.PRESS:
        # --- Position Controls ---
        if key == glfw.KEY_Q:
            desired_position[2] -= pos_step  # Move up (decrease z)
        elif key == glfw.KEY_E:
            desired_position[2] += pos_step  # Move down (increase z)
        elif key == glfw.KEY_A:
            desired_position[0] -= pos_step  # Move left (decrease x)
        elif key == glfw.KEY_D:
            desired_position[0] += pos_step  # Move right (increase x)
        elif key == glfw.KEY_W:
            desired_position[1] += pos_step  # Move forward (increase y)
        elif key == glfw.KEY_S:
            desired_position[1] -= pos_step  # Move backward (decrease y)


        # --- Orientation Controls ---
        # Roll (rotation about x-axis): U increases roll, O decreases roll.
        elif key == glfw.KEY_U:
            desired_roll += angle_step  
        elif key == glfw.KEY_O:
            desired_roll -= angle_step

        # Pitch (rotation about y-axis): I increases pitch, K decreases pitch.
        elif key == glfw.KEY_I:
            desired_pitch += angle_step  
        elif key == glfw.KEY_K:
            desired_pitch -= angle_step

        # Yaw (rotation about z-axis): J increases yaw, L decreases yaw.
        elif key == glfw.KEY_J:
            desired_yaw += angle_step  
        elif key == glfw.KEY_L:
            desired_yaw -= angle_step
        

        claw_idx = mj.mj_name2id(model, mj.mjtObj.mjOBJ_ACTUATOR, "fingers_actuator")
        # print("Claw actuator index:", claw_idx)
        # if L2 is pressed, open the claw (command low value).
        if key == glfw.KEY_R:
            data.ctrl[claw_idx] = 2550  # Fully closed
        elif key == glfw.KEY_T:
            data.ctrl[claw_idx] = -1000    # Fully open

        
        # Clamp the position values to their limits:
        # x between -0.50 m and 0.50 m,
        # y between -0.23 m and 0.23 m,
        # z between 0.0 m and 0.60 m.
        desired_position[0] = np.clip(desired_position[0], -0.50, 0.50)
        desired_position[1] = np.clip(desired_position[1], -0.23, 0.23)
        desired_position[2] = np.clip(desired_position[2], 0.0, 0.60)
        
        # Clamp orientation angles to -90° to 90° (in radians: -pi/2 to pi/2)
        desired_roll  = np.clip(desired_roll,  -np.pi/4, np.pi/4)
        desired_pitch = np.clip(desired_pitch, -np.pi/4, np.pi/4)
        desired_yaw   = np.clip(desired_yaw,   -np.pi/4, np.pi/4)
        
        # Update the desired rotation matrix based on the new Euler angles.
        desired_rotation = R.from_euler('xyz', [desired_roll, desired_pitch, desired_yaw]).as_matrix()
        
        # Update cable setpoints based on the new desired position and orientation.
        adjust_cables_for_target(desired_position, desired_rotation)

        
    elif action == glfw.RELEASE:
        pass  # No action on key release




glfw.set_key_callback(window, keyboard)

# Mouse interaction variables
lastx, lasty = 0, 0
button_left, button_middle, button_right = False, False, False

def mouse_button(window, button, action, mods):
    global button_left, button_middle, button_right
    if action == glfw.PRESS:
        if button == glfw.MOUSE_BUTTON_LEFT:
            button_left = True
        elif button == glfw.MOUSE_BUTTON_MIDDLE:
            button_middle = True
        elif button == glfw.MOUSE_BUTTON_RIGHT:
            button_right = True
    elif action == glfw.RELEASE:
        if button == glfw.MOUSE_BUTTON_LEFT:
            button_left = False
        elif button == glfw.MOUSE_BUTTON_MIDDLE:
            button_middle = False
        elif button == glfw.MOUSE_BUTTON_RIGHT:
            button_right = False

def mouse_move(window, xpos, ypos):
    global lastx, lasty, button_left, button_middle, button_right
    dx, dy = xpos - lastx, ypos - lasty
    lastx, lasty = xpos, ypos

    if button_left:
        # Rotate the camera
        mj.mjv_moveCamera(model, mj.mjtMouse.mjMOUSE_ROTATE_H, dx / 100, dy / 100, scene, cam)
    elif button_middle:
        # Pan the camera
        mj.mjv_moveCamera(model, mj.mjtMouse.mjMOUSE_PAN, dx / 100, dy / 100, scene, cam)
    elif button_right:
        # Zoom the camera
        mj.mjv_moveCamera(model, mj.mjtMouse.mjMOUSE_ZOOM, 0, dy / 100, scene, cam)

def scroll(window, xoffset, yoffset):
    # Zoom in/out
    mj.mjv_moveCamera(model, mj.mjtMouse.mjMOUSE_ZOOM, 0, -0.05 * yoffset, scene, cam)

# Set mouse-related GLFW callbacks
glfw.set_mouse_button_callback(window, mouse_button)
glfw.set_cursor_pos_callback(window, mouse_move)
glfw.set_scroll_callback(window, scroll)

# Controller function to update actuator inputs based on the PID adjustments
def controller(model, data):
    global previous_target_lengths

    current_lengths = data.ten_length[:8]  # Get current cable lengths
    target_lengths = calculate_cable_lengths(desired_position, desired_rotation)  # IK computed lengths

    alpha = 0.02  # Setpoint smoothing factor (Lower = Smoother motion)
    max_length_change = 0.005  # Limit the max length change per step

    # Apply smoothing to prevent sudden jumps in PID target values
    for i in range(4):
        delta_length = target_lengths[i] - pid_controllers[i].setpoint
        delta_length = np.clip(delta_length, -max_length_change, max_length_change)  # Limit change
        pid_controllers[i].setpoint += alpha * delta_length  # Gradual setpoint update

    # Compute PID output adjustments
    adjustments = [pid.update(length) for pid, length in zip(pid_controllers, current_lengths)]

    force_limit = 100.0  # Prevent excessive force application
    velocity_damping = 0.2  # Damping factor to smooth velocity

    # Apply force limits and ensure only pulling forces (No compression)
    for i, adjustment in enumerate(adjustments):
        force_output = np.clip(adjustment, 0, force_limit)  # No negative forces
        # data.ctrl[i] = velocity_damping * force_output  # Dampen excessive forces
        data.ctrl[i] = target_lengths[i]  # Dampen excessive forces

mj.set_mjcb_control(controller)



# Simulation update loop
def update_simulation():
    if not glfw.window_should_close(window):
        time_prev = data.time
        while data.time - time_prev < 1.0 / 60.0:
            mj.mj_step(model, data)
            # Force platform orientation to identity (zero yaw, pitch, roll)
            # data.qpos[3:7] = np.array([1, 0, 0, 0])
        # Render scene
        viewport = mj.MjrRect(0, 0, *glfw.get_framebuffer_size(window))
        mj.mjv_updateScene(model, data, opt, None, cam, mj.mjtCatBit.mjCAT_ALL.value, scene)
        mj.mjr_render(viewport, scene, context)

        glfw.swap_buffers(window)
        glfw.poll_events()

        root.after(10, update_simulation)
    else:
        glfw.terminate()
        root.destroy()

# Tkinter GUI setup
root = tk.Tk()
root.title("Cable Control Simulation")

# GUI: Labels for current mobile platform position
label_position_x = tk.Label(root, text="Current Position X: N/A")
label_position_y = tk.Label(root, text="Current Position Y: N/A")
label_position_z = tk.Label(root, text="Current Position Z: N/A")

label_position_x.pack()
label_position_y.pack()
label_position_z.pack()

# GUI: Labels for desired position
label_desired_x = tk.Label(root, text="Desired Position X: N/A")
label_desired_y = tk.Label(root, text="Desired Position Y: N/A")
label_desired_z = tk.Label(root, text="Desired Position Z: N/A")

label_desired_x.pack()
label_desired_y.pack()
label_desired_z.pack()

# GUI: Labels for pitch, yaw and roll angles
label_pitch = tk.Label(root, text="Pitch: N/A")
label_yaw = tk.Label(root, text="Yaw: N/A")
label_roll = tk.Label(root, text="Roll: N/A")

label_pitch.pack()
label_yaw.pack()
label_roll.pack()

# GUI: Labels for cable lengths (from sensors and IK calculations) for 8 cables
labels_sensor_lengths = [tk.Label(root, text=f"Cable {i+1} Sensor Length: N/A") for i in range(8)]
labels_ik_lengths = [tk.Label(root, text=f"Cable {i+1} IK Length: N/A") for i in range(8)]

for sensor_label, ik_label in zip(labels_sensor_lengths, labels_ik_lengths):
    sensor_label.pack()
    ik_label.pack()

# **NEW** Labels for Cable Forces
labels_force = [tk.Label(root, text=f"Cable {i+1} Force: N/A") for i in range(8)]
for label in labels_force:
    label.pack()

# --- Added Reset Button and Function ---
def reset_simulation():
    global desired_position, desired_rotation, desired_roll, desired_pitch, desired_yaw
    # Reset the desired state to initial (center) values.
    desired_position = initial_position.copy()
    
    # Reset Euler angles (roll, pitch, yaw) to zero.
    desired_roll = 0.0
    desired_pitch = 0.0
    desired_yaw = 0.0
    
    # Update the desired rotation matrix from the Euler angles.
    desired_rotation = R.from_euler('xyz', [desired_roll, desired_pitch, desired_yaw]).as_matrix()
    
    # Reset simulation data to initial state.
    mj.mj_resetData(model, data)
    
    # Set the mobile platform state:
    data.qpos[:3] = desired_position
    # Set the orientation to the identity quaternion.
    data.qpos[3:7] = np.array([1, 0, 0, 0])
    
    # Force all cable setpoints to the computed average cable length.
    for pid in pid_controllers:
        pid.setpoint = avg_length


    

reset_button = tk.Button(root, text="Reset to Center Position", command=reset_simulation)
reset_button.pack()
# --- End Added Reset Button ---

label_euler = tk.Label(root, text="Roll, Pitch, Yaw: 0.0, 0.0, 0.0")
label_euler.pack()

# Function to update GUI with real-time information
def update_gui():


    
    cube_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "cube")
    print("Claw actuator index:", cube_id)
    
    # Get the current position of the mobile platform (assumes data.qpos gives platform position)
    platform_position = data.qpos[:3]
    label_position_x.config(text=f"Current Position X: {platform_position[0]:.3f} m")
    label_position_y.config(text=f"Current Position Y: {platform_position[1]:.3f} m")
    label_position_z.config(text=f"Current Position Z: {platform_position[2]:.3f} m")

    # Update desired position
    label_desired_x.config(text=f"Desired Position X: {desired_position[0]:.3f} m")
    label_desired_y.config(text=f"Desired Position Y: {desired_position[1]:.3f} m")
    label_desired_z.config(text=f"Desired Position Z: {desired_position[2]:.3f} m")



    # Update platform orientation (pitch, yaw, roll)
    quat = data.qpos[3:7]  # Mujoco free joint quaternion (w, x, y, z)
    # Convert to SciPy quaternion format (x, y, z, w)
    r = R.from_quat([quat[1], quat[2], quat[3], quat[0]])
    roll, pitch, yaw = r.as_euler('xyz', degrees=True)
    label_pitch.config(text=f"Pitch: {pitch:.3f}°")
    label_yaw.config(text=f"Yaw: {yaw:.3f}°")
    label_roll.config(text=f"Roll: {roll:.3f}°")

    # Get current cable lengths from sensors for 8 cables
    sensor_lengths = data.ten_length[:8]
    for i, length in enumerate(sensor_lengths):
        labels_sensor_lengths[i].config(text=f"Cable {i+1} Sensor Length: {length:.3f} m")

    # Compute cable lengths from inverse kinematics for 8 cables
    ik_lengths = calculate_cable_lengths(desired_position, desired_rotation)
    for i, length in enumerate(ik_lengths):
        labels_ik_lengths[i].config(text=f"Cable {i+1} IK Length: {length:.3f} m")

    # Get forces from actuators
    sensor_forces = data.actuator_force[:8]  # Get forces from actuators

    for i, (length, force) in enumerate(zip(sensor_lengths, sensor_forces)):
        labels_force[i].config(text=f"Cable {i+1} Force: {force:.3f} N")

    # Display the current Euler angles from our global variables:
    label_euler.config(text=f"Roll, Pitch, Yaw: {desired_roll:.3f}, {desired_pitch:.3f}, {desired_yaw:.3f}")

    # Schedule the next update
    root.after(100, update_gui)

# Set initial cable setpoints to the computed average length
for pid in pid_controllers:
    pid.setpoint = avg_length

# Start updating the GUI and simulation
update_gui()
update_simulation()
root.mainloop()
