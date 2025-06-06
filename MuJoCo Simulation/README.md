🧠 MuJoCo Simulation – 8-Cable CDPR with PID Control & Claw Mechanism
This module simulates an 8-Cable Driven Parallel Robot (CDPR) with position and orientation control using inverse kinematics and PID loops. The system also includes a claw (gripper) modeled with articulated linkages, actuated by a general tendon actuator.

Built using:

MuJoCo

mujoco-py & mujoco-python

tkinter (for GUI overlay)

NumPy & SciPy (for calculations)

GLFW (for MuJoCo rendering)

📂 File Overview
8CDPRSimulation.py
Main simulation script:

Loads and simulates the 8-cable CDPR.

Uses inverse kinematics to compute target cable lengths.

Applies PID controllers to actuators for smooth control.

Allows manual control via keyboard for:

Translation (X/Y/Z)

Orientation (Roll/Pitch/Yaw)

Claw actuation (open/close)

Real-time GUI overlay (Tkinter) showing:

Current vs desired position

Orientation angles (Euler)

Cable sensor lengths vs IK lengths

Cable force feedback

8CDPRv5.xml
MuJoCo XML model of the robot:

Simulates the full 8-cable frame structure.

Mobile platform with 8 attachment points (top/bottom faces).

Tendon-based cables connecting platform to frame corners.

Articulated gripper (claw) modeled using STL meshes.

Includes sensors:

tendonpos for cable length

framepos / framequat for body pose

Claw controlled via fingers_actuator tendon actuator.

⚙️ How to Run
Install Dependencies:

bash
Copy
Edit
pip install mujoco numpy scipy glfw
Ensure MuJoCo is properly installed:

MuJoCo Installation Guide

Update the XML path in 8CDPRSimulation.py:

python
Copy
Edit
xml_path = r'/your/path/to/8CDPRv5.xml'
Run the simulation:

bash
Copy
Edit
python 8CDPRSimulation.py
🎮 Controls
Action	Key
Move X+ / X−	D / A
Move Y+ / Y−	W / S
Move Z+ / Z−	E / Q
Roll ±	U / O
Pitch ±	I / K
Yaw ±	J / L
Claw Close / Open	R / T
Reset to Center	GUI Button

🧠 Features
Inverse Kinematics: Computes cable lengths based on desired position and orientation.

PID Control: Smooth actuation via adjustable proportional–integral–derivative controllers.

Real-time Visualization:

Cable length comparison (Sensor vs IK)

Euler angles (roll/pitch/yaw)

Cable tension forces

Claw Integration:

Bi-directional finger actuator (fingers_actuator) using split tendon configuration.

Visual 4-bar linkage modeled with STL geometry and equality constraints.

🛠 Developer Notes
Model Tuning: The PID gains, tendon stiffness, and damping can be adjusted in the XML or Python controller.

Cable Naming Convention:

Cables 1–4: Upper anchors to lower platform

Cables 5–8: Lower anchors to upper platform

STL Files: Gripper geometries referenced in 8CDPRv5.xml must be placed in the /assets/ folder.

🧑‍💻 Contributors
Simulation Development: [8CDPRSimulation.py]

MuJoCo Model Design: [8CDPRv5.xml]

Gripper Mechanism Modeling: [Claw Module Team]


