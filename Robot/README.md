
## 🧠 Original Purpose

### 🔹 `Main_Code/`
- Developed for:
  - Forward and inverse kinematics
  - Trajectory planning
  - PID control for cable tension or end-effector position

- May have used:
  - `numpy` and `sympy` for matrix math
  - `matplotlib` for trajectory plots
  - CSV log files or serial interface to communicate with hardware

### 🔹 `GUI/`
- Created to visualize:
  - Real-time cable lengths and tensions
  - End-effector coordinates in 2D/3D space
  - User interactions via buttons/sliders

- Likely built using:
  - `Tkinter`, `PyQt`, or `matplotlib` for UI/plotting
  - ROS/serial integration to show live data from sensors or control systems

### 🔹 `Force_Reader/`
- Interfaced with load cells or tension sensors
- Included:
  - Serial reading
  - Signal smoothing/filtering
  - Real-time monitoring graphs
- May have output data to file or GUI panel for logging and analysis

---

## 🔗 Refactoring Status

- These components may now be distributed across:
  - `Claw/` – for actuator-level control
  - `tests/` – for simulation and validation
  - `Asset/` – for SDKs, external modules, or motor interfaces

To avoid duplication and improve modularity, robot motion logic may have been consolidated under a ROS node or higher-level controller.

---

## 🛠️ Developer Notes

- Consider versioning refactored logic and tagging older `Main_Code` commits for reference.
- Preserve mathematical models and simulation plots for documentation or training.
- Archive or remove unused code if it's no longer actively maintained to keep the repo clean.

---

*Last updated: June 5, 2025*
