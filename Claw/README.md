# Claw Module

This folder contains all logic and utilities related to controlling the claw (end-effector) of the Cable-Driven Parallel Robot. The claw is responsible for grasping and manipulating objects and is controlled through software and hardware integration.

---

## ⚙️ Overview

### 🔹 `control/`
This subdirectory holds the primary scripts for:
- Sending actuation commands to the claw motor (e.g., open/close, grip strength)
- Interpreting feedback from encoders or sensors
- Managing control states (e.g., idle, grasping, releasing)

The control logic may interface with microcontrollers (e.g., Arduino, Raspberry Pi) or use direct GPIO/PWM commands.

### 🔹 `utils/`
This folder contains helper functions and modules such as:
- I/O configuration scripts (e.g., pin setup, serial communication)
- Error handling or safety checks
- Reusable functions for sensor input or signal normalization

These scripts support the `control/` logic and help modularize code for clarity and reuse.

---

## ✅ Example Use Case

> Here’s a general example of how your claw code might be used in a live control system:

```python
from control.claw_controller import ClawController

claw = ClawController()
claw.open()          # Send open command
claw.close(force=80) # Close with 80% motor power
claw.stop()          # Emergency stop
