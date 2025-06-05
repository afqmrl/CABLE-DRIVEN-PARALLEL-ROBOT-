# Cable-Driven Parallel Robot (GROUP 7)

![cdpr](https://github.com/user-attachments/assets/b988b011-0617-4479-93ce-cf66e94fe6da)

**A modular framework** for controlling a cable-driven parallel robot, with EEG integration and an end-effector claw.

## 📂 Project Structure

- **Asset/**  
  - `assets/`           ← Placeholder directory for general assets (e.g., images, models)  
  - `scservo_sdk/`      ← SDK files for SCServo, ready for integration  
  - `sms_sts/`          ← For SMS-based servo  
  - `saved_commands/`   ← Pre-recorded command files for predefined trajectories  
  > **Note:** Each subfolder under **Asset/** contains a `.gitkeep` so Git can track an otherwise empty directory. 

- **Robot/**  
  - `Main_Code/`        – Main control of the robot
  - `GUI/`              – Visualization & user interface  
  - `Force_Reader/`     – Sensor data acquisition  

- **EEG/**  
  - `Main/`             – Raw & processed EEG `.mat` files  
  - `processing/`       – Filtering, artifact removal scripts  
  - `Simulated/`        – Simulated EEG signal generator for Raspberry Pi  

- **Claw/**  
  - `control/`          – Claw actuation logic (motor commands, state management)  
  - `utils/`            – Claw subsystem utility functions  

- **docs/**             – Design docs & hardware specs  
- **tests/**            – Unit & integration tests  
- **README.md**         – This file  

## 🚀 Getting Started

1. Clone the repo  
   ```bash
   git clone https://github.com/afqmrl/CABLE-DRIVEN-PARALLEL-ROBOT-.git


## Group Member
- **Avishai Wynne**
- **Amirul Afiq**
- **Jai Bolton**
- **Sadikul Islam**
- **Khairul Anuar**

