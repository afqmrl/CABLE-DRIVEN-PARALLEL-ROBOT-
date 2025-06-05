# Cable-Driven Parallel Robot (GROUP 7)

![cdpr](https://github.com/user-attachments/assets/b988b011-0617-4479-93ce-cf66e94fe6da)

**A modular framework** for controlling a cable-driven parallel robot, with EEG integration and an end-effector claw.

## 📂 Project Structure

- **Asset/**  
  - `assets/`           ← Placeholder directory for general assets (e.g., images, models)  
  - `scservo_sdk/`      ← SDK files for Dynamixel SCServo, ready for integration  
  - `sms_sts/`          ← Scripts or data for SMS-based subsystem testing  
  - `saved_commands/`   ← Pre-recorded or canned command files for replay/testing  
  > **Note:** Each subfolder under **Asset/** contains a `.gitkeep` so Git can track an otherwise empty directory. Replace or delete the `.gitkeep` once you add real content.

- **Robot/**  
  - `Main_Code/`        – (Refactored/DELETED) motion planning & kinematics  
  - `GUI/`              – (Refactored/DELETED) visualization & user interface  
  - `Force_Reader/`     – (Refactored/DELETED) sensor data acquisition  

- **EEG/**  
  - `data/`             – Raw & processed EEG `.mat` files  
  - `processing/`       – Filtering, artifact removal scripts  
  - `utils/`            – EEG helper functions  
  - `Simulated/`        – (Since June 5, 2025) Simulated EEG signal generator for Raspberry Pi  

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

