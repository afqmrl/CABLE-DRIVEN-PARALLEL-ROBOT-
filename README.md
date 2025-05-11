# Cable-Driven Parallel Robot

**A modular framework** for controlling a cable-driven parallel robot, with EEG integration and an end-effector claw.

## 📂 Project Structure

- **robot/**  
  - `control/` – motion planning & kinematics  
  - `gui/` – visualization & user interface  
  - `force_reader/` – sensor data acquisition  
- **eeg/**  
  - `data/` – raw & processed signals  
  - `processing/` – filtering, artifact removal  
  - `utils/` – EEG helper functions  
- **claw/**  
  - `control/` – claw actuation logic  
  - `utils/` – claw utilities  
- **docs/** – design docs & hardware specs  
- **tests/** – unit & integration tests  

## 🚀 Getting Started

1. Clone the repo  
   ```bash
   git clone https://github.com/afqmrl/CABLE-DRIVEN-PARALLEL-ROBOT-.git
   cd CABLE-DRIVEN-PARALLEL-ROBOT-
