
---

## 🧠 Module Descriptions

### 🔹 `data/`
- Contains EEG recordings categorized by cognitive state (e.g., `Relax_*.mat`, `Arithmetic_*.mat`, `Stroop_*.mat`)
- Typically exported from OpenBCI GUI or EEG capture software
- Used as input for model training or real-time streaming simulation

### 🔹 `processing/`
- Scripts to clean raw EEG data:
  - Bandpass filtering
  - Artifact rejection (e.g., eye blinks, EMG noise)
  - Downsampling or epoch segmentation
- Can include feature extraction methods like:
  - FFT / PSD (power spectral density)
  - Bandpower ratio (alpha/beta, etc.)

### 🔹 `Simulated/EEGSignalRPi.py`
- Simulates EEG output by sending a PWM signal from a Raspberry Pi
- Selects a random `.mat` file representing either a relaxed or stressed state
- Normalizes signal amplitude and converts it into a GPIO PWM duty cycle (~250 Hz)
- Useful for testing integration with motors or feedback systems without needing real-time EEG hardware

---

## 🧪 Example: Run Simulated EEG on Raspberry Pi

1. **Ensure you’re in the correct Python virtual environment (if used)**:
   ```bash
   source ~/Desktop/CDPR/Environment/bin/activate
