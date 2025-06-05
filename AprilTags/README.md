
---

## 🧠 Script Descriptions

### 🔹 `Apriltags6.py`
- **Primary detection script**  
- Detects AprilTags in real-time from a camera feed using OpenCV and AprilTag libraries.
- Visualizes tag boundaries and prints tag ID and pose data (position & orientation).
- Requires prior camera calibration data.

### 🔹 `calibration.py`
- **Performs camera calibration**  
- Loads checkerboard images from a local folder.
- Calculates the camera matrix and distortion coefficients.
- Saves the output for use with AprilTag detection.

### 🔹 `getImages.py`
- **Captures calibration images**  
- Launches a live feed from the camera.
- Allows the user to capture multiple images of a checkerboard from various angles.
- These images are used by `calibration.py`.

---

## ⚙️ Setup & Usage

> ✅ **Step-by-step instructions for running the full vision system**

### 1. Activate the Virtual Environment
```bash
source ~/Desktop/CDPR/Environment/bin/activate
