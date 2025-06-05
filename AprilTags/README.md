
---

## 📝 Script Overview

### `Apriltags6.py`
- **Main script for AprilTag detection**
- Detects AprilTags in a live camera feed using OpenCV and the AprilTag library
- Prints tag IDs and poses to the console
- Displays video output with tag overlays

### `calibration.py`
- **Performs camera calibration**
- Loads checkerboard images captured via `getImages.py`
- Generates and saves camera matrix and distortion coefficients
- Required before running `Apriltags6.py`

### `getImages.py`
- **Captures checkerboard images for calibration**
- Saves multiple images from a camera feed
- User moves the checkerboard to different angles/distances
- Outputs are used by `calibration.py`

---

## ⚙️ How to Use

1. **Activate the Python virtual environment**  
   ```bash
   source ~/Desktop/CDPR/Environment/bin/activate

