# sender_pwm_pi.py
import time
import numpy as np
import random
import glob
import scipy.io
import RPi.GPIO as GPIO

# --- PWM Setup ---
PWM_PIN = 18  # Use GPIO18 (supports hardware PWM)
FREQ = 1000   # 1kHz PWM, suitable for RC filtering

GPIO.setmode(GPIO.BCM)
GPIO.setup(PWM_PIN, GPIO.OUT)
pwm = GPIO.PWM(PWM_PIN, FREQ)
pwm.start(0)  # Start at 0% duty cycle

# --- Dataset Loading ---
RELAXED_FILES = glob.glob("Relax_*.mat")
STRESSED_FILES = glob.glob("Arithmetic_*.mat") + glob.glob("Stroop_*.mat")

def load_random_signal(label):
    files = RELAXED_FILES if label == "relaxed" else STRESSED_FILES
    chosen = random.choice(files)
    mat = scipy.io.loadmat(chosen)
    key = [k for k in mat if not k.startswith("__")][0]
    signal = mat[key][0][:1000]  # Take 4s at ~250Hz
    norm = (signal - np.min(signal)) / (np.max(signal) - np.min(signal))  # 0 to 1
    return norm

print("PWM EEG Signal Started ...")

try:
    while True:
        label = random.choice(["relaxed", "stressed"])
        signal = load_random_signal(label)
        print(f"Now sending: {label.upper()}")

        for val in signal:
            duty = val * 100  # Scale 0–1 to 0–100%
            pwm.ChangeDutyCycle(duty)
            time.sleep(0.004)  # Simulate 250 Hz sample rate

except KeyboardInterrupt:
    print("Stopping PWM...")
    pwm.stop()
    GPIO.cleanup()
