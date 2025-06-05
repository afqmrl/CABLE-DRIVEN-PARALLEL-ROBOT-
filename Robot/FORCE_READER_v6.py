#!/usr/bin/env python3
import RPi.GPIO as GPIO
import time
import json
import os
import numpy as np
import traceback

print("▶︎ Script starting. Working dir:", os.getcwd())

# ————— Configuration —————

HISTORY_LENGTH = 100
FILTER_ALPHA   = 1.0  # 1 = no smoothing, <1 = exponential filter

calibration_factors = np.array([
    0.005915401, 0.005695078, 0.005452689, 0.00591456,
    0.005812267, 0.029105374, 0.005115862, 0.022304154
])

HX711_PINS = [
    {'dout':11, 'pd_sck':22},  # board 1
    {'dout':13, 'pd_sck':22},  # board 2
    {'dout':15, 'pd_sck':22},  # board 3
    {'dout':16, 'pd_sck':22},  # board 4
    
]

class HX711:
    def __init__(self, dout, pd_sck):
        self.DOUT   = dout
        self.PD_SCK = pd_sck
        GPIO.setup(self.PD_SCK, GPIO.OUT)
        GPIO.setup(self.DOUT,   GPIO.IN)
        self.set_gain(128)

    def set_gain(self, gain):
        if gain == 128:
            self._gain_pulses = 1
        elif gain == 32:
            self._gain_pulses = 2
        else:
            raise ValueError("Unsupported gain")
        # apply once to latch
        self.read_raw()

    def read_raw(self):
        # wait until data ready (DOUT goes low)
        start = time.time()
        while GPIO.input(self.DOUT) == 1:
            if time.time() - start > 1.0:
                raise TimeoutError(f"DOUT pin {self.DOUT} never went low")
            time.sleep(0.001)
        count = 0
        for _ in range(24):
            GPIO.output(self.PD_SCK, True)
            count = (count << 1) | GPIO.input(self.DOUT)
            GPIO.output(self.PD_SCK, False)
        for _ in range(self._gain_pulses):
            GPIO.output(self.PD_SCK, True)
            GPIO.output(self.PD_SCK, False)
        if count & 0x800000:
            count |= ~0xffffff
        return count

# ————— Initialization —————

try:
    GPIO.setmode(GPIO.BOARD)
    hx_modules = [HX711(p['dout'], p['pd_sck']) for p in HX711_PINS]
except Exception:
    print("⚠️ GPIO setup failed (are you running as sudo?)")
    traceback.print_exc()
    exit(1)

# Load initial tensions
initial_tensions = np.zeros(8)
if os.path.exists("initial_tensions.json"):
    try:
        initial_tensions = np.array(json.load(open("initial_tensions.json")), float)
        print("▶︎ Loaded initial tensions:", initial_tensions)
    except Exception as e:
        print("⚠️ Failed to load initial_tensions.json:", e)

# Load or init history
history = {}
if os.path.exists("force_data.json"):
    try:
        history = json.load(open("force_data.json"))
        print("▶︎ Pre-existing history keys:", list(history.keys()))
    except Exception:
        history = {}

zero_offsets  = np.zeros(8)
filtered_vals = None
first_sample  = True

print("▶︎ Entering main loop (Ctrl-C to exit)")

# ————— Main Loop —————
try:
    iteration = 0
    while True:
        iteration += 1
        print(f"\n--- Loop #{iteration} ---")
        # Read each board & its channels
        # Boards 1 & 4 & 5 have both A&B; boards 2 & 3 only A.
        channel_map = [
            ("a1","b1"),   # board 1
            ("a2","b2"),       # board 2
            ("a3","b3"),       # board 3
            ("a4","b4"),   # board 4
        ]

        raw = []
        keys = []
        for board_idx, channels in enumerate(channel_map):
            module = hx_modules[board_idx]
            for ch in channels:
                # channel A needs gain=128, channel B gain=32
                if ch.startswith("a"):
                    module.set_gain(128)
                else:
                    module.set_gain(32)

                val = module.read_raw()
                raw.append(val)
                keys.append(ch)
                #print(f"raw {ch} = {val}")

        raw = np.array(raw, float)

        if first_sample:
            zero_offsets  = raw.copy()
            filtered_vals = initial_tensions.copy()
            first_sample  = False
            #print("✅ Zero offsets:", zero_offsets)

        corrected  = raw - zero_offsets
        calibrated = corrected * calibration_factors + initial_tensions
        filtered_vals = (1 - FILTER_ALPHA)*filtered_vals + FILTER_ALPHA*calibrated

        #print("→ Filtered (g):", np.round(filtered_vals,3))

        # record history
        for k,v in zip(keys, filtered_vals):
            history.setdefault(k, []).append(float(v))
            if len(history[k]) > HISTORY_LENGTH:
                history[k] = history[k][-HISTORY_LENGTH:]

        # write JSON
        with open("force_data.json","w") as f:
            json.dump(history, f)
        #print("▶︎ Written force_data.json (", sum(len(v) for v in history.values()), "total values )")

        time.sleep(0.5)  # slower so you can read output

except KeyboardInterrupt:
    print("Stopping by user.")
except Exception:
    print("❗ Unexpected error in main loop:")
    traceback.print_exc()
finally:
    GPIO.cleanup()
    print("GPIO cleaned up, exiting.")
