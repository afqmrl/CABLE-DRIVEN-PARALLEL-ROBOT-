# receiver_pi.py
import time
from datetime import datetime
import numpy as np
from brainflow.board_shim import BoardShim, BrainFlowInputParams, BoardIds
from brainflow.data_filter import DataFilter, FilterTypes, WindowFunctions

# --- BrainFlow Setup ---
params = BrainFlowInputParams()
params.serial_port = ''  # Leave empty for default serial Bluetooth
board = BoardShim(BoardIds.CYTON_BOARD.value, params)

# --- Signal Processing Parameters ---
CHANNEL_INDEX = 0  # OpenBCI channel index (usually 0 for first)
ALPHA_BAND = [8.0, 13.0]
BETA_BAND = [13.0, 30.0]
RATIO_THRESHOLD = 1.2

# --- Init ---
board.prepare_session()
board.start_stream()
print("Starting EEG stress detection...\n")

try:
    while True:
        time.sleep(1)  # Every second
        data = board.get_current_board_data(256)  # 1 second at 250 Hz

        eeg_channel = BoardShim.get_eeg_channels(BoardIds.CYTON_BOARD.value)[CHANNEL_INDEX]
        signal = data[eeg_channel]

        # Apply bandpass filtering for cleaner PSD
        DataFilter.perform_bandpass(signal, 250, 1.0, 50.0, 4, FilterTypes.BUTTERWORTH.value, 0)

        # Welch PSD
        psd, freqs = DataFilter.get_psd_welch(signal, 256, 128, 250,
                                              WindowFunctions.HANNING.value)

        # Band powers
        alpha = DataFilter.get_band_power(psd, freqs, ALPHA_BAND[0], ALPHA_BAND[1])
        beta = DataFilter.get_band_power(psd, freqs, BETA_BAND[0], BETA_BAND[1])

        ratio = beta / alpha if alpha != 0 else 0
        timestamp = datetime.now().strftime("[%H:%M:%S]")

        if ratio > RATIO_THRESHOLD:
            print(f"{timestamp} Stress event detected | Beta/Alpha Ratio: {ratio:.2f}")
        else:
            print(f"{timestamp} Normal state          | Beta/Alpha Ratio: {ratio:.2f}")

except KeyboardInterrupt:
    print("\nEEG Stress Detection interrupted.")
finally:
    board.stop_stream()
    board.release_session()
    print("\nEEG session ended.")
