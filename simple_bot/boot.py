import os
import board
import digitalio
import storage

# --- USB VBUS LOGIC (Commented out for manual testing) ---
# vbus = digitalio.DigitalInOut(board.GP24)
# vbus.direction = digitalio.Direction.INPUT
# storage.remount("/", readonly=vbus.value)

# --- UNCONDITIONAL WRITE ACCESS (Uncommented for testing) ---
storage.remount("/", readonly=False)

# Delete previous log file after boot up
try:
    os.remove("/log.txt")
except OSError:
    pass
