from openal import *
import time
import math
import os
from pynput import keyboard
from threading import Thread

# Optional: Enable HRTF and debugging (before OpenAL loads)
os.environ["ALSOFT_LOGLEVEL"] = "3"
os.environ["ALSOFT_HRTF_MODE"] = "true"

# Shared state
angle_deg = 0
running = True

def on_press(key):
    global angle_deg, running
    try:
        if key == keyboard.Key.left:
            angle_deg -= 5
        elif key == keyboard.Key.right:
            angle_deg += 5
        elif key == keyboard.Key.esc:
            running = False
    except:
        pass

def input_thread():
    with keyboard.Listener(on_press=on_press) as listener:
        listener.join()

try:
    Thread(target=input_thread, daemon=True).start()

    source = oalOpen("zombie_mono.wav")
    source.set_looping(True)  # 🔁 Enable looping
    listener = Listener()
    listener.set_position((0, 0, 0))
    source.set_position((0, 0, -5))
    source.play()

    while running:
        angle_rad = math.radians(angle_deg)
        forward = (math.sin(angle_rad), 0, -math.cos(angle_rad))
        up = (0, 1, 0)
        listener.set_orientation(forward + up)
        print(f"Orientation set to: forward={forward}")
        time.sleep(0.1)
        print(angle_deg)
    time.sleep(1)

finally:
    oalQuit()
