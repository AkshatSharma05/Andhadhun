import random
import math
import time
import os
from pynput import keyboard
from threading import Thread
from openal import *
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

# Globals
angle_deg = 0
running = True
auto_mode = False
enemies = []

# Parameters
enemy_radius = 10  # Radius around the listener for enemies
hitbox_angle = 10  # Hitbox angle range for shooting (degrees)

# Helper function to create random enemy positions in one of the 4 quadrants
def generate_enemy():
    # Randomly pick quadrant (0: top-right, 1: top-left, 2: bottom-right, 3: bottom-left)
    quadrant = random.randint(0, 3)
    distance = random.uniform(1.0, enemy_radius)  # Random distance within the radius
    angle = random.uniform(0, math.pi / 2)  # Random angle within quadrant

    if quadrant == 0:  # top-right
        x = distance * math.cos(angle)
        z = -distance * math.sin(angle)
    elif quadrant == 1:  # top-left
        x = -distance * math.cos(angle)
        z = -distance * math.sin(angle)
    elif quadrant == 2:  # bottom-right
        x = distance * math.cos(angle)
        z = distance * math.sin(angle)
    elif quadrant == 3:  # bottom-left
        x = -distance * math.cos(angle)
        z = distance * math.sin(angle)

    return (x, 0, z)

# Listener function (keyboard control for shooting)
def on_press(key):
    global angle_deg, running, auto_mode
    try:
        if key == keyboard.Key.left:
            angle_deg -= 5
        elif key == keyboard.Key.right:
            angle_deg += 5
        elif key == keyboard.Key.esc:
            running = False
        elif key == keyboard.Key.space:
            shoot_at_enemy()
    except:
        pass

def input_thread():
    with keyboard.Listener(on_press=on_press) as listener:
        listener.join()

Thread(target=input_thread, daemon=True).start()

# Load OpenAL sound
oalInit()
main_source = oalOpen("zombie_mono.wav")
main_source.set_looping(True)
main_source.set_position((0, 0, -5))
listener = Listener()
listener.set_position((0, 0, 0))
main_source.play()

# Function to spawn new enemies randomly
def spawn_enemy():
    global enemies
    x, y, z = generate_enemy()
    source = oalOpen("enemy_sound.wav")  # Replace with actual enemy sound
    source.set_position((x, y, z))
    source.play()
    enemies.append(source)  # Store the enemy sound source for later

# Function to shoot at enemies
def shoot_at_enemy():
    global enemies
    # Calculate listener direction (orientation)
    angle_rad = math.radians(angle_deg)
    listener_direction = (math.sin(angle_rad), 0, -math.cos(angle_rad))

    for enemy in enemies[:]:
        # Get enemy's position
        enemy_position = enemy.get_position()
        dx = enemy_position[0] - listener.get_position()[0]
        dz = enemy_position[2] - listener.get_position()[2]

        # Calculate angle to enemy (relative to listener)
        enemy_angle = math.degrees(math.atan2(dz, dx))
        delta_angle = abs(angle_deg - enemy_angle)

        # If the enemy is within the hitbox range, "shoot" it
        if delta_angle <= hitbox_angle:
            print(f"Shot enemy at position {enemy_position}, angle {enemy_angle}°")
            enemy.stop()  # Stop the enemy sound (simulate "death")
            enemies.remove(enemy)  # Remove enemy from list

# Main loop to control random enemy spawning
try:
    while running:
        if not auto_mode:
            # Manual orientation adjustment
            angle_rad = math.radians(angle_deg)
            forward = (math.sin(angle_rad), 0, -math.cos(angle_rad))
            up = (0, 1, 0)
            listener.set_orientation(forward + up)
            print(f"Orientation set to: forward={forward}")
            time.sleep(0.1)
        else:
            # Auto mode - spawn enemies randomly
            if random.random() < 0.1:  # 10% chance to spawn an enemy each loop
                spawn_enemy()
            time.sleep(random.uniform(2.0, 5.0))  # Wait between 2-5 seconds before spawning another enemy

    time.sleep(1)
finally:
    oalQuit()