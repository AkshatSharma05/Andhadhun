# imu_subscriber.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from openal import *
import time
import math
import os
from pynput import keyboard
from threading import Thread
import pygame
import random
import math
import sys
from openal.al import alSourcei, AL_TRUE, AL_SOURCE_RELATIVE
from std_msgs.msg import Float32

pygame.mixer.init()
breathing_sound = pygame.mixer.Sound("breathing.wav")
shoot_sound = pygame.mixer.Sound("gunshot.wav")
breathing_sound.set_volume(0.3)  # Optional: adjust volume

bite_sound = pygame.mixer.Sound("bite.wav")
bite_sound.set_volume(0.5)  # Optional volume

os.environ["ALSOFT_LOGLEVEL"] = "3"
os.environ["ALSOFT_HRTF_MODE"] = "true"

# Shared variables
# angle_deg = 0
player_angle = 0
enemies = []
enemy_sources = []  # OpenAL sources linked to each enemy
running = True
last_spawn_time = 0

# Constants
WIDTH, HEIGHT = 800, 800
CENTER = (WIDTH // 2, HEIGHT // 2)
PLAYER_SIZE = 40
PLAYER_HEALTH = 100
ENEMY_SIZE = 50
SHOT_ANGLE_RANGE = 6  # Degrees
ENEMY_SPAWN_INTERVAL = 10000  # milliseconds
FPS = 60

def on_press(key):
    global player_angle, running
    try:
        # if key == keyboard.Key.left:
        #     player_angle = (player_angle - 5) % 360
        # elif key == keyboard.Key.right:
        #     player_angle = (player_angle + 5) % 360
        # elif key == keyboard.Key.esc:
        #     running = False
        pass
    except:
        pass


def input_thread():
    with keyboard.Listener(on_press=on_press) as listener:
        listener.join()

def spawn_enemy():
    if len(enemies) >= 5:
        return  # Limit to 5 enemies on screen

    angle = random.uniform(0, 360)
    rad = math.radians(angle)
    distance = 200  # Radius of the spawn circle
    x = CENTER[0] + distance * math.cos(rad)
    y = CENTER[1] - distance * math.sin(rad)

    # Append enemy and create sound
    enemies.append({
    'x': x,
    'y': y,
    'angle': angle,
    'last_bite_time': pygame.time.get_ticks()
})

    # Create and configure OpenAL sound source
    source = oalOpen("zombie_mono.wav")
    source.set_looping(True)
    source.set_gain(1.5)
    z = -5  # z-axis to simulate depth in audio
    source.set_position(((x - CENTER[0]) / 100, 0, (CENTER[1] - y) / 100))
    source.play()
    enemy_sources.append(source)

def update_enemies():
    global PLAYER_HEALTH
    BITE_INTERVAL = 4000  # ms
    STOP_DISTANCE = 60  # px
    SPEED = 0.1  # pixels per frame

    current_time = pygame.time.get_ticks()

    for i, enemy in enumerate(enemies):
        dx = CENTER[0] - enemy['x']
        dy = CENTER[1] - enemy['y']
        distance = math.hypot(dx, dy)

        if distance > STOP_DISTANCE:
            enemy['x'] += SPEED * dx / distance
            enemy['y'] += SPEED * dy / distance
            enemy_sources[i].set_position(((enemy['x'] - CENTER[0]) / 100, 0, (CENTER[1] - enemy['y']) / 100))
        else:
            # Close enough to bite
            if current_time - enemy['last_bite_time'] >= BITE_INTERVAL:
                bite_sound.play()
                PLAYER_HEALTH -= 10
                enemy['last_bite_time'] = current_time


def draw_spawn_circle(screen):
    pygame.draw.circle(screen, (50, 50, 255), CENTER, 200, 1)

# --- Pygame Drawing & Logic ---
def draw_quadrants(screen):
    pygame.draw.line(screen, (100, 100, 100), (WIDTH//2, 0), (WIDTH//2, HEIGHT), 2)
    pygame.draw.line(screen, (100, 100, 100), (0, HEIGHT//2), (WIDTH, HEIGHT//2), 2)

def draw_player(screen, angle):
    # The player angle should be synchronized with the cone's double rotation angle
    # Double the player's angle as well to match the cone's 720° rotation per unit of distance
    player_angle = angle * 2  # Align player's angle with the shooting cone's angle

    # Ensure angle is within 0-360 degrees
    player_angle = player_angle % 360

    # Convert player angle to radians for drawing
    rad = math.radians(player_angle)
    
    # Calculate player's tip and side points for triangle shape
    tip = (CENTER[0] + PLAYER_SIZE * math.cos(rad), CENTER[1] - PLAYER_SIZE * math.sin(rad))
    left = (CENTER[0] + PLAYER_SIZE * math.cos(rad + 2.5), CENTER[1] - PLAYER_SIZE * math.sin(rad + 2.5))
    right = (CENTER[0] + PLAYER_SIZE * math.cos(rad - 2.5), CENTER[1] - PLAYER_SIZE * math.sin(rad - 2.5))
    
    # Draw player (triangle shape pointing in the direction of the angle)
    pygame.draw.polygon(screen, (0, 255, 0), [tip, left, right])

def draw_shooting_cone(screen, angle):
    # Ensure angle is within 0-360 degrees
    angle = angle % 360  # This will wrap around the angle into the 0-360 range
    
    # Double the speed of rotation (i.e., two full rotations for every unit of distance)
    double_rotation_angle = angle * 2  # Cover two full rotations per unit
    
    # The start and end of the shooting cone based on the player's angle
    start_angle = (double_rotation_angle - SHOT_ANGLE_RANGE) % 360  # Wrap the start angle
    end_angle = (double_rotation_angle + SHOT_ANGLE_RANGE) % 360  # Wrap the end angle

    # Radius of the shooting guide lines
    radius = 300  # Length of the shooting guide lines

    # Draw the lines for the shooting cone
    for a in [start_angle, end_angle]:
        # Convert the angle to radians
        rad = math.radians(a)
        
        # Calculate end coordinates for the shooting cone line
        end_x = CENTER[0] + radius * math.cos(rad)
        end_y = CENTER[1] - radius * math.sin(rad)
        
        # Invert y-axis for Pygame
        pygame.draw.line(screen, (255, 255, 0), CENTER, (end_x, end_y), 2)

def pygame_thread_fn():
    global player_angle, running, last_spawn_time

    pygame.init()
    breathing_sound.play(-1)  # -1 means loop indefinitely

    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("IMU Visualization & Enemy Quadrants")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont(None, 36)

    while running:
        screen.fill((0, 0, 0))
        draw_quadrants(screen)
        draw_shooting_cone(screen, player_angle)
        draw_player(screen, player_angle)  # Sync player with cone
        draw_spawn_circle(screen)
        update_enemies()
        draw_enemies(screen)
        display_imu_data(screen, player_angle, PLAYER_HEALTH, font)

        current_time = pygame.time.get_ticks()
        if current_time - last_spawn_time >= ENEMY_SPAWN_INTERVAL:
            spawn_enemy()
            last_spawn_time = current_time

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    shoot_sound.play()
                    shoot(player_angle)

        pygame.display.flip()
        clock.tick(FPS)

    pygame.quit()


def draw_enemies(screen):
    for enemy in enemies:
        pygame.draw.rect(screen, (255, 0, 0), pygame.Rect(enemy['x'] - ENEMY_SIZE//2, enemy['y'] - ENEMY_SIZE//2, ENEMY_SIZE, ENEMY_SIZE))
def draw_shooting_cone(screen, angle):
    # Ensure angle is within 0-360 degrees
    angle = angle % 360  # This will wrap around the angle into the 0-360 range
    
    # Double the speed of rotation (i.e., two full rotations for every unit of distance)
    # This effectively means each movement will result in two full rotations (720°)
    double_rotation_angle = angle * 2  # Cover two full rotations per unit
    
    # The start and end of the shooting cone based on the player's angle
    start_angle = (double_rotation_angle - SHOT_ANGLE_RANGE) % 360  # Wrap the start angle
    end_angle = (double_rotation_angle + SHOT_ANGLE_RANGE) % 360  # Wrap the end angle

    # Radius of the shooting guide lines
    radius = 300  # Length of the shooting guide lines

    # Draw the lines for the shooting cone
    for a in [start_angle, end_angle]:
        # Convert the angle to radians
        rad = math.radians(a)
        
        # Calculate end coordinates for the shooting cone line
        end_x = CENTER[0] + radius * math.cos(rad)
        end_y = CENTER[1] - radius * math.sin(rad)
        
        # Invert y-axis for Pygame
        pygame.draw.line(screen, (255, 255, 0), CENTER, (end_x, end_y), 2)

def display_imu_data(screen, angle, PLAYER_HEALTH, font):
    text = font.render(f"IMU Angle: {angle:.2f}°", True, (255, 255, 255))
    screen.blit(text, (20, 20))

    textHealth = font.render(f"Health: {PLAYER_HEALTH}", True, (255, 255, 255))
    screen.blit(textHealth, (60, 60))

def shoot(angle):
    global enemies, enemy_sources
    new_enemies = []
    new_sources = []

    for i, enemy in enumerate(enemies):
        dx = enemy['x'] - CENTER[0]
        dy = CENTER[1] - enemy['y']  # Inverted y-axis
        distance = math.hypot(dx, dy)

        # Calculate enemy's angle relative to the player
        enemy_angle = math.degrees(math.atan2(dy, dx))
        if enemy_angle < 0:
            enemy_angle += 360
        diff = abs(enemy_angle - angle) % 360
        if diff > 180:
            diff = 360 - diff

        # Check if enemy is in the shooting cone or already close to player
        if diff <= SHOT_ANGLE_RANGE or (diff <= 45):  # allow some leniency up close
            print("Enemy hit at angle:", enemy_angle, "Distance:", distance)
            enemy_sources[i].stop()
            continue  # Don't add back to list (despawned)
        
        # Otherwise, keep enemy
        new_enemies.append(enemy)
        new_sources.append(enemy_sources[i])

    enemies = new_enemies
    enemy_sources = new_sources


# --- Pygame Thread ---
def pygame_thread_fn():
    global player_angle, running, last_spawn_time

    pygame.init()
    breathing_sound.play(-1)  # -1 means loop indefinitely

    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("IMU Visualization & Enemy Quadrants")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont(None, 36)

    while running:
        screen.fill((0, 0, 0))
        draw_quadrants(screen)
        draw_shooting_cone(screen, player_angle)
        draw_player(screen, player_angle)
        draw_spawn_circle(screen)
        update_enemies()
        draw_enemies(screen)
        display_imu_data(screen, player_angle, PLAYER_HEALTH, font)

        current_time = pygame.time.get_ticks()
        if current_time - last_spawn_time >= ENEMY_SPAWN_INTERVAL:
            spawn_enemy()
            last_spawn_time = current_time

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    shoot_sound.play()
                    shoot(player_angle)

        keys = pygame.key.get_pressed()
        # if keys[pygame.K_LEFT]:
        #     player_angle = (player_angle + 3) % 360
        # if keys[pygame.K_RIGHT]:
        #     player_angle = (player_angle - 3) % 360

        pygame.display.flip()
        clock.tick(FPS)

    pygame.quit()

# --- ROS2 Node ---
class ImuSubscriber(Node):
    def __init__(self):
        super().__init__('imu_subscriber')
        self.subscription = self.create_subscription(
            Float32,               # Change this to Int32 if needed
            '/yaw_angle',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        global player_angle
        player_angle = msg.data

# --- Main Execution ---
def main(args=None):
    global running
    rclpy.init(args=args)

    # Start keyboard input thread
    Thread(target=input_thread, daemon=True).start()

    # Start pygame thread
    Thread(target=pygame_thread_fn, daemon=True).start()

    # Start OpenAL source
    listener = Listener()
    listener.set_position((0, 0, 0))

    # ROS2 node
    imu_subscriber = ImuSubscriber()
    ros_thread = Thread(target=rclpy.spin, args=(imu_subscriber,), daemon=True)
    ros_thread.start()

    # Orientation updater
    try:
        while running:
            angle_rad = math.radians(player_angle)
            forward = (math.sin(angle_rad), 0, -math.cos(angle_rad))
            up = (0, 1, 0)
            listener.set_orientation(forward + up)
            time.sleep(0.05)
    finally:
        for src in enemy_sources:
            src.stop()
        imu_subscriber.destroy_node()
        rclpy.shutdown()
        oalQuit()
        sys.exit()

if __name__ == '__main__':
    main()
