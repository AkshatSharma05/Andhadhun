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

# Optional: Enable HRTF and debugging (before OpenAL loads)
os.environ["ALSOFT_LOGLEVEL"] = "3"
os.environ["ALSOFT_HRTF_MODE"] = "true"

# Shared state
angle_deg = 0
running = True

# --- Input handling functions ---
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

# --- ROS2 Node ---
class ImuSubscriber(Node):
    def __init__(self):
        super().__init__('imu_subscriber')
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg: Imu):
        self.get_logger().info(
            f'Orientation -> x: {msg.orientation.x:.3f}, '
            f'y: {msg.orientation.y:.3f}, '
            f'z: {msg.orientation.z:.3f}, '
            f'w: {msg.orientation.w:.3f}'
        )

# --- Main Execution ---
def main(args=None):
    global running
    rclpy.init(args=args)

    # Start input thread
    Thread(target=input_thread, daemon=True).start()

    # Start OpenAL sound source
    source = oalOpen("zombie_mono.wav")
    source.set_looping(True)
    listener = Listener()
    listener.set_position((0, 0, 0))
    source.set_position((0, 0, -5))
    source.play()

    # ROS2 subscriber node
    imu_subscriber = ImuSubscriber()

    try:
        # Spin in a separate thread
        ros_thread = Thread(target=rclpy.spin, args=(imu_subscriber,), daemon=True)
        ros_thread.start()

        # Orientation control loop
        while running:
            angle_rad = math.radians(angle_deg)
            forward = (math.sin(angle_rad), 0, -math.cos(angle_rad))
            up = (0, 1, 0)
            listener.set_orientation(forward + up)
            # print(f"Orientation set to: forward={forward}, angle={angle_deg}")
            time.sleep(0.1)

    finally:
        imu_subscriber.destroy_node()
        rclpy.shutdown()
        oalQuit()

if __name__ == '__main__':
    main()
