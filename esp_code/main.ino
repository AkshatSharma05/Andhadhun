#include <Wire.h>
#include <MPU6050.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>


MPU6050 mpu;
rcl_publisher_t yaw_publisher;
std_msgs__msg__Float32 yaw_msg;
rcl_allocator_t allocator;
rclc_executor_t executor;
rcl_node_t node;
rclc_support_t support;

unsigned long lastTime;
float yaw = 0;  // Initialize yaw
int16_t gx, gy, gz;

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  Wire.begin();
  
  // Initialize MPU6050 sensor
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while (1);
  }
  Serial.println("MPU6050 ready.");

  // Initialize micro-ROS transports
set_microros_wifi_transports("akshat", "aryanaryan", "192.168.234.152", 8888);

  // Initialize the micro-ROS system
  allocator = rcl_get_default_allocator();

  // Initialize micro-ROS support
  rclc_support_init(&support, 0, NULL, &allocator);

  // Initialize node
  rclc_node_init_default(&node, "imu_node", "", &support);

  // Initialize publisher for yaw
  rclc_publisher_init_default(
    &yaw_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "yaw_angle");

  // Initialize executor
  rclc_executor_init(&executor, &support.context, 1, &allocator);

  lastTime = millis();
}

void loop() {
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;  // Time in seconds
  lastTime = currentTime;

  // Get gyro data
  mpu.getRotation(&gx, &gy, &gz);
  
  // Your original yaw calculation
  float gyroZdeg = (float)gz / 131.0;  // Convert raw to degrees per second
  yaw += gyroZdeg * dt;  // Integrate to get yaw

  // Normalize to 0-360 degrees
  if (yaw < 0) yaw += 360;
  if (yaw > 360) yaw -= 360;

  // Send the yaw value via micro-ROS
  yaw_msg.data = yaw;
  rcl_publish(&yaw_publisher, &yaw_msg, NULL);

  // Print yaw to Serial Monitor for debugging
  Serial.print("Yaw: ");
  Serial.println(yaw);

  // Spin executor to handle other tasks
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}
