// === REPLACEMENT VERSION WITH PID & LEDC ===
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <tf2_msgs/msg/tf_message.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <rosidl_runtime_c/string_functions.h>

#define WHEEL_RADIUS 0.04
#define WHEEL_BASE 0.33
#define TICKS_PER_REV 104
#define PI 3.14159265358979323846

// === Motor Pins ===
const int motor1PinA = 26, motor1PinB = 27;
const int motor2PinA = 18, motor2PinB = 19;

// === Encoder Pins ===
const int encoder1PinA = 4, encoder1PinB = 2;
const int encoder2PinA = 33, encoder2PinB = 32;

// === Encoder State ===
volatile int left_ticks = 0, right_ticks = 0;
float x = 0.0, y = 0.0, theta = 0.0;
unsigned long last_time;

// === PID parameters ===
float kp = 1.0, ki = 0.01, kd = 0.3;
float l_error = 0, r_error = 0;
float l_integral = 0, r_integral = 0;
float l_prev_error = 0, r_prev_error = 0;
const int BASE_PWM = 150;

// === ROS2 ===
rcl_subscription_t cmd_vel_sub;
rcl_publisher_t odom_pub;
rcl_publisher_t tf_pub;
tf2_msgs__msg__TFMessage tf_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
geometry_msgs__msg__Twist cmd_vel_msg;
nav_msgs__msg__Odometry odom_msg;
geometry_msgs__msg__TransformStamped transform_msg;

// === ISRs ===
void IRAM_ATTR left_encoder_ISR() { left_ticks++; }
void IRAM_ATTR right_encoder_ISR() { right_ticks++; }

// === Motor Control using LEDC ===
void set_motor_pwm(int channelA, int channelB, int speed) {
  speed = constrain(speed, -255, 255);
  if (speed >= 0) {
    ledcWrite(channelA, speed);
    ledcWrite(channelB, 0);
  } else {
    ledcWrite(channelA, 0);
    ledcWrite(channelB, -speed);
  }
}

void set_motor_speeds(int left_pwm, int right_pwm) {
  set_motor_pwm(0, 1, left_pwm);
  set_motor_pwm(2, 3, right_pwm);
}

// === PID Control ===
int compute_pid(float target_ticks, float actual_ticks, float &prev_error, float &integral) {
  float error = target_ticks - actual_ticks;
  integral += error;
  float derivative = error - prev_error;
  prev_error = error;
  return (int)(kp * error + ki * integral + kd * derivative);
}

// === ROS cmd_vel Callback ===
void cmd_vel_callback(const void *msg_in) {
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msg_in;
  float v = msg->linear.x;
  float omega = msg->angular.z;

  float v_R = v + (omega * WHEEL_BASE / 2);
  float v_L = v - (omega * WHEEL_BASE / 2);

  float revs_L = v_L / (2 * PI * WHEEL_RADIUS);
  float revs_R = v_R / (2 * PI * WHEEL_RADIUS);

  int target_L = revs_L * TICKS_PER_REV;
  int target_R = revs_R * TICKS_PER_REV;

  int pwm_L = BASE_PWM + compute_pid(target_L, left_ticks, l_prev_error, l_integral);
  int pwm_R = BASE_PWM + compute_pid(target_R, right_ticks, r_prev_error, r_integral);

  set_motor_speeds(pwm_L, pwm_R);
}

// === Odometry Publisher ===
void publish_odometry() {
  unsigned long now = micros();
  float dt = (now - last_time) / 1e6;
  last_time = now;

  float omega_L = (left_ticks / (float)TICKS_PER_REV) * 2 * PI / dt;
  float omega_R = (right_ticks / (float)TICKS_PER_REV) * 2 * PI / dt;
  float v_L = WHEEL_RADIUS * omega_L;
  float v_R = WHEEL_RADIUS * omega_R;

  float v = (v_R + v_L) / 2.0;
  float omega = (v_R - v_L) / WHEEL_BASE;

  x += v * cos(theta) * dt;
  y += v * sin(theta) * dt;
  theta += omega * dt;

  left_ticks = 0;
  right_ticks = 0;

  odom_msg.header.stamp.sec = now / 1000000;
  odom_msg.header.stamp.nanosec = (now % 1000000) * 1000;
  rosidl_runtime_c__String__assign(&odom_msg.header.frame_id, "odom");
  rosidl_runtime_c__String__assign(&odom_msg.child_frame_id, "base_link");

  odom_msg.pose.pose.position.x = x;
  odom_msg.pose.pose.position.y = y;
  odom_msg.pose.pose.orientation.z = sin(theta / 2.0);
  odom_msg.pose.pose.orientation.w = cos(theta / 2.0);
  odom_msg.twist.twist.linear.x = v;
  odom_msg.twist.twist.angular.z = omega;

  rcl_publish(&odom_pub, &odom_msg, NULL);
}

// === TF Publisher ===
void publish_transform() {
  unsigned long now = micros();

  geometry_msgs__msg__TransformStamped transform;
  geometry_msgs__msg__TransformStamped__init(&transform);

  transform.header.stamp.sec = now / 1000000;
  transform.header.stamp.nanosec = (now % 1000000) * 1000;
  rosidl_runtime_c__String__assign(&transform.header.frame_id, "odom");
  rosidl_runtime_c__String__assign(&transform.child_frame_id, "base_footprint");

  transform.transform.translation.x = x;
  transform.transform.translation.y = y;
  transform.transform.translation.z = 0.0;
  transform.transform.rotation.x = 0.0;
  transform.transform.rotation.y = 0.0;
  transform.transform.rotation.z = sin(theta / 2.0);
  transform.transform.rotation.w = cos(theta / 2.0);

  tf2_msgs__msg__TFMessage__init(&tf_msg);
  geometry_msgs__msg__TransformStamped__Sequence__init(&tf_msg.transforms, 1);
  tf_msg.transforms.data[0] = transform;

  rcl_publish(&tf_pub, &tf_msg, NULL);
}

// === Setup ===
void setup() {
  set_microros_transports();
  delay(2000);

  pinMode(motor1PinA, OUTPUT);
  pinMode(motor1PinB, OUTPUT);
  pinMode(motor2PinA, OUTPUT);
  pinMode(motor2PinB, OUTPUT);

  ledcSetup(0, 500, 8); ledcAttachPin(motor1PinA, 0);
  ledcSetup(1, 500, 8); ledcAttachPin(motor1PinB, 1);
  ledcSetup(2, 500, 8); ledcAttachPin(motor2PinA, 2);
  ledcSetup(3, 500, 8); ledcAttachPin(motor2PinB, 3);

  pinMode(encoder1PinA, INPUT_PULLUP);
  pinMode(encoder1PinB, INPUT_PULLUP);
  pinMode(encoder2PinA, INPUT_PULLUP);
  pinMode(encoder2PinB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encoder1PinA), left_encoder_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(encoder2PinA), right_encoder_ISR, RISING);

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "esp32_node", "", &support);

  rclc_subscription_init_default(&cmd_vel_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel");

  rclc_publisher_init_default(&odom_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "odom");

  rclc_publisher_init_default(&tf_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage), "tf");

  rclc_executor_init(&executor, &support.context, 2, &allocator);
  rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg,
    &cmd_vel_callback, ON_NEW_DATA);

  last_time = micros();
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
  publish_odometry();
  publish_transform();
  delay(100);
}
