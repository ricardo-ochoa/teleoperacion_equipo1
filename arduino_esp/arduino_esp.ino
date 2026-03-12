#include <Arduino.h>
#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <sensor_msgs/msg/joint_state.h>
#include <std_msgs/msg/float32.h>

//////////////////////////////////////////////////////
// PIN CONFIG
//////////////////////////////////////////////////////

#define IN1 26
#define IN2 25
#define ENA 27

#define IN3 14
#define IN4 12
#define ENB 13

#define ENC_L_A 35
#define ENC_L_B 34
#define ENC_R_A 33
#define ENC_R_B 32

//////////////////////////////////////////////////////
// PWM
//////////////////////////////////////////////////////

#define PWM_FREQ 20000
#define PWM_RES 10
#define PWM_MAX ((1 << PWM_RES) - 1)

#define PWM_CH_L 0
#define PWM_CH_R 1

//////////////////////////////////////////////////////
// Encoder variables
//////////////////////////////////////////////////////

volatile long ticksL = 0;
volatile long ticksR = 0;

long lastTicksL = 0;
long lastTicksR = 0;

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

//////////////////////////////////////////////////////
// Motor parameters
//////////////////////////////////////////////////////

#define PPR 11
#define GEAR_RATIO 34

//////////////////////////////////////////////////////
// PID gains
//////////////////////////////////////////////////////

float Kp = 8.0;
float Ki = 20.0;
float Kd = 0.02;

float eL = 0, eL1 = 0, eL2 = 0;
float eR = 0, eR1 = 0, eR2 = 0;

float uL = 0;
float uR = 0;

//////////////////////////////////////////////////////
// ROS
//////////////////////////////////////////////////////

rcl_node_t node;
rcl_subscription_t sub_joint;
rcl_publisher_t pub_left;
rcl_publisher_t pub_right;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;

sensor_msgs__msg__JointState joint_msg;
std_msgs__msg__Float32 velL_msg;
std_msgs__msg__Float32 velR_msg;

//////////////////////////////////////////////////////
// Control variables
//////////////////////////////////////////////////////

float setpointL_rpm = 0.0;
float setpointR_rpm = 0.0;

unsigned long lastSample = 0;
#define SAMPLE_MS 100
#define SAMPLE_S 0.1

//////////////////////////////////////////////////////
// Encoder ISR
//////////////////////////////////////////////////////

void IRAM_ATTR encLA() {
  int b = digitalRead(ENC_L_B);
  portENTER_CRITICAL_ISR(&mux);
  ticksL += (b ? -1 : 1);
  portEXIT_CRITICAL_ISR(&mux);
}

void IRAM_ATTR encRA() {
  int b = digitalRead(ENC_R_B);
  portENTER_CRITICAL_ISR(&mux);
  ticksR += (b ? -1 : 1);
  portEXIT_CRITICAL_ISR(&mux);
}

//////////////////////////////////////////////////////
// Helper
//////////////////////////////////////////////////////

float radps_to_rpm(float w_rad_s) {
  return w_rad_s * 60.0 / (2.0 * PI);
}

int find_joint_index(const sensor_msgs__msg__JointState *msg, const char *target) {
  for (size_t i = 0; i < msg->name.size; i++) {
    if (msg->name.data[i].data != NULL && strcmp(msg->name.data[i].data, target) == 0) {
      return (int)i;
    }
  }
  return -1;
}

//////////////////////////////////////////////////////
// JOINT STATES callback
//////////////////////////////////////////////////////

void joint_callback(const void *msgin) {
  const sensor_msgs__msg__JointState *msg =
      (const sensor_msgs__msg__JointState *)msgin;

  // Change these names to EXACTLY match your Gazebo joint names
  int idxL = find_joint_index(msg, "left_wheel_joint");
  int idxR = find_joint_index(msg, "right_wheel_joint");

  if (idxL >= 0 && idxL < (int)msg->velocity.size) {
    setpointL_rpm = radps_to_rpm(msg->velocity.data[idxL]);
  }

  if (idxR >= 0 && idxR < (int)msg->velocity.size) {
    setpointR_rpm = radps_to_rpm(msg->velocity.data[idxR]);
  }

}

//////////////////////////////////////////////////////
// PID incremental
//////////////////////////////////////////////////////

float pid_update(float &u, float e, float e1, float e2) {
  u += (Kp + Kd / SAMPLE_S) * e;
  u += (-Kp + Ki * SAMPLE_S - 2.0 * Kd / SAMPLE_S) * e1;
  u += (Kd / SAMPLE_S) * e2;

  u = constrain(u, -PWM_MAX, PWM_MAX);
  return u;
}

//////////////////////////////////////////////////////
// Motor drive
//////////////////////////////////////////////////////

void setMotorL(float u) {
  int duty = (int)constrain(fabs(u), 0, PWM_MAX);

  if (u > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else if (u < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }

  ledcWrite(PWM_CH_L, duty);
}

void setMotorR(float u) {
  int duty = (int)constrain(fabs(u), 0, PWM_MAX);

  if (u > 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else if (u < 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }

  ledcWrite(PWM_CH_R, duty);
}

//////////////////////////////////////////////////////
// SETUP
//////////////////////////////////////////////////////

void setup() {
  Serial.begin(115200);
  delay(2000);

  set_microros_transports();

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "esp32_diffdrive", "", &support);

  rclc_subscription_init_default(
      &sub_joint,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
      "/joint_states");

  rclc_publisher_init_default(
      &pub_left,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "/wheel_left_measured_rpm");

  rclc_publisher_init_default(
      &pub_right,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "/wheel_right_measured_rpm");

  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(
      &executor, &sub_joint, &joint_msg, &joint_callback, ON_NEW_DATA);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  ledcSetup(PWM_CH_L, PWM_FREQ, PWM_RES);
  ledcAttachPin(ENA, PWM_CH_L);

  ledcSetup(PWM_CH_R, PWM_FREQ, PWM_RES);
  ledcAttachPin(ENB, PWM_CH_R);

  // GPIO34 and GPIO35: INPUT only, no internal pullup
  pinMode(ENC_L_A, INPUT);
  pinMode(ENC_L_B, INPUT);
  pinMode(ENC_R_A, INPUT_PULLUP);
  pinMode(ENC_R_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_L_A), encLA, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), encRA, RISING);
}

//////////////////////////////////////////////////////
// LOOP
//////////////////////////////////////////////////////

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

  if (millis() - lastSample < SAMPLE_MS) return;
  lastSample = millis();

  long tL, tR;
  portENTER_CRITICAL(&mux);
  tL = ticksL;
  tR = ticksR;
  portEXIT_CRITICAL(&mux);

  long dL = tL - lastTicksL;
  long dR = tR - lastTicksR;

  lastTicksL = tL;
  lastTicksR = tR;

  float rpmL = (dL / (float)(PPR * GEAR_RATIO)) / SAMPLE_S * 60.0;
  float rpmR = (dR / (float)(PPR * GEAR_RATIO)) / SAMPLE_S * 60.0;

  eL2 = eL1;
  eL1 = eL;
  eL = setpointL_rpm - rpmL;

  eR2 = eR1;
  eR1 = eR;
  eR = setpointR_rpm - rpmR;

  float outL = pid_update(uL, eL, eL1, eL2);
  float outR = pid_update(uR, eR, eR1, eR2);

  setMotorL(outL);
  setMotorR(outR);

  velL_msg.data = rpmL;
  velR_msg.data = rpmR;

  rcl_publish(&pub_left, &velL_msg, NULL);
  rcl_publish(&pub_right, &velR_msg, NULL);
}