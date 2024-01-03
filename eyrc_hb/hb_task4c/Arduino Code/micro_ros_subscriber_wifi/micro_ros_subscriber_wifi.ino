#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/wrench.h>
#include <std_msgs/msg/int32.h>

#include <ESP32Servo.h>


rcl_subscription_t rear_wheel_subscriber;
rcl_subscription_t left_wheel_subscriber;
rcl_subscription_t right_wheel_subscriber;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

geometry_msgs__msg__Wrench rear;
geometry_msgs__msg__Wrench left;
geometry_msgs__msg__Wrench right;

#define LED_PIN 2
#define BOT_ID 3

#define rear_servo_pin 25
#define left_servo_pin 26
#define right_servo_pin 27

Servo rear_servo;
Servo left_servo;
Servo right_servo;


#define RCCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { error_loop(); } \
  }
#define RCSOFTCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) {} \
  }


void error_loop() {
  while (1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    Serial.println("Error Occured");
    delay(100);
  }
}

void rear_wheel_callback(const void *msgin) {
  const geometry_msgs__msg__Wrench *msg = (const geometry_msgs__msg__Wrench *)msgin;
  double speed = msg->force.y;

  Serial.println("Rear Wheel Force" + String(speed));
  
  rear_servo.write(int(speed));
}

void left_wheel_callback(const void *msgin) {
  const geometry_msgs__msg__Wrench *msg = (const geometry_msgs__msg__Wrench *)msgin;
  double speed = msg->force.y;

  Serial.println("Left Wheel Force" + String(speed));

  left_servo.write(int(speed));
}

void right_wheel_callback(const void *msgin) {
  const geometry_msgs__msg__Wrench *msg = (const geometry_msgs__msg__Wrench *)msgin;
  double speed = msg->force.y;

  Serial.println("Right Wheel Force" + String(speed));

  right_servo.write(int(speed));
}

void apply_force(Servo servoo, int speed) {
  // servoo.write(speed);
  Serial.println("in apply force");
  // rear_servo.write(180);
  // rear_servo.write(int(speed));
}

void init_servos() {
  rear_servo.attach(rear_servo_pin);
  left_servo.attach(left_servo_pin);
  right_servo.attach(right_servo_pin);
}

void setup() {
  Serial.begin(115200);
  while (!Serial)
    ;
  Serial.println("Start");
  set_microros_wifi_transports("testt", "123456789", "192.168.91.27", 8888);
  Serial.println("wifi connected");
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  init_servos();

  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));


  String node_name = "subscriber_node" + String(BOT_ID);
  // create node
  RCCHECK(rclc_node_init_default(&node, node_name.c_str(), "", &support));

  //topic names
  String rear_topic = "/hb_bot_" + String(BOT_ID) + "/rear_wheel_force";
  String left_topic = "/hb_bot_" + String(BOT_ID) + "/left_wheel_force";
  String right_topic = "/hb_bot_" + String(BOT_ID) + "/right_wheel_force";

  Serial.println(rear_topic);
  Serial.println(left_topic);
  Serial.println(right_topic);

  //create subscribers
  RCCHECK(rclc_subscription_init_best_effort(
    &rear_wheel_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Wrench),
    rear_topic.c_str()));

  RCCHECK(rclc_subscription_init_best_effort(
    &left_wheel_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Wrench),
    left_topic.c_str()));

  RCCHECK(rclc_subscription_init_best_effort(
    &right_wheel_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Wrench),
    right_topic.c_str()));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));

  //add subscriptions
  RCCHECK(rclc_executor_add_subscription(&executor, &rear_wheel_subscriber, &rear, &rear_wheel_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &left_wheel_subscriber, &left, &left_wheel_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &right_wheel_subscriber, &right, &right_wheel_callback, ON_NEW_DATA));
}

void loop() {
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
