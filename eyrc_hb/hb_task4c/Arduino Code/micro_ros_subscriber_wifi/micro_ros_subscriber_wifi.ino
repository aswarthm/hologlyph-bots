// '''
// *****************************************************************************************
// *
// *        		===============================================
// *           		Hologlyph Bots (HB) Theme (eYRC 2023-24)
// *        		===============================================
// *
// *  This script is to implement Task 2B of Hologlyph Bots (HB) Theme (eYRC 2023-24).
// *
// *  This software is made available on an "AS IS WHERE IS BASIS".
// *  Licensee/end user indemnifies and will keep e-Yantra indemnified from
// *  any and all claim(s) that emanate from the use of the Software or
// *  breach of the terms of this agreement.
// *
// *****************************************************************************************
// '''

/*
 * Team Id: hb_1036
 * Author List: [ M Aswartha Reddy, D K Bharath Reddy, Pulkit Dhamija, Sangeeta Prasad ]
 * Filename: micro_ros_subscriber_wifi.ino
 * Theme: <Theme name -- Specific to eYRC>
 * Functions: [error_loop(),velocity_callback(const void *msgin), pen_down_callback(const void *msgin),init_servos(), setup(), loop()]
 * Nodes:		           Subs: [ 'hb_bot_{self.bot_id}/goal', '/detected_aruco_{self.bot_id}' ]
                       Pubs: [ "/hb_bot_{self.bot_id}/rear_wheel_force", "/hb_bot_{self.bot_id}/left_wheel_force", "/hb_bot_{self.bot_id}/right_wheel_force" ]
 * Global Variables: None
 */

// ################### IMPORT MODULES #######################
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/vector3.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/bool.h>

#include <ESP32Servo.h>


rcl_subscription_t velocity_subscriber;
rcl_subscription_t pen_down_subscriber;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

geometry_msgs__msg__Vector3 velocity;
std_msgs__msg__Bool pen_down;


#define LED_PIN 2
#define BOT_ID 2

#define rear_servo_pin 25
#define left_servo_pin 26
#define right_servo_pin 27
#define pen_down_servo_pin 33

int servo_angles[3][2] = { // [servo id][0-up angle, 1-down angle]
                        {50, 25}, 
                        {50, 24},
                        {50, 23},
};

Servo rear_servo;
Servo left_servo;
Servo right_servo;
Servo pen_down_servo;


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

/*
 * Function Name: error_loop
 * Input: None
 * Output: None
 * Logic: Toggles the LED infinitely if any error occurs
 * Example Call: error_loop()
 */
void error_loop() {
  while (1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    Serial.println("Error Occured");
    delay(100);
  }
}
/*
 * Function Name: velocity_callback
 * Input: msgin--> input vector with x,y,z components
 * Output: None
 * Logic: extracts x,y,z components and stores the rear,left,right velocity
 * Example Call: valocity_callback()
 */
void velocity_callback(const void *msgin) {
  const geometry_msgs__msg__Vector3 *msg = (const geometry_msgs__msg__Vector3 *)msgin;
  double rear_vel = msg->x;
  double left_vel = msg->y;
  double right_vel = msg->z;

  Serial.println("Rear Wheel Velocity" + String(rear_vel));
  Serial.println("Left Wheel Velocity" + String(left_vel));
  Serial.println("Right Wheel Velocity" + String(right_vel));

  rear_servo.write(int(rear_vel));
  left_servo.write(int(left_vel));
  right_servo.write(int(right_vel));
  
  // rear_servo.write(int(speed));
}
/*
 * Function Name: velocity_callback
 * Input: msgin--> input vector with x,y,z components
 * Output: None
 * Logic: extracts x,y,z components and stores the rear,left,right velocity
 * Example Call: valocity_callback()
 */
void pen_down_callback(const void *msgin){
  const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msgin;

  if(msg->data == 0){ 
    //penup
    pen_down_servo.write(servo_angles[BOT_ID-1][0]);
    Serial.println("Pen Up");
  }
  else{
    //pendown
    pen_down_servo.write(servo_angles[BOT_ID-1][1]);
    Serial.println("Pen Down");
  }
}

/*
 * Function Name: init_servos
 * Input: msgin--> None
 * Output: None
 * Logic: initialises the servos to the pins specified and also initialises pen_down servo
 * Example Call: init_servos()
 */
void init_servos() {
  rear_servo.attach(rear_servo_pin);
  left_servo.attach(left_servo_pin);
  right_servo.attach(right_servo_pin);

  pen_down_servo.attach(pen_down_servo_pin);

  pen_down_servo.write(50);
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  // while (!Serial)
  //   ;
  Serial.println("Start");
  set_microros_wifi_transports("AstraLAN", "12345678", "192.168.0.104", 8888);
  Serial.println("wifi connected");
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  init_servos();

  // delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));


  String node_name = "subscriber_node" + String(BOT_ID);
  // create node
  RCCHECK(rclc_node_init_default(&node, node_name.c_str(), "", &support));

  //topic names
  String velocity_topic = "/hb_bot_" + String(BOT_ID) + "/cmd_vell";
  String pen_down_topic = "/pen" + String(BOT_ID) + "_down";

  Serial.println(velocity_topic);

  //create subscribers
  RCCHECK(rclc_subscription_init_best_effort(
    &velocity_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
    velocity_topic.c_str()));

  RCCHECK(rclc_subscription_init_best_effort(
    &pen_down_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    pen_down_topic.c_str()));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));

  //add subscriptions
  RCCHECK(rclc_executor_add_subscription(&executor, &velocity_subscriber, &velocity, &velocity_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &pen_down_subscriber, &pen_down, &pen_down_callback, ON_NEW_DATA));
}

void loop() {
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
