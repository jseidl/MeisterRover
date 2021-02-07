#include <Arduino.h>

#ifdef ROS_ENABLE
#include "ros.h"
#include "geometry_msgs/Twist.h"
#endif

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

/* Constants */
// RX PINS
#define RX_PIN_COUNT 4
#define RX_CH1_PIN 7
#define RX_CH2_PIN 8
#define RX_CH3_PIN 9
#define RX_CH4_PIN 10

#define RX_THROTTLE RX_CH2_PIN
#define RX_STEERING RX_CH1_PIN
#define RX_ESTOP RX_CH4_PIN

// Logical Values
#define VIRT_MIN -255
#define VIRT_MAX 255
#define VIRT_MID VIRT_MIN+VIRT_MAX

// Servo Values
#define SERVO_MIN 900
#define SERVO_MID 1500
#define SERVO_MAX 2100
#define SERVO_FREQ 50

// RX Values
#define RX_MIN 1052
#define RX_MID 1470
#define RX_MAX 1880
#define RX_DEADZONE 5
#define RX_DEADZONE_LOW RX_MID-RX_DEADZONE
#define RX_DEADZONE_HIGH RX_MID+RX_DEADZONE

// Servo Numbers
#define SERVO_THROTTLE 15
#define SERVO_STEERING 14

// General
#define STARTUP_DELAY 5000
#define LOOP_TIME 10

#define THROTTLE_IDLE 0.0
#define STEERING_IDLE 0.0

// Limits
#define SPEED_LIMIT 70
#define SPEED_LIMIT_REV -110
#define ANGLE_MAX 255.0
#define ANGLE_MIN -255.0

/* Globals */
bool eStop = false;
const byte RX_PINS[] = {RX_CH1_PIN, RX_CH2_PIN, RX_CH3_PIN, RX_CH4_PIN};

// Timers for the sub-main loop
unsigned long currentMillis;
long previousMillis = 0;    // set up timers

// cmd_vel variables
float demand_x;
float demand_z;

// PWM Driver
Adafruit_PWMServoDriver pwm_board = Adafruit_PWMServoDriver();

/* Prototypes */
#ifdef ROS_ENABLE
void cmdVelCallback(  const geometry_msgs::Twist& vel);
#endif
void steer(float angle);
void throttle(float percent);
void resetMotors();
void executeMovement();
void readReceiverData();

#ifdef ROS_ENABLE
// ROS
ros::NodeHandle nh;
ros::Subscriber<geometry_msgs::Twist> sub_cmd_vel("cmd_vel" , cmdVelCallback);   
#endif