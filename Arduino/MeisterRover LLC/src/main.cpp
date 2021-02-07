#include "main.h"

/* Callbacks */
#ifdef ROS_ENABLE
void cmdVelCallback(  const geometry_msgs::Twist& vel)
{
    demand_x = vel.linear.x;
    demand_z = vel.angular.z;
}// end :: cmdVelCallback()
#endif

/* Helpers */
void steer(float angle) {

  // Apply speed limit
  angle = constrain(angle, ANGLE_MIN, ANGLE_MAX);

  // Flip the angle
  angle = angle * -1;

  uint16_t final_ms = map(angle, VIRT_MIN, VIRT_MAX, SERVO_MIN, SERVO_MAX);
  pwm_board.writeMicroseconds(SERVO_STEERING, final_ms);

}// end :: steer()

void throttle(float percent) {

  // Apply speed limit
  percent = constrain(percent, SPEED_LIMIT_REV, SPEED_LIMIT);

  uint16_t final_ms = map(percent, VIRT_MIN, VIRT_MAX, SERVO_MIN, SERVO_MAX);
  pwm_board.writeMicroseconds(SERVO_THROTTLE, final_ms);

}// end :: throttle

/* Routines */
void resetMotors() {
  throttle(THROTTLE_IDLE);
  steer(STEERING_IDLE);
}// end :: resetNotors()

void executeMovement() {

  // Execute
  steer(demand_z);
  throttle(demand_x);

}// end :: executeMovement()

void readReceiverData() {
  uint16_t throttle_ms = pulseIn(RX_THROTTLE, HIGH);
  uint16_t steering_ms = pulseIn(RX_STEERING, HIGH);
  uint16_t estop_ms = pulseIn(RX_ESTOP, HIGH);

#ifndef ROS_ENABLE
  // Serial.print("Throttle: ");
  // Serial.println(throttle_ms);
  // Serial.print("Steering: ");
  // Serial.println(steering_ms);
  // Serial.print("E-Stop: ");
  // Serial.println(estop_ms);
  //delay(1000);
#endif

  // eStop
  if (estop_ms > SERVO_MID) {
    eStop = true;
  } else {
    eStop = false;
  }// end :: if 

  if (!eStop) {
    float _dx;
    float _dy;
    
    if (RX_DEADZONE_LOW <= throttle_ms && throttle_ms <= RX_DEADZONE_HIGH) {
      _dx = THROTTLE_IDLE;
    } else {
      _dx = map(throttle_ms, RX_MIN, RX_MAX, VIRT_MIN, VIRT_MAX);
    }// end :: if
    
    if (RX_DEADZONE_LOW <= steering_ms && steering_ms <= RX_DEADZONE_HIGH) {
      _dy = STEERING_IDLE;
    } else {
      _dy = map(steering_ms, RX_MIN, RX_MAX, VIRT_MIN, VIRT_MAX);
    }// end :: if
    
    steer(_dy);
    throttle(_dx);
  } else {
    resetMotors();
  }//end :: if

}// end :: readReceiverData()

/* Setup & Loop */

void setup() {

#ifdef ROS_ENABLE
  // ROS Node Handler
  nh.getHardware()->setBaud(115200);    // set baud rate to 115200
  nh.initNode();                        // init ROS
  nh.subscribe(sub_cmd_vel);            // subscribe to cmd_vel
#else
  Serial.begin(9600);
#endif

  // Configure RX Pins
  for (byte i=0; i < RX_PIN_COUNT; i++) {
    pinMode(RX_PINS[i], INPUT);
  }//end :: for

  // Setup Servo Driver
  pwm_board.begin();
  pwm_board.setPWMFreq(SERVO_FREQ);

  // Reset Motors
  resetMotors();

  // Delay start
  delay(STARTUP_DELAY);

}// end :: setup()

void loop() {

#ifdef ROS_ENABLE  
  // ROS Heartbeat
  nh.spinOnce();
#endif

  // Refresh Current Timer
  currentMillis = millis();

  if (currentMillis - previousMillis >= LOOP_TIME) {

    readReceiverData();

    if (!eStop) {
      //executeMovement();
    } else {
      resetMotors();
    }//end :: if

    // Update Previous Timer
    previousMillis = currentMillis;

  }// end :: if

}//end :: loop()