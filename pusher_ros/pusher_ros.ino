#include <Adafruit_MotorShield.h>
#include <utility/Adafruit_PWMServoDriver.h>
#include <Servo.h>

#include <ros.h>
#include <wiff_waff/HopperGoal.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// We'll also test out the built in Arduino Servo library
Servo servo1;

int servo_low_angle=70;
int servo_high_angle=110;

ros::NodeHandle nh;

const int POS_FULL_OPEN=0;
const int POS_FULL_FWD=255;
const int POS_LOCK_ONLY=135;
const int POS_OPEN_ENOUGH=20;

int servo_position=0;
int target_servo_position=0;
robot_pong::HopperGoal active_goal;

const int SHOOT_OPEN=0;
const int SHOOT_CLOSE=1;
const int SHOOT_READY=2;
int shoot_state = 0;

void messageCb( const robot_pong::HopperGoal& goal_msg){
  active_goal = goal_msg;
  if( active_goal.action == robot_pong::HopperGoal::ACTION_OFF ){
    servo1.detach();
  }else{
    if(!servo1.attached()) servo1.attach(9);

    switch( active_goal.action ){
    case robot_pong::HopperGoal::ACTION_LOCK:
      target_servo_position = POS_LOCK_ONLY;
      break;
    case robot_pong::HopperGoal::ACTION_OPEN:
      target_servo_position = POS_FULL_OPEN;
      break;
    case robot_pong::HopperGoal::ACTION_SHOOT:
      target_servo_position = POS_OPEN_ENOUGH;
      shoot_state=SHOOT_OPEN;
      break;
    }
  }
}

ros::Subscriber<robot_pong::HopperGoal> goal_sub("hopper_goal", &messageCb );

void setup() {

  AFMS.begin();  // create with the default frequency 1.6KHz

  active_goal.action = robot_pong::HopperGoal::ACTION_OFF;
  servo_position=0;
  target_servo_position=0;

  nh.initNode();
  nh.subscribe(goal_sub);
}

void loop() {
  nh.spinOnce();

  if( servo_position > target_servo_position ){
    servo_position --;
  }else if( servo_position < target_servo_position ){
    servo_position ++;
  }

  servo1.write(map(servo_position, 0, 255, servo_low_angle, servo_high_angle));
  if( active_goal.action == robot_pong::HopperGoal::ACTION_SHOOT && servo_position == target_servo_position ){
     if( shoot_state == SHOOT_OPEN ){
        delay(300);
        target_servo_position = POS_FULL_FWD;
        shoot_state = SHOOT_CLOSE;
     }else if( shoot_state == SHOOT_CLOSE){
        delay(2000);
        target_servo_position = POS_LOCK_ONLY;
        shoot_state = SHOOT_READY;
     }
  }
  delay(2);
}
