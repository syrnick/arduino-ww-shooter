#include <ros.h>
#include <std_msgs/String.h>

#include <whiff_waff_msgs/System.h>
#include <whiff_waff_msgs/ShooterSpeed.h>

ros::NodeHandle nh;

whiff_waff_msgs::System power_msg;
whiff_waff_msgs::System hopper_msg;
whiff_waff_msgs::ShooterSpeed shooter_speed_msg;

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

char chatter_msg[100];


// server
//    pins
int serverPin1=2;
int serverPin2=4;
int serverSpeedPin=5;
//    speed
int serverSpeed = 128;


// shooters
#define DIR_IDX 0
#define SPEED_IDX 1
#define BRAKE_IDX 2
#define CURRENT_IDX 3

int motorA[] = {12, 3, 9, 0};
int motorB[] = {13, 11, 8, 1};

int onOffSwitch = 1;
bool wasRunning = false;

const int MAX_SHOOTER_SPEED = 255;

int shooterASpeed = -1;
int shooterBSpeed = -1;

void stopMotor(int *motorPins);

void setupServer()
{
  pinMode(serverPin1,OUTPUT);
  pinMode(serverPin2,OUTPUT);
  pinMode(serverSpeedPin,OUTPUT);
}

void setupMotor(int *motorPins)
{
  pinMode(motorPins[DIR_IDX], OUTPUT);
  pinMode(motorPins[SPEED_IDX], OUTPUT);
  pinMode(motorPins[BRAKE_IDX], OUTPUT);
  pinMode(motorPins[CURRENT_IDX], INPUT);
  stopMotor(motorPins);
}

void runServer()
{
  analogWrite(serverSpeedPin,serverSpeed);
  digitalWrite(serverPin2,HIGH); // turn server clockwise
  digitalWrite(serverPin1,LOW);
}
void stopServer(int waitDelay=100)
{
  // Unenble the pin, to stop the motor. this should be done to avid damaging the motor.
  digitalWrite(serverSpeedPin,LOW);
  delay(waitDelay);
}


void stopMotor(int *motorPins){
  analogWrite(motorPins[SPEED_IDX], 0);
  digitalWrite(motorPins[BRAKE_IDX], HIGH);
  digitalWrite(motorPins[DIR_IDX], LOW);
}

void startMotor(int *motorPins, int direction, int speed){
  analogWrite(motorPins[SPEED_IDX], speed);
  digitalWrite(motorPins[BRAKE_IDX], LOW);
  digitalWrite(motorPins[DIR_IDX], direction);
}

void runMotors(){
  shooterASpeed = min(MAX_SHOOTER_SPEED,max(0,shooter_speed_msg.left_shooter_speed));
  shooterBSpeed = min(MAX_SHOOTER_SPEED,max(0,shooter_speed_msg.right_shooter_speed));
  startMotor(motorA, LOW, shooterASpeed);
  startMotor(motorB, LOW, shooterBSpeed);
}
void startMotors(){
  startMotor(motorA, LOW, MAX_SHOOTER_SPEED);
  startMotor(motorB, LOW, MAX_SHOOTER_SPEED);
  delay(100);
  runMotors();
}

void checkState(){
  if(power_msg.on && onOffSwitch){
    if(wasRunning){
      runMotors();
    }else{
      startMotors();
    }
    if( hopper_msg.on ){
      runServer();
    }else{
      stopServer();
    }
    wasRunning = true;
  }else{
    stopMotor(motorB);
    stopMotor(motorA);
    stopServer();
    wasRunning = false;
  }
  sprintf(chatter_msg, "KILL %d PWR %d HPR %d L %d R %d",
          power_msg.on, onOffSwitch, hopper_msg.on,
          (int)shooter_speed_msg.left_shooter_speed,
          (int)shooter_speed_msg.right_shooter_speed);
  str_msg.data = chatter_msg;
  chatter.publish( &str_msg );
}

void onSystemCb( const whiff_waff_msgs::System& system_msg ){
  power_msg = system_msg;
  checkState();
}

void onHopperCb( const whiff_waff_msgs::System& system_msg ){
  hopper_msg = system_msg;
  checkState();
}

void onShooterCb( const whiff_waff_msgs::ShooterSpeed& speed_msg ){
  shooter_speed_msg = speed_msg;
  checkState();
}

ros::Subscriber<whiff_waff_msgs::System> system_sub( "system", &onSystemCb );
ros::Subscriber<whiff_waff_msgs::System> hopper_sub( "hopper", &onHopperCb );

ros::Subscriber<whiff_waff_msgs::ShooterSpeed> shooter_sub( "shooter", &onShooterCb );

void setup()
{
  power_msg.on = false;
  hopper_msg.on = false;
  wasRunning = false;
  shooter_speed_msg.left_shooter_speed = 0;
  shooter_speed_msg.right_shooter_speed = 0;

  setupServer();

  setupMotor(motorA);
  setupMotor(motorB);

  nh.initNode();
  nh.subscribe(system_sub);
  nh.subscribe(hopper_sub);
  nh.subscribe(shooter_sub);
  nh.advertise(chatter);
}

void loop()
{
  nh.spinOnce();

  onOffSwitch = digitalRead(10);

  if( ! (power_msg.on&&onOffSwitch) ){
    checkState();

    delay(1000);
    return;
  }

  delay(10);
}
