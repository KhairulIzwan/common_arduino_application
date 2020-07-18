/*
 * Example showing how to use endless mode (wheel mode) on AX-12A
 * Be sure you removed all mechanical assemblies (hinges) before using this code !
 */
//include necessary library
#include <ros.h>
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include "Arduino.h"
#include "AX12A.h"

#define DirectionPin 	(10u)
#define BaudRate  		(1000000ul)
//ROBOT1
//#define IDB2        (10u)
//#define IDB1        (9u)
//#define IDA1        (8u)
//#define IDA2        (7u)
//ROBOT2
#define IDB2        (13u)
#define IDB1        (5u)
#define IDA1        (16u)
#define IDA2        (17u)

//Change according to the robot wheel dimension
#define wheelSep 0.34 // in unit meter (m)
#define wheelRadius 0.12; // in unit meter (m)

//Variables declaration
float transVelocity;
float rotVelocity;

float leftVelocity;
float rightVelocity;

float leftDutyCycle;
float rightDutyCycle;

float leftPWM;
float rightPWM;

//Callback function for geometry_msgs::Twist
void messageCb_cmd_vel(const geometry_msgs::Twist &msg)
{
//  Get the ros topic value
  transVelocity = 0;
  rotVelocity = msg.angular.z;
  
//  Differential Drive Kinematics
//::http://www.cs.columbia.edu/~allen/F15/NOTES/icckinematics.pdf
//  Differential Drive Kinematics
//::https://snapcraft.io/blog/your-first-robot-the-driver-4-5

//  Step 1: Calculate wheel speeds from Twist
  leftVelocity = transVelocity - ((rotVelocity * wheelSep) / 2);
  rightVelocity = transVelocity + ((rotVelocity * wheelSep) / 2);
  
//  Step 2: Convert wheel speeds into duty cycles
  leftDutyCycle = (1023 * leftVelocity) / 0.22;
  rightDutyCycle = (1023 * rightVelocity) / 0.22;

  rightDutyCycle = leftDutyCycle;
  
//  Step3: Ensure DutyCycle is between minimum and maximum
  leftPWM = clipPWM(abs(leftDutyCycle), 0, 1023);
  rightPWM = clipPWM(abs(rightDutyCycle), 0, 1023);

  rightPWM = leftPWM;

//  motor directection helper function
  motorDirection();
}

//Helper function to ensure DutyCycle is between minimum
//and maximum
float clipPWM(float PWM, float minPWM, float maxPWM)
{
  if (PWM < minPWM)
  {
    return minPWM;
  }
  else if (PWM > maxPWM)
  {
    return maxPWM;
  }
  return PWM;
}

//Motor Direction helper function
void motorDirection()
{
//  Forward
  if (leftDutyCycle > 0 and rightDutyCycle > 0)
  {
    ax12a.turn(IDA1, LEFT, rightPWM);
    ax12a.turn(IDA2, RIGHT, rightPWM);
    ax12a.turn(IDB1, LEFT, leftPWM);
    ax12a.turn(IDB2, RIGHT, leftPWM);
    
    ax12a.ledStatus(IDA1, ON);
    ax12a.ledStatus(IDA2, ON);
    ax12a.ledStatus(IDB1, ON);
    ax12a.ledStatus(IDB2, ON);
  }
//  Backward
  else if (leftDutyCycle < 0 and rightDutyCycle < 0)
  {
    ax12a.turn(IDA1, RIGHT, rightPWM);
    ax12a.turn(IDA2, LEFT, rightPWM);
    ax12a.turn(IDB1, RIGHT, leftPWM);
    ax12a.turn(IDB2, LEFT, leftPWM);

    ax12a.ledStatus(IDA1, ON);
    ax12a.ledStatus(IDA2, ON);
    ax12a.ledStatus(IDB1, ON);
    ax12a.ledStatus(IDB2, ON);
  }
////  Left
//  else if (leftDutyCycle < 0 and rightDutyCycle > 0)
//  {
//    digitalWrite(DIRA, HIGH);
//    digitalWrite(DIRB, HIGH);
//    analogWrite(PWMA, leftPWM);
//    analogWrite(PWMB, rightPWM);
//  }
////  Right
//  else if (leftDutyCycle > 0 and rightDutyCycle < 0)
//  {
//    digitalWrite(DIRA, LOW);
//    digitalWrite(DIRB, LOW);
//    analogWrite(PWMA, leftPWM);
//    analogWrite(PWMB, rightPWM);
//  }
  else if (leftDutyCycle == 0 and rightDutyCycle == 0)
  {
    ax12a.turn(IDA1, RIGHT, 0);
    ax12a.turn(IDA2, LEFT, 0);
    ax12a.turn(IDB1, LEFT, 0);
    ax12a.turn(IDB2, RIGHT, 0);
  
    ax12a.ledStatus(IDA1, OFF);
    ax12a.ledStatus(IDA2, OFF);
    ax12a.ledStatus(IDB1, OFF);
    ax12a.ledStatus(IDB2, OFF);
  }
}

//Set up the ros node (publisher and subscriber)
ros::Subscriber<geometry_msgs::Twist> sub_cmd_vel("/cmd_vel", messageCb_cmd_vel);

ros::NodeHandle nh;

void setup()
{
  ax12a.begin(BaudRate, DirectionPin, &Serial2);
  ax12a.setEndless(IDA1, ON);
  ax12a.setEndless(IDA2, ON);
  ax12a.setEndless(IDB1, ON);
  ax12a.setEndless(IDB2, ON);

//Initiate ROS-node
  nh.initNode();
  nh.subscribe(sub_cmd_vel);
}

void loop()
{
  nh.spinOnce();  
}
