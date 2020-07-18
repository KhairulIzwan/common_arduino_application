/*
 * Title: Dual DC Motor Control with Encoder
 * Author: Khairul Izwan 22-03-2020
 * Description: Controlling Dual DC Motor with Encoder using
 * 10Amp 7V-30V DC Motor Driver Shield for Arduino (2 Channels)
 */

//include necessary library
#include <ros.h>
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>

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

//Motor Pin Assignment
int DIRA = 4;
int PWMA = 5;
int DIRB = 12;
int PWMB = 10;

//Callback function for geometry_msgs::Twist
void messageCb_cmd_vel(const geometry_msgs::Twist &msg)
{
//  Get the ros topic value
  transVelocity = msg.linear.x;
  rotVelocity = 0;
  
//  Differential Drive Kinematics
//::http://www.cs.columbia.edu/~allen/F15/NOTES/icckinematics.pdf
//  Differential Drive Kinematics
//::https://snapcraft.io/blog/your-first-robot-the-driver-4-5

//  Step 1: Calculate wheel speeds from Twist
  leftVelocity = transVelocity - ((rotVelocity * wheelSep) / 2);
  rightVelocity = transVelocity + ((rotVelocity * wheelSep) / 2);
  
//  Step 2: Convert wheel speeds into duty cycles
  leftDutyCycle = (255 * leftVelocity) / 0.22;
  rightDutyCycle = (255 * rightVelocity) / 0.22;

  rightDutyCycle = leftDutyCycle;

//  Ensure DutyCycle is between minimum and maximum
  leftPWM = clipPWM(abs(leftDutyCycle), 0, 255);
  rightPWM = clipPWM(abs(rightDutyCycle), 0, 255);

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
    digitalWrite(DIRA, HIGH);
    digitalWrite(DIRB, LOW);
    analogWrite(PWMA, leftPWM);
    analogWrite(PWMB, rightPWM);
  }
//  Backward
  else if (leftDutyCycle < 0 and rightDutyCycle < 0)
  {
    digitalWrite(DIRA, LOW);
    digitalWrite(DIRB, HIGH);
    analogWrite(PWMA, leftPWM);
    analogWrite(PWMB, rightPWM);
  }
/*
 * There is no turn left and right (not a differential drive mode)
 */
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
    analogWrite(PWMA, 0);
    analogWrite(PWMB, 0);
  }
}

//Set up the ros node (publisher and subscriber)
ros::Subscriber<geometry_msgs::Twist> sub_cmd_vel("/cmd_vel", messageCb_cmd_vel);

ros::NodeHandle nh;

//put your setup code here, to run once:
void setup()
{
  
//Input/Output Pins Assigment
  pinMode(DIRA, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(DIRB, OUTPUT);
  pinMode(PWMB, OUTPUT);

//Initiate ROS-node
  nh.initNode();
  nh.subscribe(sub_cmd_vel);
}

//put your main code here, to run repeatedly:
void loop()
{
  nh.spinOnce();
}
