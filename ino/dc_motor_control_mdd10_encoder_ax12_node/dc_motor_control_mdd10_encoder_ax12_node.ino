/*
 * Title: Dual DC Motor Control with Encoder
 * Author: Khairul Izwan
 * Description: 
 */

//include necessary library
#include <ros.h>
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>

#include "Arduino.h"
#include "AX12A.h"

#define DirectionPin   (10u)
#define BaudRate      (1000000ul)
#define IDB2        (10u)
#define IDB1        (9u)
#define IDA1        (8u)
#define IDA2        (7u)

//Variables declaration (AX-12)
float transVelocity_ax12;
float rotVelocity_ax12;

float leftVelocity_ax12;
float rightVelocity_ax12;

float leftDutyCycle_ax12;
float rightDutyCycle_ax12;

float leftPWM_ax12;
float rightPWM_ax12;

//Encoder Pins Definition
//Using an interrupt pins :: 2, 3, 18, 19, 20, 21 
//:: https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/?setlang=it
#define ENCODER_COUNT_UP_A 19
#define ENCODER_COUNT_DOWN_A 18
#define ENCODER_COUNT_UP_B 20
#define ENCODER_COUNT_DOWN_B 21

//Encoder Variables
volatile signed int TEMP_A, COUNTER_A = 0;
volatile signed int TEMP_B, COUNTER_B = 0; 

//Change according to the robot wheel dimension
#define wheelSep 0.34 // in unit meter (m)
#define wheelRadius 0.12; // in unit meter (m)

//Variables declaration (DC Motor)
float transVelocity;
float rotVelocity;

float leftVelocity;
float rightVelocity;

float leftDutyCycle;
float rightDutyCycle;

float leftPWM;
float rightPWM;

//Motor Pin Assignment (DC Motor)
int DIRA = 4;
int PWMA = 5;
int DIRB = 12;
int PWMB = 10;

//Callback function for geometry_msgs::Twist (AX-12)
void messageCb_ax12_cmd_vel(const geometry_msgs::Twist &msg)
{
//  Get the ros topic value
  transVelocity_ax12 = msg.linear.x;
  rotVelocity_ax12 = msg.angular.z;
  
//  Differential Drive Kinematics
//::http://www.cs.columbia.edu/~allen/F15/NOTES/icckinematics.pdf
//  Differential Drive Kinematics
//::https://snapcraft.io/blog/your-first-robot-the-driver-4-5

//  Step 1: Calculate wheel speeds from Twist
  leftVelocity_ax12 = transVelocity_ax12 - ((rotVelocity_ax12 * wheelSep) / 2);
  rightVelocity_ax12 = transVelocity_ax12 + ((rotVelocity_ax12 * wheelSep) / 2);
  
//  Step 2: Convert wheel speeds into duty cycles
  leftDutyCycle_ax12 = (1023 * leftVelocity_ax12) / 0.22;
  rightDutyCycle_ax12 = (1023 * rightVelocity_ax12) / 0.22;

  rightDutyCycle_ax12 = leftDutyCycle_ax12;
  
//  Step3: Ensure DutyCycle is between minimum and maximum
  leftPWM_ax12 = clipPWM(abs(leftDutyCycle_ax12), 200, 1023);
  rightPWM_ax12 = clipPWM(abs(rightDutyCycle_ax12), 200, 1023);

  rightPWM_ax12 = leftPWM_ax12;

//  motor directection helper function
  ax12motorDirection();
}

//Motor Direction helper function
void ax12motorDirection()
{
//  Left
  if (leftDutyCycle_ax12 > 0 and rightDutyCycle_ax12 > 0)
  {
    ax12a.turn(IDA1, RIGHT, rightPWM_ax12);
    ax12a.turn(IDA2, LEFT, rightPWM_ax12);
    ax12a.turn(IDB1, RIGHT, rightPWM_ax12);
    ax12a.turn(IDB2, LEFT, rightPWM_ax12);
    
    ax12a.ledStatus(IDA1, ON);
    ax12a.ledStatus(IDA2, ON);
    ax12a.ledStatus(IDB1, ON);
    ax12a.ledStatus(IDB2, ON);
  }
//  Right
  else if (leftDutyCycle_ax12 < 0 and rightDutyCycle_ax12 < 0)
  {
    ax12a.turn(IDA1, LEFT, rightPWM_ax12);
    ax12a.turn(IDA2, RIGHT, rightPWM_ax12);
    ax12a.turn(IDB1, LEFT, rightPWM_ax12);
    ax12a.turn(IDB2, RIGHT, rightPWM_ax12);

    ax12a.ledStatus(IDA1, ON);
    ax12a.ledStatus(IDA2, ON);
    ax12a.ledStatus(IDB1, ON);
    ax12a.ledStatus(IDB2, ON);
  }
//  Stop
  else if (leftDutyCycle_ax12 == 0 and rightDutyCycle_ax12 == 0)
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

//Counter Helper Function
//Count Up A
void COUNT_INTERRUPT_CW_A() 
{
  if(digitalRead(ENCODER_COUNT_DOWN_A)==LOW) 
  {
    COUNTER_A++;
  }
  else
  {
    COUNTER_A--;
  }
}

//Count Down A
void COUNT_INTERRUPT_CCW_A()
{
  if(digitalRead(ENCODER_COUNT_UP_A)==LOW) 
  {
    COUNTER_A--;
  }
  else
  {
    COUNTER_A++;
  }
}

//Count Up B
void COUNT_INTERRUPT_CW_B() 
{
  if(digitalRead(ENCODER_COUNT_DOWN_B)==LOW) 
  {
    COUNTER_B++;
  }
  else
  {
    COUNTER_B--;
  }
}

//Count Down A
void COUNT_INTERRUPT_CCW_B()
{
  if(digitalRead(ENCODER_COUNT_UP_B)==LOW) 
  {
    COUNTER_B--;
  }
  else
  {
    COUNTER_B++;
  }
}

//Callback function for geometry_msgs::Twist (DC Motor)
void messageCb_DC_cmd_vel(const geometry_msgs::Twist &msg)
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
  DCmotorDirection();
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

//Motor Direction helper function (DC Motor)
void DCmotorDirection()
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
  else if (leftDutyCycle == 0 and rightDutyCycle == 0)
  {
    analogWrite(PWMA, 0);
    analogWrite(PWMB, 0);
  }
}

//Set up the ros node (publisher and subscriber)
std_msgs::Float32 encLeft;
std_msgs::Float32 encRight;
ros::Publisher pub_encLeft("val_encLeft", &encLeft);
ros::Publisher pub_encRight("val_encRight", &encRight);

ros::Subscriber<geometry_msgs::Twist> sub_DC_cmd_vel("/cmd_vel", messageCb_DC_cmd_vel);
ros::Subscriber<geometry_msgs::Twist> sub_ax12_cmd_vel("/cmd_vel", messageCb_ax12_cmd_vel);

ros::NodeHandle nh;

//put your setup code here, to run once:
void setup()
{
  Serial2.begin(57600);
  Serial3.begin(57600);
  
//  Encoder Pins Pull-Up
  pinMode(ENCODER_COUNT_UP_A, INPUT_PULLUP);
  pinMode(ENCODER_COUNT_DOWN_A, INPUT_PULLUP);
  pinMode(ENCODER_COUNT_UP_B, INPUT_PULLUP);
  pinMode(ENCODER_COUNT_DOWN_B, INPUT_PULLUP);

//  Interrupt Input
  attachInterrupt(digitalPinToInterrupt(ENCODER_COUNT_UP_A), COUNT_INTERRUPT_CW_A, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_COUNT_DOWN_A), COUNT_INTERRUPT_CCW_A, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_COUNT_UP_B), COUNT_INTERRUPT_CW_B, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_COUNT_DOWN_B), COUNT_INTERRUPT_CCW_B, RISING);
    
//Input/Output Pins Assigment
  pinMode(DIRA, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(DIRB, OUTPUT);
  pinMode(PWMB, OUTPUT);

  ax12a.begin(BaudRate, DirectionPin, &Serial2);
  
  ax12a.setEndless(IDA1, ON);
  ax12a.setEndless(IDA2, ON);
  ax12a.setEndless(IDB1, ON);
  ax12a.setEndless(IDB2, ON);

//Initiate ROS-node
  nh.initNode();

  nh.advertise(pub_encLeft);
  nh.advertise(pub_encRight);
  
  nh.subscribe(sub_DC_cmd_vel);
  nh.subscribe(sub_ax12_cmd_vel);
}

//put your main code here, to run repeatedly:
void loop()
{
  encRight.data = COUNTER_A;
  encLeft.data = COUNTER_B;

  pub_encLeft.publish(&encLeft);
  pub_encRight.publish(&encRight);
  
  nh.spinOnce();
}
