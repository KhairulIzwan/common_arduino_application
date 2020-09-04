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

//Encoder Pins Definition
//Using an interrupt pins :: 2, 3, 18, 19, 20, 21 
//:: https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/?setlang=it
#define ENCODER_COUNT_UP_A 2
#define ENCODER_COUNT_DOWN_A 3

//Encoder Variables
volatile signed int TEMP_A, COUNTER_A = 0;

//Change according to the robot wheel dimension
#define wheelSep 0.34 // in unit meter (m)
#define wheelRadius 0.12; // in unit meter (m)

//Variables declaration (DC Motor)
float DutyCycle;
float PWM;

//Motor Pin Assignment (DC Motor)
int DIRA = 11;
int PWMA = 9;

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

//Callback function for geometry_msgs::Twist (DC Motor)
void messageCb_DC_cmd_vel(const geometry_msgs::Twist &msg)
{
//  Get the ros topic value
  DutyCycle = msg.linear.x;
  PWM = msg.angular.z;

//  motor directection helper function
  DCmotorDirection();
}

//Motor Direction helper function (DC Motor)
void DCmotorDirection()
{
//  Forward
  if (DutyCycle > 0)
  {
//    digitalWrite(DIRA, LOW);
    digitalWrite(DIRA, HIGH);
    analogWrite(PWMA, PWM);
  }
//  Backward
  else if (DutyCycle < 0)
  {
//    digitalWrite(DIRA, HIGH);
    digitalWrite(DIRA, LOW);
    analogWrite(PWMA, PWM);
  }
/*
 * There is no turn left and right (not a differential drive mode)
 */
  else if (DutyCycle == 0)
  {
    analogWrite(PWMA, 0);
  }
}

//Set up the ros node (publisher and subscriber)
std_msgs::Float32 enc;
//ros::Publisher pub_enc("encRight", &enc);
ros::Publisher pub_enc("encLeft", &enc);

//ros::Subscriber<geometry_msgs::Twist> sub_DC_cmd_vel("/leftSpeed", messageCb_DC_cmd_vel);
ros::Subscriber<geometry_msgs::Twist> sub_DC_cmd_vel("/leftSpeed", messageCb_DC_cmd_vel);

ros::NodeHandle nh;

//put your setup code here, to run once:
void setup()
{ 
//  Encoder Pins Pull-Up
  pinMode(ENCODER_COUNT_UP_A, INPUT_PULLUP);
  pinMode(ENCODER_COUNT_DOWN_A, INPUT_PULLUP);

//  Interrupt Input
  attachInterrupt(digitalPinToInterrupt(ENCODER_COUNT_UP_A), COUNT_INTERRUPT_CW_A, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_COUNT_DOWN_A), COUNT_INTERRUPT_CCW_A, RISING);
    
//Input/Output Pins Assigment
  pinMode(DIRA, OUTPUT);
  pinMode(PWMA, OUTPUT);

//Initiate ROS-node
  nh.initNode();
  nh.advertise(pub_enc);
  nh.subscribe(sub_DC_cmd_vel);
}

//put your main code here, to run repeatedly:
void loop()
{
  enc.data = COUNTER_A;
  pub_enc.publish(&enc);
  nh.spinOnce();
  delay(1);
}
