/*
 * Title: Dual DC Motor Control with Encoder
 * Author: Khairul Izwan
 * Description: 
 */

//include necessary library
#include <ros.h>
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Bool.h>

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

//Callback function for geometry_msgs::Twist
void messageCb_rstEncLeft(const std_msgs::Bool &msg)
{
  if (msg.data == true)
  {
    COUNTER_B = 0;
  }
}

//Callback function for geometry_msgs::Twist
void messageCb_rstEncRight(const std_msgs::Bool &msg)
{
  if (msg.data == true)
  {
    COUNTER_A = 0;
  }
}

//Set up the ros node (publisher and subscriber)
std_msgs::Int64 encLeft;
std_msgs::Int64 encRight;
ros::Publisher pub_encLeft("val_encLeft", &encLeft);
ros::Publisher pub_encRight("val_encRight", &encRight);

ros::Subscriber<std_msgs::Bool> sub_rstLeft("/rstEncLeft", messageCb_rstEncLeft);
ros::Subscriber<std_msgs::Bool> sub_rstRight("/rstEncRight", messageCb_rstEncRight);

ros::NodeHandle nh;

//put your setup code here, to run once:
void setup()
{ 
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

//Initiate ROS-node
  nh.initNode();

  nh.advertise(pub_encLeft);
  nh.advertise(pub_encRight);

  nh.subscribe(sub_rstLeft);
  nh.subscribe(sub_rstRight);
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
