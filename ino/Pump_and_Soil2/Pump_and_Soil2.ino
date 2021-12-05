/*
 * Title: Pump control with Soil moister reader
 * Author: Khairul Izwan
 * Description: 
 */

//include necessary library
#include <ros.h>
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Bool.h>

#define sensorPin0 A0
#define sensorPin1 A1
#define pwm0_Pump0 10
#define pwm1_Pump0 11
#define pwm0_Pump1 5
#define pwm1_Pump1 6

void messageCb_pump0(const std_msgs::Bool &msg)
{
  if (msg.data == true)
  {
    analogWrite(pwm0_Pump0, 0);
    analogWrite(pwm1_Pump0, 10);
  }
  else
  {
    analogWrite(pwm0_Pump0, 0);
    analogWrite(pwm1_Pump0, 0);
  }
}

void messageCb_pump1(const std_msgs::Bool &msg)
{
  if (msg.data == true)
  {
    analogWrite(pwm0_Pump1, 0);
    analogWrite(pwm1_Pump1, 10);
  }
  else
  {
    analogWrite(pwm0_Pump1, 0);
    analogWrite(pwm1_Pump1, 0);
  }
}

//Set up the ros node (publisher and subscriber)
std_msgs::Int64 soil0;
std_msgs::Int64 soil1;
ros::Publisher pub_soil0("val_soil2", &soil0);
ros::Publisher pub_soil1("val_soil3", &soil1);

ros::Subscriber<std_msgs::Bool> sub_pump0("/pump2", messageCb_pump0);
ros::Subscriber<std_msgs::Bool> sub_pump1("/pump3", messageCb_pump1);

ros::NodeHandle nh;

//put your setup code here, to run once:
void setup()
{ 

//Initiate ROS-node
  nh.initNode();

  nh.advertise(pub_soil0);
  nh.advertise(pub_soil1);

  nh.subscribe(sub_pump0);
  nh.subscribe(sub_pump1);
}

//put your main code here, to run repeatedly:
void loop()
{
  soil0.data = analogRead(sensorPin0);
  soil1.data = analogRead(sensorPin1);

  pub_soil0.publish(&soil0);
  pub_soil1.publish(&soil1);
  
  nh.spinOnce();
}
