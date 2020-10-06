/*
 * Title: Relay Control
 * Author: Khairul Izwan 10-06-2020
 * Description: Read keyboard strokes (Int64) to activate Relay
 */

//include necessary library
#include <ros.h>
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"

#define forward 2
#define backward 3
#define left 4
#define right 5

//Callback function for std_msgs::Bool
void messageCb_relayF(const std_msgs::Bool &msg)
{
//  Get the ros topic value
  bool state = msg.data;

  if (state == true)
  {
    digitalWrite(forward, 1);
  }
  else
  {
    digitalWrite(forward, 0);
  }
}

//Callback function for std_msgs::Bool
void messageCb_relayB(const std_msgs::Bool &msg)
{
//  Get the ros topic value
  bool state = msg.data;

  if (state == true)
  {
    digitalWrite(backward, 1);
  }
  else
  {
    digitalWrite(backward, 0);
  }
}

//Callback function for std_msgs::Bool
void messageCb_relayL(const std_msgs::Bool &msg)
{
//  Get the ros topic value
  bool state = msg.data;

  if (state == true)
  {
    digitalWrite(left, 1);
  }
  else
  {
    digitalWrite(left, 0);
  }
}

//Callback function for std_msgs::Bool
void messageCb_relayR(const std_msgs::Bool &msg)
{
//  Get the ros topic value
  bool state = msg.data;

  if (state == true)
  {
    digitalWrite(right, 1);
  }
  else
  {
    digitalWrite(right, 0);
  }
}

//Set up the ros node (publisher and subscriber)
ros::Subscriber<std_msgs::Bool> sub_relayF("/relayF", messageCb_relayF);
ros::Subscriber<std_msgs::Bool> sub_relayB("/relayB", messageCb_relayB);
ros::Subscriber<std_msgs::Bool> sub_relayL("/relayL", messageCb_relayL);
ros::Subscriber<std_msgs::Bool> sub_relayR("/relayR", messageCb_relayR);

ros::NodeHandle nh;

//put your setup code here, to run once:
void setup()
{
  
//Input/Output Pins Assigment
  pinMode(forward, OUTPUT);
  pinMode(backward, OUTPUT);
  pinMode(left, OUTPUT);
  pinMode(right, OUTPUT);

//Initiate ROS-node
  nh.initNode();
  nh.subscribe(sub_relayF);
  nh.subscribe(sub_relayB);
  nh.subscribe(sub_relayL);
  nh.subscribe(sub_relayR);
}

//put your main code here, to run repeatedly:
void loop()
{
  nh.spinOnce();
}
