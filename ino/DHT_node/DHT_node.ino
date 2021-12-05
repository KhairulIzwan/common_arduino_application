
//include necessary library
#include <ros.h>
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Bool.h>

#include "DHT.h"

#define DHTPIN 53
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
#define pwm0_Pump0 10
#define pwm1_Pump0 11
#define pwm0_Pump1 5
#define pwm1_Pump1 6
#define L1 25
#define L2 24
#define L3 23
#define L4 22
#define ldrPin A0

DHT dht(DHTPIN, DHTTYPE);

void messageCb_pump0(const std_msgs::Bool &msg)
{
  if (msg.data == true)
  {
    analogWrite(pwm0_Pump0, 0);
    analogWrite(pwm1_Pump0, 255);
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
    analogWrite(pwm1_Pump1, 255);
  }
  else
  {
    analogWrite(pwm0_Pump1, 0);
    analogWrite(pwm1_Pump1, 0);
  }
}


//Set up the ros node (publisher and subscriber)
std_msgs::Float64 humid;
std_msgs::Float64 temp;
//std_msgs::Int64 level1;
//std_msgs::Int64 level2;
//std_msgs::Int64 level3;
//std_msgs::Int64 level4;
std_msgs::Int64 ldrPinOutputVal;
ros::Publisher pub_Humid("val_Humid", &humid);
ros::Publisher pub_Temp("val_Temp", &temp);
//ros::Publisher pub_L1("val_L1", &level1);
//ros::Publisher pub_L2("val_L2", &level2);
//ros::Publisher pub_L3("val_L3", &level3);
//ros::Publisher pub_L4("val_L4", &level4);
ros::Publisher pub_ldrPinOutputVal("val_ldrPin", &ldrPinOutputVal);

ros::Subscriber<std_msgs::Bool> sub_pump0("/light", messageCb_pump0);
ros::Subscriber<std_msgs::Bool> sub_pump1("/fan", messageCb_pump1);

ros::NodeHandle nh;

void setup() {
//  pinMode(L1, INPUT_PULLUP);
//  pinMode(L2, INPUT_PULLUP);
//  pinMode(L3, INPUT_PULLUP);
//  pinMode(L4, INPUT_PULLUP);

  pinMode(ldrPin, INPUT);
  
  dht.begin();

  //Initiate ROS-node
  nh.initNode();

  nh.advertise(pub_Humid);
  nh.advertise(pub_Temp);

//  nh.advertise(pub_L1);
//  nh.advertise(pub_L2);
//  nh.advertise(pub_L3);
//  nh.advertise(pub_L4);
  nh.advertise(pub_ldrPinOutputVal);

  nh.subscribe(sub_pump0);
  nh.subscribe(sub_pump1);
}

void loop() {
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  humid.data = dht.readHumidity();
  // Read temperature as Celsius (the default)
  temp.data = dht.readTemperature();

//  level1.data=digitalRead(L1);
//  level2.data=digitalRead(L2);
//  level3.data=digitalRead(L3);
//  level4.data=digitalRead(L4);
  ldrPinOutputVal.data=analogRead(ldrPin);

  pub_Humid.publish(&humid);
  pub_Temp.publish(&temp);
  
//  pub_L1.publish(&level1);
//  pub_L2.publish(&level2);
//  pub_L3.publish(&level3);
//  pub_L4.publish(&level4);
  pub_ldrPinOutputVal.publish(&ldrPinOutputVal);
  
  nh.spinOnce();
}
