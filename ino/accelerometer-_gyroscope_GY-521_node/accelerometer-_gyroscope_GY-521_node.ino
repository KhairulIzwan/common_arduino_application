// Basic demo for accelerometer readings from Adafruit MPU6050

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

//Set up the ros node (publisher and subscriber)
//std_msgs::Float32 accelX;
//std_msgs::Float32 accelY;
//std_msgs::Float32 accelZ;
//
//std_msgs::Float32 gyroX;
//std_msgs::Float32 gyroY;
//std_msgs::Float32 gyroZ;
//
//std_msgs::Float32 tempC;

std_msgs::Float32 roll;
std_msgs::Float32 pitch;
std_msgs::Float32 yaw;

//ros::Publisher pub_accelX("accelX", &accelX);
//ros::Publisher pub_accelY("accelX", &accelY);
//ros::Publisher pub_accelZ("accelX", &accelZ);
//
//ros::Publisher pub_gyroX("gyroX", &gyroX);
//ros::Publisher pub_gyroY("gyroY", &gyroY);
//ros::Publisher pub_gyroZ("gyroZ", &gyroZ);
//
//ros::Publisher pub_tempC("tempC", &tempC);

ros::Publisher pub_roll("roll", &roll);
ros::Publisher pub_pitch("pitch", &pitch);
ros::Publisher pub_yaw("yaw", &yaw);

ros::NodeHandle nh;

void setup(void) {
  mpu.begin();
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  //Initiate ROS-node
  nh.initNode();

//  nh.advertise(pub_accelX);
//  nh.advertise(pub_accelY);
//  nh.advertise(pub_accelZ);
//  
//  nh.advertise(pub_gyroX);
//  nh.advertise(pub_gyroY);
//  nh.advertise(pub_gyroZ);
//  
//  nh.advertise(pub_tempC);

  nh.advertise(pub_roll);
  nh.advertise(pub_pitch);
  nh.advertise(pub_yaw);
}

void loop() {

  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

//  accelX.data = a.acceleration.x;
//  accelY.data = a.acceleration.y;
//  accelZ.data = a.acceleration.z;
//
//  gyroX.data = g.gyro.x;
//  gyroY.data = g.gyro.y;
//  gyroZ.data = g.gyro.z;
//
//  tempC.data = temp.temperature;

  pitch.data = 180 * atan (a.acceleration.x/sqrt(a.acceleration.y*a.acceleration.y + a.acceleration.z*a.acceleration.z))/M_PI;
  roll.data = 180 * atan (a.acceleration.y/sqrt(a.acceleration.x*a.acceleration.x + a.acceleration.z*a.acceleration.z))/M_PI;
  yaw.data = 180 * atan (a.acceleration.z/sqrt(a.acceleration.x*a.acceleration.x + a.acceleration.z*a.acceleration.z))/M_PI;

//  pub_accelX.publish(&accelX);
//  pub_accelY.publish(&accelY);
//  pub_accelZ.publish(&accelZ);
//  
//  pub_gyroX.publish(&gyroX);
//  pub_gyroY.publish(&gyroY);
//  pub_gyroZ.publish(&gyroZ);
//
//  pub_tempC.publish(&tempC);

  pub_roll.publish(&roll);
  pub_pitch.publish(&pitch);
  pub_yaw.publish(&yaw);
  
  nh.spinOnce();
}
