// Basic demo for accelerometer readings from Adafruit MPU6050

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

//Set up the ros node (publisher and subscriber)
std_msgs::Float32 accelX;
std_msgs::Float32 accelY;
std_msgs::Float32 accelZ;

ros::Publisher pub_accelX("accelX", &accelX);
ros::Publisher pub_accelY("accelY", &accelY);
ros::Publisher pub_accelZ("accelZ", &accelZ);

ros::NodeHandle nh;

void setup(void) {
  mpu.begin();
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  //Initiate ROS-node
  nh.initNode();

  nh.advertise(pub_accelX);
  nh.advertise(pub_accelY);
  nh.advertise(pub_accelZ);
}

void loop() {

  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  accelX.data = a.acceleration.x;
  accelY.data = a.acceleration.y;
  accelZ.data = a.acceleration.z;

  pub_accelX.publish(&accelX);
  pub_accelY.publish(&accelY);
  pub_accelZ.publish(&accelZ);
  
  nh.spinOnce();
}
