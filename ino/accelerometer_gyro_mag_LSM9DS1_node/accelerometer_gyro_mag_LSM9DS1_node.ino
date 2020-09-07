#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>  // not used in this demo but required!

// i2c
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

#define LSM9DS1_SCK A5
#define LSM9DS1_MISO 12
#define LSM9DS1_MOSI A4
#define LSM9DS1_XGCS 6
#define LSM9DS1_MCS 5

// You can also use software SPI
//Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(LSM9DS1_SCK, LSM9DS1_MISO, LSM9DS1_MOSI, LSM9DS1_XGCS, LSM9DS1_MCS);
// Or hardware SPI! In this case, only CS pins are passed in
//Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(LSM9DS1_XGCS, LSM9DS1_MCS);


void setupSensor()
{
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_16G);
  
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_12GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_16GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_2000DPS);
}

//Set up the ros node (publisher and subscriber)
std_msgs::Float32 accelX;
std_msgs::Float32 accelY;
std_msgs::Float32 accelZ;
//
//std_msgs::Float32 magX;
//std_msgs::Float32 magY;
//std_msgs::Float32 magZ;
//
//std_msgs::Float32 gyroX;
//std_msgs::Float32 gyroY;
//std_msgs::Float32 gyroZ;
//
//std_msgs::Float32 tempC;
//
//std_msgs::Float32 roll;
//std_msgs::Float32 pitch;
//std_msgs::Float32 yaw;

ros::Publisher pub_accelX("accelX", &accelX);
ros::Publisher pub_accelY("accelY", &accelY);
ros::Publisher pub_accelZ("accelZ", &accelZ);
//
//ros::Publisher pub_gyroX("gyroX", &magX);
//ros::Publisher pub_gyroY("gyroY", &magY);
//ros::Publisher pub_gyroZ("gyroZ", &magZ);
//
//ros::Publisher pub_gyroX("gyroX", &gyroX);
//ros::Publisher pub_gyroY("gyroY", &gyroY);
//ros::Publisher pub_gyroZ("gyroZ", &gyroZ);
//
//ros::Publisher pub_tempC("tempC", &tempC);

//ros::Publisher pub_roll("roll", &roll);
//ros::Publisher pub_pitch("pitch", &pitch);
//ros::Publisher pub_yaw("yaw", &yaw);

ros::NodeHandle nh;

void setup() 
{
//  Serial.begin(115200);
//
//  while (!Serial) {
//    delay(1); // will pause Zero, Leonardo, etc until serial console opens
//  }
//  
//  Serial.println("LSM9DS1 data read demo");
//  
//  // Try to initialise and warn if we couldn't detect the chip
//  if (!lsm.begin())
//  {
//    Serial.println("Oops ... unable to initialize the LSM9DS1. Check your wiring!");
//    while (1);
//  }
//  Serial.println("Found LSM9DS1 9DOF");

  // helper to just set the default scaling we want, see above!
  setupSensor();

  //Initiate ROS-node
  nh.initNode();

  nh.advertise(pub_accelX);
  nh.advertise(pub_accelY);
  nh.advertise(pub_accelZ);
//  
//  nh.advertise(pub_magX);
//  nh.advertise(pub_magY);
//  nh.advertise(pub_magZ);
//  
//  nh.advertise(pub_gyroX);
//  nh.advertise(pub_gyroY);
//  nh.advertise(pub_gyroZ);
//  
//  nh.advertise(pub_tempC);
//
//  nh.advertise(pub_roll);
//  nh.advertise(pub_pitch);
//  nh.advertise(pub_yaw);
}

void loop() 
{
  lsm.read();  /* ask it to read in the data */ 

  /* Get a new sensor event */ 
  sensors_event_t a, m, g, temp;

  lsm.getEvent(&a, &m, &g, &temp); 

  accelX.data = a.acceleration.x;
  accelY.data = a.acceleration.y;
  accelZ.data = a.acceleration.z;
//
//  gyroX.data = g.gyro.x;
//  gyroY.data = g.gyro.y;
//  gyroZ.data = g.gyro.z;
//
//  tempC.data = temp.temperature;
//
//  pitch.data = 180 * atan (a.acceleration.x/sqrt(a.acceleration.y*a.acceleration.y + a.acceleration.z*a.acceleration.z))/M_PI;
//  roll.data = 180 * atan (a.acceleration.y/sqrt(a.acceleration.x*a.acceleration.x + a.acceleration.z*a.acceleration.z))/M_PI;
//  yaw.data = 180 * atan (a.acceleration.z/sqrt(a.acceleration.x*a.acceleration.x + a.acceleration.z*a.acceleration.z))/M_PI;
//
  pub_accelX.publish(&accelX);
  pub_accelY.publish(&accelY);
  pub_accelZ.publish(&accelZ);
//  
//  pub_gyroX.publish(&gyroX);
//  pub_gyroY.publish(&gyroY);
//  pub_gyroZ.publish(&gyroZ);
//
//  pub_tempC.publish(&tempC);
//
//  pub_roll.publish(&roll);
//  pub_pitch.publish(&pitch);
//  pub_yaw.publish(&yaw);
  
  nh.spinOnce();
}
