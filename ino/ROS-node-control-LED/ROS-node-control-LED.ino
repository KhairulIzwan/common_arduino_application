/*
 * Title: The ROS node that controls the LED
 * Author: https://www.intorobotics.com/how-to-use-rosserial-with-two-arduinos-and-raspberry-pi/
 * Description: The second Arduino board will turn ON and OFF the LED (pin 13) depending on the commands received from the ROS node running on the Pi (SBC) board.
 * 
 * Re-arrange the code in order to make it works! [Khairul Izwan (24-01-2020)]
 */

//include necessary library
#include <ros.h>
#include <std_msgs/Int32.h>

ros::NodeHandle nh;

void messageCb(const std_msgs::Int32& msg)
{
  if(msg.data ==1)
  {
    digitalWrite(13, HIGH);   //blink the led
  }
  else
  {
    digitalWrite(13, LOW);   //turn off the led
  }
}

ros::Subscriber<std_msgs::Int32> sub("/LED", &messageCb);

//put your setup code here, to run once:
void setup()
{
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
}

//put your main code here, to run repeatedly:
void loop()
{
  nh.spinOnce();
  delay(1000);
}
