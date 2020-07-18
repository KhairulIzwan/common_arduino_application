/*
 * Title: Dual DC Motor Control with Encoder
 * Author: Khairul Izwan 22-03-2020
 * Description: Controlling Dual DC Motor with Encoder using
 * 10Amp 7V-30V DC Motor Driver Shield for Arduino (2 Channels)
 */

/* Parts ::
 * 1. DC Motor:: 12V 38RPM 5kgfcm Brushed DC Geared Motor 
 * :: https://my.cytron.io/p-12v-38rpm-5kgfcm-brushed-dc-geared-motor?search=12%2038rpm&description=1&src=search.list
 * 2. Driver:: Shield L298P Motor Driver with GPIO 
 * :: https://my.cytron.io/p-shield-l298p-motor-driver-with-gpio?src=search.list
 */
 
//Motor Pin Assignment
int DIRA = 4;
int PWMA = 5;
int DIRB = 12;
int PWMB = 10;

//Speed
int SPD = 50;

//put your setup code here, to run once:
void setup()
{
  
//Input/Output Pins Assigment
  pinMode(DIRA, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(DIRB, OUTPUT);
  pinMode(PWMB, OUTPUT);

}

//put your main code here, to run repeatedly:
void loop()
{

  digitalWrite(DIRA, HIGH);
  digitalWrite(DIRB, LOW);
  analogWrite(PWMA, SPD);
  analogWrite(PWMB, SPD);
  
}
