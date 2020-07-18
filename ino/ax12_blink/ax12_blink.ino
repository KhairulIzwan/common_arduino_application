/*
 * Example of a simple Blink using the AX-12A built-in LED
 */

#include "Arduino.h"
#include "AX12A.h"

#define DirectionPin 	(10u)
#define BaudRate  		(1000000ul)
#define IDB2				(10u)
#define IDB1        (9u)
#define IDA1        (8u)
#define IDA2        (7u)

void setup()
{
	ax12a.begin(BaudRate, DirectionPin, &Serial);
}

void loop()
{
	ax12a.ledStatus(IDA1, ON);
	delay(1000);
	ax12a.ledStatus(IDA1, OFF);
	delay(1000);
  ax12a.ledStatus(IDA2, ON);
  delay(1000);
  ax12a.ledStatus(IDA2, OFF);
  delay(1000);
  ax12a.ledStatus(IDB1, ON);
  delay(1000);
  ax12a.ledStatus(IDB1, OFF);
  delay(1000);
  ax12a.ledStatus(IDB2, ON);
  delay(1000);
  ax12a.ledStatus(IDB2, OFF);
}
