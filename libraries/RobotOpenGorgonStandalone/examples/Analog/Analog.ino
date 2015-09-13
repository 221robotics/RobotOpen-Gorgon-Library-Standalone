#include <SPI.h>
#include <RobotOpenGS.h>



/* I/O Setup */
ROAnalog analogZero(0);   	// Analog Channel 0
ROAnalog analogOne(1);		// Analog Channel 1


void setup()
{
  /* Initial Serial */
  Serial.begin(9600);

  /* Initiate comms */
  RobotOpen.begin(&timedtasks);
}


/* This loop ALWAYS runs */
void timedtasks() {
  Serial.println(analogZero.read());
  Serial.println(analogOne.read());
}


// !!! DO NOT MODIFY !!!
void loop() {
  RobotOpen.sync();
}