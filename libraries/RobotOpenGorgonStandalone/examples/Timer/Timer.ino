#include <SPI.h>
#include <RobotOpenGS.h>



/* Timer Setup */
ROTimer stepOne;   	// First timer step
ROTimer stepTwo;	// Second timer step
ROTimer loopMe;		// A looping timer


void setup()
{
  /* Initial Serial */
  Serial.begin(9600);

  /* Initiate comms */
  RobotOpen.begin(&timedtasks);

  stepOne.queue(0);
  loopMe.queue(0);
}


/* This loop ALWAYS runs */
void timedtasks() {
  if (stepOne.ready()) {
  	Serial.println("Step One!");
  	stepTwo.queue(1000);
  }
  if (stepTwo.ready()) {
  	Serial.println("Step Two!");
  	stepOne.queue(1000);
  }
  if (loopMe.ready()) {
  	Serial.println("I looped!");
  	loopMe.queue(5000);
  }
}


// !!! DO NOT MODIFY !!!
void loop() {
  RobotOpen.sync();
}