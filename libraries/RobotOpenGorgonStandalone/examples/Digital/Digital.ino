#include <SPI.h>
#include <RobotOpenGS.h>



/* I/O Setup */
RODigitalIO dig0In(0, INPUT);    // DIO channel 0, input mode


void setup()
{
  /* Initial Serial */
  Serial.begin(9600);

  /* Initiate comms */
  RobotOpen.begin(&timedtasks);
}


/* This loop ALWAYS runs */
void timedtasks() {
  if (dig0In.read())
    Serial.println("Digital 0 High!");
}


// !!! DO NOT MODIFY !!!
void loop() {
  RobotOpen.sync();
}