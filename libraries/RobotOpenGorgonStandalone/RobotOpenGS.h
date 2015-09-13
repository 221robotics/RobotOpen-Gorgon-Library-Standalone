/*
  RobotOpen.h - Library implementation of the RobotOpen Hardware found at www.RobotOpen.biz
  Created by Eric Barch, September 27, 2012.
*/

#ifndef RobotOpen_h
#define RobotOpen_h

#include "Arduino.h"
#include <ROPWM.h>
#include <ROAnalog.h>
#include <RODigitalIO.h>
#include <ROSolenoid.h>
#include <ROStatus.h>
#include <ROTimer.h>
#include <ROEncoder.h>


// coprocessor opcodes
#define COPROCESSOR_OP_SET_CONTROLLER_STATE     0x01
#define COPROCESSOR_OP_GET_ENCODER              0x02
#define COPROCESSOR_OP_RESET_ENCODER            0x03
#define COPROCESSOR_OP_GET_ENCODER_CPS          0x04
#define COPROCESSOR_OP_SET_ENCODER_SENSITIVITY  0x05
#define COPROCESSOR_OP_ATTACH_PWM               0x06
#define COPROCESSOR_OP_DETACH_PWM               0x07
#define COPROCESSOR_OP_SET_ENCODER_AVERAGE      0x08
#define COPROCESSOR_OP_RESET                    0x64


typedef void LoopCallback();


class RobotOpenClass {
public:
    // Fire up the RobotOpen object and get things running
    static void begin(LoopCallback *timedtasksCallback);
    
    // Update internal state
    static void sync();
    
    // Tells us if the robot is enabled
    static boolean enabled();

    // Calls for ROShield
    static void xmitCoprocessor();

    static void writePWM(byte channel, byte pwmVal);
    static void writeSolenoid(byte channel, uint8_t state);

    static int32_t readEncoder(byte channel);
    static float readEncoderCPS(byte channel);
    static void setEncoderSensitivity(byte channel, uint16_t sensitivity);
    static void setEncoderSamplesToAverage(byte channel, uint8_t samples);
    static void resetEncoder(byte channel);

    static void detachPWM(byte pwmChannel);
    static void attachPWM(byte pwmChannel);

private:
    // Attach or Detach a PWM pin
    static void attachDetachPWM(byte pwmChannel, bool attach);

    // Coprocessor methods
    static void beginCoprocessor();
    static void endCoprocessor();
};

extern RobotOpenClass RobotOpen;

#endif