/*
  RobotOpen.cpp - Library implementation of the RobotOpen Hardware found at www.RobotOpen.biz
  Created by Eric Barch, September 27, 2012.
*/

#include "Arduino.h"
#include <SPI.h>
#include <avr/wdt.h>
#include "RobotOpenGS.h"


// Pointers to loop callbacks
static LoopCallback *whileTimedTasks;

// Robot specific stuff
static boolean _enabled = true;             // Tells us if the robot is enabled or disabled
static uint8_t _controller_state = 3;       // 1 - NC, 2 - Disabled, 3 - Enabled (sent over SPI to coprocessor)

// sent via SPI to coprocessor
static uint8_t _pwmStates[12];
static uint8_t _solenoidStates[8];

// Class constructor
RobotOpenClass RobotOpen;


void RobotOpenClass::begin(LoopCallback *timedtasksCallback) {
    // Setup callbacks
    whileTimedTasks = timedtasksCallback;

    // we do NOT want to talk to the coprocessor yet
    pinMode(9, OUTPUT);
    digitalWrite(9, HIGH);

    // neutral out all PWMs
    for (int i = 0; i < 12; i++) {
        _pwmStates[i] = 127;
    }
    // disable all Solenoids
    for (int i = 0; i < 8; i++) {
        _solenoidStates[i] = 0;
    }
    xmitCoprocessor();

    // watchdog go!
    wdt_enable(WDTO_250MS);
}

void RobotOpenClass::beginCoprocessor() {
    // for use w/ stm32
    SPI.setClockDivider(SPI_CLOCK_DIV16);

    // enable Slave Select
    digitalWrite(9, LOW);

    // coprocessor activate
    SPI.transfer(0xFF);
    SPI.transfer(0x7F);
}

void RobotOpenClass::endCoprocessor() {
    // disable Slave Select
    digitalWrite(9, HIGH);
}

void RobotOpenClass::xmitCoprocessor() {
    // this function sends PWM, solenoid, and enable state to the coprocessor

    // begin coprocessor transaction
    beginCoprocessor();

    // set controller state OPCODE
    SPI.transfer(COPROCESSOR_OP_SET_CONTROLLER_STATE);
  
    // write PWMs
    for (uint8_t i=0; i<12; i++) {
        SPI.transfer(_pwmStates[i]);
    }

    // write solenoids
    for (uint8_t i=0; i<8; i++) {
        SPI.transfer(_solenoidStates[i]);
    }

    // write LED state
    SPI.transfer(_controller_state);

    // end coprocessor transaction
    endCoprocessor();
}

void RobotOpenClass::attachDetachPWM(byte pwmChannel, bool attach) {
    // begin coprocessor transaction
    beginCoprocessor();

    // attach/detach OPCODE
    if (attach)
        SPI.transfer(COPROCESSOR_OP_ATTACH_PWM);
    else
        SPI.transfer(COPROCESSOR_OP_DETACH_PWM);

    // write PWM chan
    SPI.transfer(pwmChannel);

    // end coprocessor transaction
    endCoprocessor();
}

void RobotOpenClass::detachPWM(byte pwmChannel) {
    attachDetachPWM(pwmChannel, false);
}

void RobotOpenClass::attachPWM(byte pwmChannel) {
    attachDetachPWM(pwmChannel, true);
}

void RobotOpenClass::sync() {
    // feed watchdog
    wdt_reset();

    // send update to coprocessor
    xmitCoprocessor();

    // Run user loops
    if (whileTimedTasks)
        whileTimedTasks();
}

void RobotOpenClass::writePWM(byte channel, uint8_t pwmVal) {
    if (channel < 12) {
        _pwmStates[channel] = pwmVal;
    }
}

long RobotOpenClass::readEncoder(byte channel) {
    // begin coprocessor transaction
    beginCoprocessor();

    // read encoder OPCODE
    SPI.transfer(COPROCESSOR_OP_GET_ENCODER);

    // send encoder channel
    SPI.transfer(channel);

    // coprocessor buffer byte
    SPI.transfer(0x04);

    // grab encoder count off SPI bus
    long encoderCount = (SPI.transfer(0x04) << 24) | (SPI.transfer(0x04) << 16) | (SPI.transfer(0x04) << 8) | (SPI.transfer(0x04) & 0xFF);

    // end coprocessor transaction
    endCoprocessor();

    return encoderCount;
}

float RobotOpenClass::readEncoderCPS(byte channel) {
    // begin coprocessor transaction
    beginCoprocessor();

    // read encoder CPS OPCODE
    SPI.transfer(COPROCESSOR_OP_GET_ENCODER_CPS);

    // send encoder channel
    SPI.transfer(channel);

    // coprocessor buffer byte
    SPI.transfer(0x04);

    // grab encoder count off SPI bus
    union {
        float f;
        uint8_t b[4];
    } u;
    u.b[0] = SPI.transfer(0x04);
    u.b[1] = SPI.transfer(0x04);
    u.b[2] = SPI.transfer(0x04);
    u.b[3] = SPI.transfer(0x04);

    // end coprocessor transaction
    endCoprocessor();

    return u.f;
}

void RobotOpenClass::setEncoderSensitivity(byte channel, uint16_t sensitivity) {
    // begin coprocessor transaction
    beginCoprocessor();

    // set encoder sensitivty OPCODE
    SPI.transfer(COPROCESSOR_OP_SET_ENCODER_SENSITIVITY);

    // send encoder channel
    SPI.transfer(channel);

    // send sensitivty
    SPI.transfer((sensitivity << 8) & 0xFF);
    SPI.transfer(sensitivity & 0xFF);

    // end coprocessor transaction
    endCoprocessor();
}

void RobotOpenClass::setEncoderSamplesToAverage(byte channel, uint8_t samples) {
    // begin coprocessor transaction
    beginCoprocessor();

    // set encoder sensitivty OPCODE
    SPI.transfer(COPROCESSOR_OP_SET_ENCODER_AVERAGE);

    // send encoder channel
    SPI.transfer(channel);

    // send sensitivty
    SPI.transfer(samples);

    // end coprocessor transaction
    endCoprocessor();
}

void RobotOpenClass::resetEncoder(byte channel) {
    // begin coprocessor transaction
    beginCoprocessor();

    // reset encoder OPCODE
    SPI.transfer(COPROCESSOR_OP_RESET_ENCODER);

    // send encoder channel
    SPI.transfer(channel);

    // end coprocessor transaction
    endCoprocessor();
}

void RobotOpenClass::writeSolenoid(byte channel, uint8_t state) {
    if (channel < 8) {
        _solenoidStates[channel] = state;
    }
}

boolean RobotOpenClass::enabled() {
    return _enabled;
}