#include <Arduino.h>
#include <AccelStepper.h>
#include <ArduinoBLE.h>
#include <WiFiS3.h>

#include <string>
#include <vector>

#include "motor_controller.hpp"

// ################################################################################################### //
//                                        Definitions
// ################################################################################################### //

// All motors:
#define MOTORS_ENABLE_PIN 8

// On Arduino shield: X
#define MOTOR_L2_STEP_PIN 2
#define MOTOR_L2_DIR_PIN 5

// On Arduino shield: Y
#define MOTOR_L1_STEP_PIN 3
#define MOTOR_L1_DIR_PIN 6

// On Arduino shield: Z
#define MOTOR_R2_STEP_PIN 4
#define MOTOR_R2_DIR_PIN 7

// On Arduino shield: A
#define MOTOR_R1_STEP_PIN 12
#define MOTOR_R1_DIR_PIN 13

// General definitions
int steps_per_rev = 200;  //  1.8° per step
int microsteps = 8;

AccelStepper motorL2(1, MOTOR_L2_STEP_PIN, MOTOR_L2_DIR_PIN);
AccelStepper motorL1(1, MOTOR_L1_STEP_PIN, MOTOR_L1_DIR_PIN);
AccelStepper motorR2(1, MOTOR_R2_STEP_PIN, MOTOR_R2_DIR_PIN);
AccelStepper motorR1(1, MOTOR_R1_STEP_PIN, MOTOR_R1_DIR_PIN);

// ################################################################################################### //
//                                             Setup
// ################################################################################################### //

void setup() {
  Serial.begin(9600);

  pinMode(MOTORS_ENABLE_PIN, OUTPUT);

  motorL1.setEnablePin(MOTORS_ENABLE_PIN);
  motorL1.setPinsInverted(false, false, true);
  motorL1.setAcceleration(100);
  motorL1.setMaxSpeed(1000);                       // Max reliable 4000
  //motorL1.setSpeed(100);
  motorL1.enableOutputs();

  motorR1.setEnablePin(MOTORS_ENABLE_PIN);
  motorR1.setPinsInverted(false, false, true);
  motorR1.setAcceleration(100);
  motorR1.setMaxSpeed(1000);                       // Max reliable 4000
  //motorR1.setSpeed(100);
  motorR1.enableOutputs();

  motorL2.setEnablePin(MOTORS_ENABLE_PIN);
  motorL2.setPinsInverted(false, false, true);
  motorL2.setAcceleration(100);
  motorL2.setMaxSpeed(1000);                       // Max reliable 4000
  //motorL2.setSpeed(100);
  motorL2.enableOutputs();

  motorR2.setEnablePin(MOTORS_ENABLE_PIN);
  motorR2.setPinsInverted(false, false, true);
  motorR2.setAcceleration(100);
  motorR2.setMaxSpeed(1000);                       // Max reliable 4000
  //motorR2.setSpeed(100);
  motorR2.enableOutputs();
}


// ################################################################################################### //
//                                        Main Loop
// ################################################################################################### //

void loop()
{
  
}