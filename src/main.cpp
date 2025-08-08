#include <Arduino.h>
#include <AccelStepper.h>

#include <string>
#include <vector>

#include "hardware_config.hpp"
#include "robot_controller.hpp"

// ################################################################################################### //
//                                        Definitions
// ################################################################################################### //

AccelStepper motorL2(1, MOTOR_L2_STEP_PIN, MOTOR_L2_DIR_PIN);
AccelStepper motorL1(1, MOTOR_L1_STEP_PIN, MOTOR_L1_DIR_PIN);
AccelStepper motorR2(1, MOTOR_R2_STEP_PIN, MOTOR_R2_DIR_PIN);
AccelStepper motorR1(1, MOTOR_R1_STEP_PIN, MOTOR_R1_DIR_PIN);

RobotController robotController;

// ################################################################################################### //
//                                             Declarations
// ################################################################################################### //


// ################################################################################################### //
//                                             Setup
// ################################################################################################### //

void setup() {
  analogReadResolution(12);     // Set ADC resolution to 12 bits (0-4095)
  analogReference(AR_INTERNAL); // Set ADC reference to internal ~1.1V

  robotController.initiate();

  pinMode(MOTORS_ENABLE_PIN, OUTPUT);

  motorL1.setEnablePin(MOTORS_ENABLE_PIN);
  motorL1.setPinsInverted(false, false, true);
  motorL1.setAcceleration(100);
  motorL1.setMaxSpeed(MAX_SPEED);
  motorL1.setSpeed(0);
  motorL1.enableOutputs();

  motorR1.setEnablePin(MOTORS_ENABLE_PIN);
  motorR1.setPinsInverted(false, false, true);
  motorR1.setAcceleration(100);
  motorR1.setMaxSpeed(MAX_SPEED);
  motorR1.setSpeed(0);
  motorR1.enableOutputs();

  motorL2.setEnablePin(MOTORS_ENABLE_PIN);
  motorL2.setPinsInverted(false, false, true);
  motorL2.setAcceleration(100);
  motorL2.setMaxSpeed(MAX_SPEED);
  motorL2.setSpeed(0);
  motorL2.enableOutputs();

  motorR2.setEnablePin(MOTORS_ENABLE_PIN);
  motorR2.setPinsInverted(false, false, true);
  motorR2.setAcceleration(100);
  motorR2.setMaxSpeed(MAX_SPEED);
  motorR2.setSpeed(0);
  motorR2.enableOutputs();
}


// ################################################################################################### //
//                                        Main Loop
// ################################################################################################### //

void loop()
{
  robotController.update();
}

// ################################################################################################### //
//                                     Function Definitions
// ################################################################################################### //

