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

// BLE setup:
const char * CCCD_DESCRIPTOR_UUID = "00002902-0000-1000-8000-00805F9B34FB";
const char * deviceServiceUUID = "19b10000-e8f2-537e-4f6c-d104768a1214";
const char * statusCharacteristicUUID = "19b10001-e8f2-537e-4f6c-d104768a1215";
const char * controlCharacteristicUUID = "19b10001-e8f2-537e-4f6c-d104768a1216";

BLEService roverService(deviceServiceUUID);
BLEIntCharacteristic statusResponseCharacteristic(statusCharacteristicUUID, BLENotify);
BLECharacteristic controlInputCharacteristic(controlCharacteristicUUID, BLEWrite, 20);

uint8_t status = 0;

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

  if(!BLE.begin()){
    Serial.println("[ERROR]\tstarting BLE module failed!");
    while(true);
  }

  BLE.setDeviceName("Omni-Rover");
  BLE.setLocalName("Omni-Rover");

  BLE.setAdvertisedService(roverService);
  roverService.addCharacteristic(statusResponseCharacteristic);
  roverService.addCharacteristic(controlInputCharacteristic);
  //statusResponseCharacteristic.descriptor(CCCD_DESCRIPTOR_UUID);
  BLE.addService(roverService);

  statusResponseCharacteristic.writeValue(0);
  controlInputCharacteristic.setValue(0);

  BLE.advertise();
  Serial.println("[INFO]\tStart Scanning...");

}


// ################################################################################################### //
//                                        Main Loop
// ################################################################################################### //

void loop()
{

  if (status%2==0){
    Serial.print("      \t---\r");
  } 
  else {
    Serial.print("      \t-|-\r");
  }
  // Discovering central device
  BLEDevice central = BLE.central();
  delay(100);

  status++;

      if (status >= 3) {
        status = 0;
      }

  if(central){
    Serial.println("[INFO]\tConnected!");

    while(central.connected()){

      Serial.println(status);
      
      statusResponseCharacteristic.writeValue(status);

      status++;
      delay(1000);

      if (status >= 3) {
        status = 0;
      }
    }
  }
}