#ifndef HARDWARE_CONFIG_HPP
#define HARDWARE_CONFIG_HPP

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

// Built in LED
#define LED_PIN 13

#define BATTERY_VOLTAGE_PIN A0

// Motor configuration
constexpr int STEPS_PER_REV = 200;  //  1.8° per step
constexpr int MICROSTEPS = 8;       //  8 microsteps per step
constexpr int MAX_SPEED = 2000;     // Max speed in steps per second, max reliable 4000

#endif // HARDWARE_CONFIG_HPP
