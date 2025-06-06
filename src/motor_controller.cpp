#include "motor_controller.hpp"

#include <vector>
#include <math.h>

#include <AccelStepper.h>

float WHEEL_RADIUS = 0.03;
float L_X = 0.075;
float L_Y = 0.05;

float vMaxMultiplier = 6; // 24 for absolut max of 4000 steps/s, 6 for 1000 steps/s

extern int steps_per_rev;
extern int microsteps;

extern AccelStepper motorL2;
extern AccelStepper motorL1;
extern AccelStepper motorR2;
extern AccelStepper motorR1;

void applyWheelVelocities(std::vector<float> wheel_velocities){
  motorL1.setSpeed(wheel_velocities[0]);
  motorR1.setSpeed(wheel_velocities[1]);
  motorL2.setSpeed(wheel_velocities[2]);
  motorR2.setSpeed(wheel_velocities[3]);
}

void resetMotorVelocities(){
  motorL1.setSpeed(0);
  motorR1.setSpeed(0);
  motorL2.setSpeed(0);
  motorR2.setSpeed(0);
}

void runMotors(){
  motorL1.runSpeed();
  motorR1.runSpeed();
  motorL2.runSpeed();
  motorR2.runSpeed();
}

float radiansToSteps(float radians){
    return radians * steps_per_rev * microsteps / (2 * M_PI);
}

std::vector<float> wheelVelocitiesFromCartesian(float v_x, float v_y, float omega){
    std::vector<float> wheel_velocities(4);

    v_x *= vMaxMultiplier;
    v_y *= vMaxMultiplier;
    omega *= vMaxMultiplier;

    float omega_fl = 1/WHEEL_RADIUS * (v_x - v_y - (L_X + L_Y) * omega);
    float omega_fr = 1/WHEEL_RADIUS * (v_x + v_y + (L_X + L_Y) * omega);
    float omega_rl = 1/WHEEL_RADIUS * (v_x + v_y - (L_X + L_Y) * omega);
    float omega_rr = 1/WHEEL_RADIUS * (v_x - v_y + (L_X + L_Y) * omega);

    wheel_velocities[0] = radiansToSteps(omega_fl);
    wheel_velocities[1] = radiansToSteps(omega_fr);
    wheel_velocities[2] = radiansToSteps(omega_rl);
    wheel_velocities[3] = radiansToSteps(omega_rr);

    return wheel_velocities;
}