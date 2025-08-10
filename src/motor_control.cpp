#include "motor_control.hpp"

#include <vector>
#include <math.h>

#include <AccelStepper.h>
#include "hardware_config.hpp"

constexpr float WHEEL_RADIUS = 0.03;
constexpr float L_X = 0.075;
constexpr float L_Y = 0.05;

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
  return radians * STEPS_PER_REV * MICROSTEPS / (2 * M_PI);
}

std::vector<float> wheelVelocitiesFromCartesian(float v_x, float v_y, float omega){
  std::vector<float> wheel_velocities(4);

  v_x *= xMaxMultiplier;
  v_y *= yMaxMultiplier;
  omega *= wMaxMultiplier;

  float L = L_X + L_Y;

  float omega_fl = 1/WHEEL_RADIUS * (v_x - v_y - L * omega);
  float omega_fr = 1/WHEEL_RADIUS * (v_x + v_y + L * omega);
  float omega_rl = 1/WHEEL_RADIUS * (v_x + v_y - L * omega);
  float omega_rr = 1/WHEEL_RADIUS * (v_x - v_y + L * omega);

  wheel_velocities[0] = radiansToSteps(omega_fl);
  wheel_velocities[1] = radiansToSteps(omega_fr);
  wheel_velocities[2] = radiansToSteps(omega_rl);
  wheel_velocities[3] = radiansToSteps(omega_rr);

  float max_wheel_speed = 0.0f;
  for (auto w : wheel_velocities) {
    if (std::fabs(w) > max_wheel_speed) max_wheel_speed = std::fabs(w);
  }

  if (max_wheel_speed > MAX_SPEED) {
    float scale = MAX_SPEED / max_wheel_speed;
    for (auto& w : wheel_velocities) {
      w *= scale;
    }
  }

  return wheel_velocities;
}