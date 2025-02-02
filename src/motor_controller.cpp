#include "motor_controller.hpp"

#include <vector>
#include <math.h>

float WHEEL_RADIUS = 0.03;
float L_X = 0.075;
float L_Y = 0.05;

extern int steps_per_rev;
extern int microsteps;

float radiansToSteps(float radians){
    return radians * steps_per_rev * microsteps / (2 * M_PI);
}

std::vector<float> wheelVelocitiesFromCartesian(float v_x, float v_y, float omega){
    std::vector<float> wheel_velocities(4);

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