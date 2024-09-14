#include "motor_controller.hpp"

float WHEEL_RADIUS = 0.03;
float L_X = 0.075;
float L_Y = 0.05;

std::vector<float> get_wheel_velocities(float v_x, float v_y, float omega){
    std::vector<float> wheel_velocities(4);

    float omega_fl = 1/WHEEL_RADIUS * (v_x - v_y - (L_X + L_Y) * omega);
    float omega_fr = 1/WHEEL_RADIUS * (v_x + v_y + (L_X + L_Y) * omega);
    float omega_rl = 1/WHEEL_RADIUS * (v_x + v_y - (L_X + L_Y) * omega);
    float omega_rr = 1/WHEEL_RADIUS * (v_x - v_y + (L_X + L_Y) * omega);

    wheel_velocities[0] = omega_fl;
    wheel_velocities[1] = omega_fr;
    wheel_velocities[2] = omega_rl;
    wheel_velocities[3] = omega_rr;

    return wheel_velocities;
}