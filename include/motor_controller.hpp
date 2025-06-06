#ifndef MOTOR_CONTROLLER_HPP
#define MOTOR_CONTROLLER_HPP

#include <vector>

/**
 * @brief Applies the specified wheel velocities to the motors.
 * 
 * This function takes a vector of four floats representing the desired wheel velocities for a
 * four-wheeled omnidirectional rover and applies them to the corresponding stepper motors.
 * 
 * @param wheel_velocities A vector of four floats representing the wheel velocities.
 */
void applyWheelVelocities(std::vector<float> wheel_velocities);

/**
 * @brief Resets the motor speeds to zero.
 * 
 * This function sets the speeds of all motors to zero, effectively stopping them.
 */
void resetMotorVelocities();

/**
 * @brief Runs the stepper motors at the specified speeds.
 * 
 * This function runs the stepper motors at the speeds set by the applyWheelVelocities function.
 */
void runMotors();

/**
 * @brief Converts radians to steps.
 * 
 * This function takes an angle in radians and converts it to the corresponding number of steps
 * for a stepper motor. The number of steps per revolution and the number of microsteps are taken
 * from the global variables steps_per_rev and microsteps, respectively.
 * 
 * @param radians The angle in radians.
 * @return The number of steps corresponding to the angle.
 */
float radiansToSteps(float radians);

/**
 * @brief Calculates wheel velocities from Cartesian coordinates.
 * 
 * This function takes the desired velocities in the x and y directions, as well as the angular
 * velocity, and calculates the corresponding wheel velocities for a four-wheeled omnidirectional
 * rover. The velocities are returned as a vector of four floats, representing the velocities
 * for the front-left, front-right, rear-left, and rear-right wheels, respectively.
 * 
 * @param v_x The velocity in the x direction.
 * @param v_y The velocity in the y direction.
 * @param omega The angular velocity.
 * @return A vector of four floats representing the wheel velocities.
 */
std::vector<float> wheelVelocitiesFromCartesian(float v_x, float v_y, float omega);

#endif // MOTOR_CONTROLLER_HPP