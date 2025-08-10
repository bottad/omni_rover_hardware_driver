#ifndef MOTOR_CONTROL_HPP
#define MOTOR_CONTROL_HPP

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
 * @brief Converts Cartesian velocity commands (v_x, v_y, omega) to wheel velocities in steps per second.
 * 
 * This function converts unitless Cartesian velocity commands for a four-wheeled omnidirectional robot
 * into individual wheel velocities expressed in steps per second. The inputs v_x, v_y, and omega 
 * represent normalized commands in the range [-1, 1], which are scaled by configured maximum velocity 
 * multipliers (xMaxMultiplier, yMaxMultiplier, wMaxMultiplier) to convert to physical units.
 * 
 * After calculating the wheel velocities based on the robot geometry and wheel radius, the function 
 * checks if any computed wheel velocity exceeds the defined maximum stepper motor speed (MAX_SPEED). 
 * If so, all wheel velocities are proportionally scaled down to ensure the maximum speed constraint 
 * is not violated. This preserves the relative velocity ratios, maintaining the intended motion 
 * direction but limiting the overall speed.
 * 
 * @param v_x Unitless linear velocity command in the x direction, expected in [-1, 1].
 * @param v_y Unitless linear velocity command in the y direction, expected in [-1, 1].
 * @param omega Unitless angular velocity command about the z-axis, expected in [-1, 1].
 * @return A vector of four floats representing wheel velocities in steps per second, scaled if necessary
 *         to not exceed the motor controller's maximum speed.
 */
std::vector<float> wheelVelocitiesFromCartesian(float v_x, float v_y, float omega);

#endif // MOTOR_CONTROL_HPP