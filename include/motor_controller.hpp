#include <vector>

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