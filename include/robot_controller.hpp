#ifndef ROBOT_CONTROLLER_HPP
#define ROBOT_CONTROLLER_HPP

#include <Arduino.h>
#include "Arduino_LED_Matrix.h"

#include "serial_handler.hpp"
#include "messages.hpp"

/**
 * @brief Controls the main behavior of the robot system.
 * 
 * The RobotController handles serial communication, motor control, watchdog timeout behavior, 
 * battery status reporting, and visual feedback via the built-in LED matrix. It acts as the 
 * central logic unit that is updated continuously in the main loop.
 */
class RobotController {
private:
    OperatingMode mode_;                                ///< Current operating mode of the robot (e.g., idle or velocity control).

    SerialHandler serialHandler_;                       ///< Handles serial input/output and message parsing.

    ArduinoLEDMatrix matrix_;                           ///< Interface to the 8x12 LED matrix display.

    byte frame_[8][12];                                 ///< Current frame data to be rendered on the LED matrix.

    unsigned long lastCommandTime_ = 0;                 ///< Timestamp of the last received velocity/super command.
    const unsigned long watchDogTimer_ = 500;           ///< Duration (ms) to wait for a new command before stopping motors.

    unsigned long lastBatteryUpdateTime_ = 0;           ///< Timestamp of the last battery status update.
    const unsigned long batteryReportInterval_ = 5000;  ///< Interval (ms) at which to report battery voltage.

public:
    /**
     * @brief Constructs a RobotController object.
     * 
     * Initializes internal variables and prepares the controller for use.
     */
    RobotController();

    /**
     * @brief Initializes the robot controller.
     * 
     * This function sets up the serial communication and initializes the LED matrix.
     */
    void initiate();

    /**
     * @brief Updates the robot controller state.
     * 
     * This function performs several tasks each time it is called:
     * - Checks for and processes incoming serial messages (e.g., velocity commands, super commands).
     * - Manages a watchdog timer that resets motor velocities if no commands are received within a configured timeout.
     * - Periodically reads the battery voltage, and sends the status back via serial.
     * - Updates the LED matrix display with the current frame.
     * - Runs the motors with the latest velocity commands.
     */
    void update();
    
    /**
     * @brief Handles the super command.
     * 
     * This function processes the super command to switch between different operating modes.
     * 
     * @param content Pointer to the command content after the 'S' command byte.
     * @param length Length of the command content.
     */
    void onSuperCommand(const char* content, size_t length);

    /**
     * @brief Handles the velocity command.
     * 
     * This function processes the velocity command to set wheel velocities based on joystick input. 
     * If the robot is in MODE_VELOCITY, it applies the velocities to the motors.
     * 
     * @param content Pointer to the command content after the 'V' command byte.
     * @param length Length of the command content.
     */
    void onVelocityCommand(const char* content, size_t length);


    /**
     * @brief Reads and calculates the battery voltage in millivolts.
     * 
     * This function reads the analog voltage from the configured battery voltage pin,
     * applies the voltage divider scaling, and converts the result to millivolts.
     * 
     * @return uint16_t Battery voltage in millivolts (mV).
     */
    uint16_t readBatteryVoltage();

    /**
     * @brief Draws the joystick region on the LED matrix.
     * 
     * This function lights up the LEDs based on the joystick coordinates (v_x, v_y).
     * If the coordinates are near the center, it lights up the center LEDs.
     * Otherwise, it maps the coordinates to the matrix range and lights up the corresponding LED.
     * 
     * @param v_x Joystick x-coordinate (0 to 65535).
     * @param v_y Joystick y-coordinate (0 to 65535).
     */
    void drawJoystickRegion(uint16_t v_x, uint16_t v_y);
};

#endif // ROBOT_CONTROLLER_HPP
