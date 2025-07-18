#ifndef ROBOT_CONTROLLER_HPP
#define ROBOT_CONTROLLER_HPP

#include <Arduino.h>
#include "Arduino_LED_Matrix.h"

#include "serial_handler.hpp"
#include "meassages.hpp"

extern byte frame[8][12];

class RobotController {
private:
    OperatingMode mode_;

    SerialHandler serialHandler_;

    ArduinoLEDMatrix matrix_;

    byte frame_[8][12];

    unsigned long lastCommandTime_ = 0;
    unsigned long watchDogTimer_ = 500; // 0.5 second

public:
    RobotController();
    void initiate();
    void update();
    
    void onSuperCommand(const char* content, size_t length);
    void onVelocityCommand(const char* content, size_t length);

    void drawJoystickRegion(uint16_t v_x, uint16_t v_y);
};

#endif // ROBOT_CONTROLLER_HPP