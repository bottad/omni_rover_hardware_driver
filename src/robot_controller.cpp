#include "robot_controller.hpp"
#include "hardware_config.hpp"
#include "motor_control.hpp"

RobotController::RobotController() : mode_(MODE_IDLE), serialHandler_(Serial){
    frame_[8][12] = {0};
}

void RobotController::initiate() {
    serialHandler_.begin(115200);

    matrix_.begin();
    
    return;
}

void RobotController::update() {
    auto maybeMessage = serialHandler_.handleSerialInput();

    unsigned long now = millis();

    if (maybeMessage) {// Check if valid message from the serial handler received
        const SerialMessage& msg = *maybeMessage;

        switch (msg.type) {
            case MSG:
                // Handle a generic message
                break;
            case CMD_SUPER:
                onSuperCommand(msg.content.c_str(), msg.content.length());
                break;
            case CMD_VELOCITY:
                onVelocityCommand(msg.content.c_str(), msg.content.length());
                break;
            case ERR:
                // Handle error message
                break;
            default:
                // Unknown message type
                serialHandler_.sendError("Unknown message type");
                break;
        }

        lastCommandTime_ = now; // Update time if we command received
    }

    if (millis() - lastCommandTime_ > watchDogTimer_) {
        resetMotorVelocities();
        frame_[8][12] = {0};
    }

    if (now - lastBatteryUpdateTime_ >= batteryReportInterval_) {
        uint16_t voltage = readBatteryVoltage();

        uint8_t data[2];
        data[0] = (voltage >> 8) & 0xFF; // high byte
        data[1] = voltage & 0xFF;        // low byte

        serialHandler_.sendCommand(CMD_BATTERY, data, 2);

        lastBatteryUpdateTime_ = now;
    }

    matrix_.renderBitmap(frame_, 8, 12);
    runMotors();
}

void RobotController::onSuperCommand(const char* content, size_t length) {
    if (length != 1) {
        char errorMsg[40];
        snprintf(errorMsg, sizeof(errorMsg), "Invalid super command length: %u", static_cast<unsigned>(length));
        serialHandler_.sendError(errorMsg);
        return;
    }

    char commandType = content[0];

    switch (commandType) {
        case MODE_IDLE: // Idle mode
            mode_ = MODE_IDLE;
            resetMotorVelocities();
            break;
        case MODE_VELOCITY: // Manual mode
            mode_ = MODE_VELOCITY;
            break;
        default:
            serialHandler_.sendError("Unknown super command type");
            mode_ = MODE_IDLE; // Reset to idle mode on unknown command
            resetMotorVelocities();
            break;
    }
}

void RobotController::onVelocityCommand(const char* content, size_t length) {
    if (length != 6) {
        char errorMsg[50];
        snprintf(errorMsg, sizeof(errorMsg), "Invalid speed command length: %u", (unsigned)length);
        serialHandler_.sendError(errorMsg);
        return;
    }

    // Read as uint16_t (little-endian)
    uint16_t raw_vx   = (uint8_t)content[0] | ((uint8_t)content[1] << 8);
    uint16_t raw_vy   = (uint8_t)content[2] | ((uint8_t)content[3] << 8);
    uint16_t raw_rotz = (uint8_t)content[4] | ((uint8_t)content[5] << 8);

    drawJoystickRegion(raw_vx, raw_vy);

    if (mode_ != MODE_VELOCITY) {
        return;
    }

    // Convert raw joystick values [0, 65535] to float in the range [-1.0, 1.0
    float v_x = (static_cast<float>(raw_vx) - 32768.0f) / 32768.0f;
    float v_y = (static_cast<float>(raw_vy) - 32768.0f) / 32768.0f;
    float rot_z = (static_cast<float>(raw_rotz) - 32768.0f) / 32768.0f;

    applyWheelVelocities(wheelVelocitiesFromCartesian(v_x, v_y, rot_z));

    // Optional debug print
    // char msg[64];
    // snprintf(msg, sizeof(msg), "v_x=%f v_y=%f rot=%f", vx, vy, rotz);
    // serialHandler_.sendMessage(msg);
}


uint16_t RobotController::readBatteryVoltage() {
    const float referenceVoltage = 5.0;      // ADC reference voltage (Arduino 5V)
    const float voltageDividerRatio = 4.2;   // Adjust according to your resistor divider
    
    int analogValue = analogRead(BATTERY_VOLTAGE_PIN);  // Read ADC (0 - 1023)
    
    // Calculate voltage at the battery terminals in volts
    float voltage = (analogValue / 1023.0f) * referenceVoltage * voltageDividerRatio;
    
    // Convert to millivolts and return as uint16_t
    uint16_t voltage_mV = static_cast<uint16_t>(voltage * 1000.0f);
    
    return voltage_mV;
}

void RobotController::drawJoystickRegion(uint16_t v_x, uint16_t v_y) {
    memset(frame_, 0, sizeof(frame_));  // Clear the global frame array

    // Define midpoint and tolerance threshold
    const uint16_t midpoint = 32768;
    const uint16_t threshold = 3500;  // threshold for "close to center"

    // Check if both coordinates are near the midpoint ± threshold
    bool near_center_x = (v_x >= (midpoint - threshold)) && (v_x <= (midpoint + threshold));
    bool near_center_y = (v_y >= (midpoint - threshold)) && (v_y <= (midpoint + threshold));

    if (near_center_x && near_center_y) {
        // Light the 4 center LEDs
        frame_[3][5] = 1;
        frame_[3][6] = 1;
        frame_[4][5] = 1;
        frame_[4][6] = 1;
    } else {
        // Map the x,y coordinates normally to matrix range
        int x = map(v_x, 0, 65535, 0, 11);
        int y = map(v_y, 0, 65535, 0, 7);

        frame_[y][x] = 1;
    }
}
