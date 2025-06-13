#ifndef SERIAL_HANDLER_HPP
#define SERIAL_HANDLER_HPP

#include <Arduino.h>
#include "Arduino_LED_Matrix.h"

#define LED_PIN 13 // Built in LED

extern byte frame[8][12];

static const char START_SYMBOL = '$';
static const char END_SYMBOL = '\n';
static const size_t MAX_MESSAGE_LENGTH = 30;

// Define message types
const char MSG = 'M';
const char CMD_SUPER = 'S';
const char CMD_VELOCITY = 'V';
const char ERR = 'E';

enum ParserState {
    IDLE,
    RECEIVING,
    COMPLETE
};

class SerialHandler {
public:
    SerialHandler(HardwareSerial& serial);
    void begin(unsigned long baud);
    void handleSerialInput();
    void processMessage();

    // Methods for sending messages, commands, and errors
    void sendMessage(const char* message);
    void sendCommand(const char commandType, const char* command);
    void sendError(const char* error);

private:
    HardwareSerial& serialPort_; // Reference to the serial port

    char messageBuffer_[MAX_MESSAGE_LENGTH];
    size_t bufferIndex_;
    ParserState parserState_;

    void onVelocityCommand(const char* content, size_t length);
    void resetParserState();

    void drawJoystickRegion(uint16_t v_x, uint16_t v_y);
};

#endif // SERIAL_HANDLER_HPP