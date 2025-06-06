#ifndef SERIAL_HANDLER_HPP
#define SERIAL_HANDLER_HPP

#include <Arduino.h>

#define LED_PIN 13 // Built in LED

static const char START_SYMBOL = '$';
static const char END_SYMBOL = '\n';
static const size_t MAX_MESSAGE_LENGTH = 64;
static const unsigned long serialTimeout = 100; // ms

// Define message types
const char MSG = 'M';
const char CMD_SPEED = 'S';
const char ERR = 'E';

extern unsigned long lastCommandTime;

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
    HardwareSerial& serialPort; // Reference to the serial port

    char messageBuffer[MAX_MESSAGE_LENGTH];
    size_t bufferIndex;
    bool messageComplete;
    bool messageStarted;

    unsigned long lastCharTime = 0;

    void onSpeedCommand(const char* content, size_t length);
    void resetParserState();
};

#endif // SERIAL_HANDLER_HPP