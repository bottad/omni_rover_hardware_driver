#ifndef SERIAL_HANDLER_HPP
#define SERIAL_HANDLER_HPP

#include <Arduino.h>
#include "meassages.hpp"

#include <optional>

enum ParserState {
    IDLE,
    RECEIVING,
    COMPLETE
};

class SerialHandler {
public:
    SerialHandler(HardwareSerial& serial);
    void begin(unsigned long baud);
    std::optional<SerialMessage> handleSerialInput();

    // Methods for sending messages, commands, and errors
    void sendMessage(const char* message);
    void sendCommand(const char commandType, const char* command);
    void sendError(const char* error);

private:
    HardwareSerial& serialPort_; // Reference to the serial port

    char messageBuffer_[MAX_MESSAGE_LENGTH];
    size_t bufferIndex_;
    ParserState parserState_;

    std::optional<SerialMessage> processMessage();
    void resetParserState();
};

#endif // SERIAL_HANDLER_HPP