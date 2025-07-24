#ifndef MESSAGES_HPP
#define MESSAGES_HPP

#include <cstddef>
#include <string>

// ------------------------------------------
// Operating Modes
// ------------------------------------------
enum OperatingMode {
    MODE_IDLE,
    MODE_VELOCITY
    // ... Future modes ...
};

// ------------------------------------------
// Serial Message Symbols
// ------------------------------------------
constexpr char START_SYMBOL = '$';
constexpr char END_SYMBOL = '\n';
constexpr size_t MAX_MESSAGE_LENGTH = 30;

// ------------------------------------------
// Message Types
// ------------------------------------------
constexpr char MSG           = 'M';  // Generic message
constexpr char CMD_SUPER     = 'S';  // High-level command
constexpr char CMD_VELOCITY  = 'V';  // Velocity command
constexpr char CMD_BATTERY   = 'B';  // Battery voltage report
constexpr char ERR           = 'E';  // Error message

// ------------------------------------------
// Parsed Serial Message Struct
// ------------------------------------------
struct SerialMessage {
    char type;
    std::string content;
};

#endif // MESSAGES_HPP