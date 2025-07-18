#ifndef MESSAGES_HPP
#define MESSAGES_HPP

#include <cstddef>
#include <string>

enum OperatingMode {
    MODE_IDLE,
    MODE_VELOCITY
    // ... Future modes ...
};

// Message symbols
constexpr char START_SYMBOL = '$';
constexpr char END_SYMBOL = '\n';
constexpr size_t MAX_MESSAGE_LENGTH = 30;

// Message types
constexpr char MSG = 'M';
constexpr char CMD_SUPER = 'S';
constexpr char CMD_VELOCITY = 'V';
constexpr char ERR = 'E';

struct SerialMessage {
  char type;
  std::string content;
};

#endif // MESSAGES_HPP