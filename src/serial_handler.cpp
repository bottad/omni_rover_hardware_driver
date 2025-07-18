#include "serial_handler.hpp"
#include "motor_controller.hpp"
#include "robot_controller.hpp"

SerialHandler::SerialHandler(HardwareSerial& serial): serialPort_(serial), bufferIndex_(0), parserState_(IDLE){
  memset(messageBuffer_, 0, sizeof(messageBuffer_));
}

/**
 * @brief Initializes the serial port with the specified baud rate.
 * 
 * @param baud The baud rate for serial communication.
 */
void SerialHandler::begin(unsigned long baud) {
  serialPort_.begin(baud);
}

/**
 * @brief Handles incoming serial data.
 * 
 * This function reads incoming serial data and stores it in a buffer until the end symbol is detected, then returns the processed message.
 */
std::optional<SerialMessage> SerialHandler::handleSerialInput() {
  std::optional<SerialMessage> msg = std::nullopt;
  
  while (serialPort_.available()) {
    char byte = serialPort_.read();

    switch (parserState_) {
      case IDLE:
        if (byte == START_SYMBOL) {
          resetParserState();
          parserState_ = RECEIVING;
        }
        break;

      case RECEIVING:
        if (byte == END_SYMBOL) {
          parserState_ = COMPLETE;
        } else if (byte == START_SYMBOL) { // Error handling for unexpected start symbol
          resetParserState();
          parserState_ = RECEIVING;
        } else if (bufferIndex_ < MAX_MESSAGE_LENGTH - 1) {
          messageBuffer_[bufferIndex_++] = byte;
        } else {
          // Overflow: discard message
          resetParserState();
        }
        break;

      case COMPLETE:
        // This is only called when more than one message is in the input buffer (only happens when code to slow)
        char msgType = messageBuffer_[0];

        if (msgType == CMD_SUPER) {
          msg = processMessage();  // process super command immediately
          if (byte == START_SYMBOL) {
            parserState_ = RECEIVING;
          }
        } else {
          if (byte == START_SYMBOL) {
            resetParserState();
            parserState_ = RECEIVING;
          }
        }
        break;
    }
  }
  if (parserState_ == COMPLETE) {
    msg = processMessage();
  }
  return msg;
}

/**
 * @brief Resets the parser state.
 * 
 * This function resets the buffer index and parser state to prepare for the next message.
 */
void SerialHandler::resetParserState() {
  memset(messageBuffer_, 0, MAX_MESSAGE_LENGTH);
  bufferIndex_ = 0;
  parserState_ = IDLE;
}

/**
 * @brief Processes the received message.
 * 
 * This function processes the received message into a message class and returns it if its a valid message.
 */
std::optional<SerialMessage> SerialHandler::processMessage() {
  if (bufferIndex_ == 0) {
    resetParserState();
    return std::nullopt;
  }

  char msgType = messageBuffer_[0];
  std::string content(messageBuffer_ + 1, bufferIndex_ - 1); // excludes type

  resetParserState();

  return SerialMessage{msgType, content};
}

/**
 * @brief Sends a message.
 * 
 * This function sends a message marked with the message type indicator.
 * 
 * @param message The message to send.
 */
void SerialHandler::sendMessage(const char* message) {
  serialPort_.print(START_SYMBOL);
  serialPort_.print(MSG); // 'M' for message
  serialPort_.print(message);
  serialPort_.print(END_SYMBOL);
}

/**
 * @brief Sends a command.
 * 
 * This function sends a command marked with the command type indicator.
 * 
 * @param commandType The type of command to send.
 * @param command The command to send.
 */
void SerialHandler::sendCommand(const char commandType, const char* command) {
  serialPort_.print(START_SYMBOL);
  serialPort_.print(commandType); // Command type
  serialPort_.print(command);
  serialPort_.print(END_SYMBOL);
}

/**
 * @brief Sends an error.
 * 
 * This function sends an error message marked with the error type indicator.
 * 
 * @param error The error message to send.
 */
void SerialHandler::sendError(const char* error) {
  serialPort_.print(START_SYMBOL);
  serialPort_.print(ERR); // 'E' for error
  serialPort_.print(error);
  serialPort_.print(END_SYMBOL);
}
