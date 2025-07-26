#include "serial_handler.hpp"

SerialHandler::SerialHandler(HardwareSerial& serial): serialPort_(serial), bufferIndex_(0), parserState_(IDLE){
  memset(messageBuffer_, 0, sizeof(messageBuffer_));
}

void SerialHandler::begin(unsigned long baud) {
  serialPort_.begin(baud);
}

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
          return msg; // Return immediately to process super command
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

void SerialHandler::sendMessage(const char* message) {
  serialPort_.print(START_SYMBOL);
  serialPort_.print(MSG); // 'M' for message
  serialPort_.print(message);
  serialPort_.print(END_SYMBOL);
}

void SerialHandler::sendCommand(const char commandType, const char* command) {
  serialPort_.print(START_SYMBOL);
  serialPort_.print(commandType); // Command type
  serialPort_.print(command);
  serialPort_.print(END_SYMBOL);
}

void SerialHandler::sendCommand(const char commandType, const uint8_t* data, size_t length) {
  serialPort_.write(START_SYMBOL);
  serialPort_.write(commandType);
  serialPort_.write(data, length);
  serialPort_.write(END_SYMBOL);
}

void SerialHandler::sendError(const char* error) {
  serialPort_.print(START_SYMBOL);
  serialPort_.print(ERR); // 'E' for error
  serialPort_.print(error);
  serialPort_.print(END_SYMBOL);
}

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

void SerialHandler::resetParserState() {
  memset(messageBuffer_, 0, MAX_MESSAGE_LENGTH);
  bufferIndex_ = 0;
  parserState_ = IDLE;
}
