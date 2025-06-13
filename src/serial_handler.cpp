#include "serial_handler.hpp"
#include "motor_controller.hpp"

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
 * This function reads incoming serial data and stores it in a buffer until the end symbol is detected.
 */
void SerialHandler::handleSerialInput() {
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
          processMessage();  // process super command immediately
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
    processMessage();
  }
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
 * This function processes the received message based on the message type indicator.
 */
void SerialHandler::processMessage() {
  if (bufferIndex_ == 0) {
    // No message to process
    resetParserState();
    return;
  }

  char msgType = messageBuffer_[0];
  const char* content = &messageBuffer_[1];  // The rest after the type char

  switch (msgType) {
    case MSG:
      // Handle a generic message
      break;

    case CMD_SUPER:
      // Handle super command
      break;

    case CMD_VELOCITY:
      onVelocityCommand(content, bufferIndex_ - 1);
      break;

    case ERR:
      // Handle error message
      break;

    default:
      // Unknown message type
      sendError("Unknown message type");
      break;
  }
  resetParserState();
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

/**
 * @brief Handles the speed command (binary format).
 * 
 * Expects 6 bytes: v_x (2 bytes), v_y (2 bytes), rot_z (2 bytes)
 * All values are uint16_t (unsigned), little-endian, and map [0..65535] to [-1.0 .. 1.0].
 * 
 * @param content Pointer to binary data after the 'V' command byte.
 * @param length Length of the binary content (should be 6).
 */
void SerialHandler::onVelocityCommand(const char* content, size_t length) {
  if (length != 6) {
    char errorMsg[50];
    snprintf(errorMsg, sizeof(errorMsg), "Invalid speed command length: %u", (unsigned)length);
    sendError(errorMsg);
    return;
  }

  // Read as uint16_t (little-endian)
  uint16_t raw_vx   = (uint8_t)content[0] | ((uint8_t)content[1] << 8);
  uint16_t raw_vy   = (uint8_t)content[2] | ((uint8_t)content[3] << 8);
  uint16_t raw_rotz = (uint8_t)content[4] | ((uint8_t)content[5] << 8);

  drawJoystickRegion(raw_vx, raw_vy);

  float v_x = static_cast<float>(raw_vx) / 32768.0; // Map to [-1.0, 1.0]
  float v_y = static_cast<float>(raw_vy) / 32768.0; // Map to [-1.0, 1.0]
  float rot_z = static_cast<float>(raw_rotz) / 32768.0; // Map to [-1.0, 1.0]

  // applyWheelVelocities(wheelVelocitiesFromCartesian(v_x, v_y, rot_z));

  // Optional debug print
  // char msg[64];
  // snprintf(msg, sizeof(msg), "v_x=%f v_y=%f rot=%f", vx, vy, rotz);
  // sendMessage(msg);
}

void SerialHandler::drawJoystickRegion(uint16_t v_x, uint16_t v_y) {
  memset(frame, 0, sizeof(frame));  // Clear the global frame array

  // Define midpoint and tolerance threshold
  const uint16_t midpoint = 32768;
  const uint16_t threshold = 3500;  // threshold for "close to center"

  // Check if both coordinates are near the midpoint ± threshold
  bool near_center_x = (v_x >= (midpoint - threshold)) && (v_x <= (midpoint + threshold));
  bool near_center_y = (v_y >= (midpoint - threshold)) && (v_y <= (midpoint + threshold));

  if (near_center_x && near_center_y) {
    // Light the 4 center LEDs
    frame[3][5] = 1;
    frame[3][6] = 1;
    frame[4][5] = 1;
    frame[4][6] = 1;
  } else {
    // Map the x,y coordinates normally to matrix range
    int x = map(v_x, 0, 65535, 0, 11);
    int y = map(v_y, 0, 65535, 0, 7);

    frame[y][x] = 1;
  }
}