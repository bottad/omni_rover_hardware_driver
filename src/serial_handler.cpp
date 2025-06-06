#include "serial_handler.hpp"
#include "motor_controller.hpp"

SerialHandler::SerialHandler(HardwareSerial& serial) 
    : serialPort(serial), bufferIndex(0), messageComplete(false), messageStarted(false) {}

/**
 * @brief Initializes the serial port with the specified baud rate.
 * 
 * @param baud The baud rate for serial communication.
 */
void SerialHandler::begin(unsigned long baud) {
  serialPort.begin(baud);
}

/**
 * @brief Handles incoming serial data.
 * 
 * This function reads incoming serial data and stores it in a buffer until the end symbol is detected.
 */
void SerialHandler::handleSerialInput() {
  while (serialPort.available()) {
    char inChar = (char)serialPort.read();
    lastCharTime = millis();

    if (inChar == START_SYMBOL) {
      resetParserState();
      messageStarted = true;
    }

    if (messageStarted) {
      // Protect against buffer overflow
      if (bufferIndex < MAX_MESSAGE_LENGTH - 1) {
        messageBuffer[bufferIndex++] = inChar;

        // End of message detected
        if (inChar == END_SYMBOL) {
          messageComplete = true;
          break;
        }
      } else {
        sendError("Buffer overflow");
        resetParserState();
        break;
      }
    }
  }
  
  if (messageComplete) {
    processMessage();
    resetParserState();
  }
}

/**
 * @brief Resets the parser state.
 * 
 * This function resets the buffer index and message flags to prepare for the next message.
 */
void SerialHandler::resetParserState() {
  memset(messageBuffer, 0, sizeof(messageBuffer));

  bufferIndex = 0;
  messageStarted = false;
  messageComplete = false;
}

/**
 * @brief Processes the received message.
 * 
 * This function processes the received message based on the message type indicator.
 */
void SerialHandler::processMessage() {
  if (messageBuffer[0] == START_SYMBOL && messageBuffer[bufferIndex - 1] == END_SYMBOL) {
    char messageType = messageBuffer[1];
    const char* content = &messageBuffer[2];
    size_t contentLength = bufferIndex - 3; // Exclude start and end symbols

    switch (messageType) {
      case CMD_SPEED: // Speed command
        onSpeedCommand(content, contentLength);
        break;
      default:
        sendError("Unknown message type");
        break;
    }
  } else {
    sendError("Invalid message format");
  }
}

/**
 * @brief Sends a message.
 * 
 * This function sends a message marked with the message type indicator.
 * 
 * @param message The message to send.
 */
void SerialHandler::sendMessage(const char* message) {
  serialPort.print(START_SYMBOL);
  serialPort.print(MSG); // 'M' for message
  serialPort.print(message);
  serialPort.print(END_SYMBOL);
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
  serialPort.print(START_SYMBOL);
  serialPort.print(commandType); // Command type
  serialPort.print(command);
  serialPort.print(END_SYMBOL);
}

/**
 * @brief Sends an error.
 * 
 * This function sends an error message marked with the error type indicator.
 * 
 * @param error The error message to send.
 */
void SerialHandler::sendError(const char* error) {
  serialPort.print(START_SYMBOL);
  serialPort.print(ERR); // 'E' for error
  serialPort.print(error);
  serialPort.print(END_SYMBOL);
}

/**
 * @brief Handles the speed command.
 * 
 * This function processes the speed command and prints the received speeds.
 * 
 * @param content The content of the speed command.
 * @param length The length of the content.
 */
void SerialHandler::onSpeedCommand(const char* content, size_t length) {
  if (length >= 5) { // Minimum length for "int1,int2,int3" is 5 characters
    int int1, int2, int3;
    // Parse the integers from the content
    int parsed = sscanf(content, "%d,%d,%d", &int1, &int2, &int3);
    if (parsed == 3) {
      // Convert integers to floats and divide by 10
      float floatSpeeds[3];
      floatSpeeds[0] = static_cast<float>(int1) / 10.0;
      floatSpeeds[1] = static_cast<float>(int2) / 10.0;
      floatSpeeds[2] = static_cast<float>(int3) / 10.0;

      // Print the converted float values
      String message = "Speed command received: ";
      for (size_t i = 0; i < 3; ++i) {
        message += String(floatSpeeds[i], 2);  // Format with 2 decimal places
        if (i < 2) {
          message += ", ";
        }
      }
      sendMessage(message.c_str());

      lastCommandTime = millis(); // Reset the watchdog timer0 

      applyWheelVelocities(wheelVelocitiesFromCartesian(floatSpeeds[0], floatSpeeds[1], floatSpeeds[2]));
    } else {
      sendError("Invalid speed command format");
    }
  } else {
    sendError("Invalid speed command length");
  }
}