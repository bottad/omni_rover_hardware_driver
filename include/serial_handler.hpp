#ifndef SERIAL_HANDLER_HPP
#define SERIAL_HANDLER_HPP

#include <Arduino.h>
#include "messages.hpp"

#include <optional>

/**
 * @brief States of the serial message parser.
 * 
 * Used internally to track the progress of receiving and parsing an incoming serial message.
 */
enum ParserState {
    IDLE,       ///< Waiting for a start symbol to begin receiving a message.
    RECEIVING,  ///< Currently receiving bytes of the message.
    COMPLETE    ///< A complete message has been received and is ready for processing.
};

/**
 * @brief Handles serial communication including message parsing and sending.
 * 
 * This class manages receiving and sending messages over a HardwareSerial interface.
 * It buffers incoming data, parses messages framed by start/end symbols,
 * and provides methods to send various types of messages including commands and errors.
 */
class SerialHandler {
public:
    /**
     * @brief Constructs a SerialHandler with the specified HardwareSerial port.
     * 
     * @param serial Reference to the HardwareSerial instance used for communication.
     */
    SerialHandler(HardwareSerial& serial);
    
    /**
     * @brief Initializes the serial port with the specified baud rate.
     * 
     * Sets up the hardware serial interface with the given baud rate for communication.
     * 
     * @param baud The baud rate for serial communication.
     */
    void begin(unsigned long baud);
    
    /**
     * @brief Handles incoming serial data.
     * 
     * Reads bytes from the serial port and accumulates them in a buffer until a complete message
     * is detected (based on start/end symbols). Parses and returns the message if available.
     * 
     * @return std::optional<SerialMessage> The parsed message if one is complete; otherwise, std::nullopt.
     */
    std::optional<SerialMessage> handleSerialInput();

    /**
     * @brief Sends a generic message.
     * 
     * Sends a message wrapped with the standard start and end symbols, marked as a generic message.
     * 
     * @param message Null-terminated ASCII string message to send.
     */
    void sendMessage(const char* message);
    
    /**
     * @brief Sends a command as an ASCII string.
     * 
     * Sends a command identified by a single character command type followed by a null-terminated ASCII string.
     * The message is framed by start and end symbols.
     * 
     * @param commandType The character indicating the command type.
     * @param command Null-terminated ASCII string containing the command data.
     */
    void sendCommand(const char commandType, const char* command);
    
    /**
     * @brief Sends a command with binary data over the serial port.
     * 
     * Sends a command with a raw binary payload of specified length, framed by start and end symbols.
     * This method supports sending non-ASCII data such as sensor readings or packed values.
     * 
     * @param commandType The character indicating the command type.
     * @param data Pointer to the binary data buffer.
     * @param length Number of bytes to send from the data buffer.
     */
    void sendCommand(const char commandType, const uint8_t* data, size_t length);

    /**
     * @brief Sends an error message.
     * 
     * Sends an error message wrapped with start and end symbols, marked with the error type indicator.
     * 
     * @param error Null-terminated ASCII string describing the error.
     */
    void sendError(const char* error);

private:
    HardwareSerial& serialPort_;                ///< Reference to the serial port interface.

    char messageBuffer_[MAX_MESSAGE_LENGTH];    ///< Buffer to accumulate incoming message bytes.
    size_t bufferIndex_;                        ///< Current index in the message buffer.
    ParserState parserState_;                   ///< Current state of the message parser.

    /**
     * @brief Processes the received message buffer into a SerialMessage.
     * 
     * Validates and converts the accumulated message buffer into a SerialMessage structure.
     * Returns the message if valid; otherwise returns nullopt.
     * 
     * @return std::optional<SerialMessage> The processed message or nullopt if invalid.
     */
    std::optional<SerialMessage> processMessage();

    /**
     * @brief Resets the parser state and buffer for the next incoming message.
     * 
     * Clears the message buffer and resets the parser state to IDLE.
     */
    void resetParserState();
};

#endif // SERIAL_HANDLER_HPP