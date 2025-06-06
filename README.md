# Omni Rover Hardware Driver

## Purpose

The **Omni Rover Hardware Driver** is a software system designed to control an omni-directional robot with mecanum wheels. The robot can move in all directions (X, Y, and rotation) by controlling four stepper motors. The system uses an Arduino and communicates with external devices via serial commands, making it easy to control the robot's movement remotely.

## Communication Format

The robot receives movement commands via **serial communication**. Commands must follow a specific format, consisting of a start symbol, message type, content, and an end symbol:

```
<START_SYMBOL><Message_Type><Content><END_SYMBOL>
```

Where:
- **START_SYMBOL**: The symbol marking the start of the message (`$`).
- **END_SYMBOL**: The symbol marking the end of the message (`\n`).
- **Message_Type**: A single character indicating the type of command (e.g., `S` for speed control).
- **Content**: The data for the command (e.g., velocities for movement).

### Speed Command (`S`)

- **Message_Type**: `S`
- **Content**: A comma-separated list of three integers representing the speeds for the X, Y, and Z (rotation) axes. Each speed value is scaled by a factor of 10.

Example format:

```
S<int1>,<int2>,<int3>
```

Where:
- `<int1>`: Speed for the X-axis (linear movement).
- `<int2>`: Speed for the Y-axis (linear movement).
- `<int3>`: Speed for rotation (angular velocity).

For example, to send a command to move with speeds 2 m/s in the X direction, 1 m/s in the Y direction, and 0.5 rad/s for rotation, send the following command:

```
$S20,10,5\n
```

### Error Handling

If the message format is invalid or the command is unrecognized, the system will send an error message back in the following format:

```
$E<Error_Message>\n
```

Where:
- **Error_Message**: A brief description of the error.

For example, if the speed command is not formatted correctly, the system might respond with:

```
$EInvalid speed command format\n
```

## Setup and Usage

1. Connect the robot to the Arduino board, ensuring that the stepper motors and motor drivers are correctly wired.
2. Upload the provided code to your Arduino board.
3. Use a serial terminal (e.g., Arduino Serial Monitor or any other serial communication tool) to send commands to the robot.
