# ESP32 CAN Robot Controller

ESP32 robot-control firmware using a Bluetooth gamepad, two CAN-controlled
motors, an MCP2515 CAN interface, and an MPU-6050 for yaw-based heading
correction.

> [!IMPORTANT]
> `SmartDriver.h` and `SmartDriver.cpp` are proprietary competition files and
> are intentionally **not included** in this public repository. The sketch is
> published as reference source code and will not compile or control the motors
> without the team's private SmartDriver library.

## Main Features

- First Bluetooth controller connected after startup becomes the only
  authorized controller until the ESP32 restarts.
- Left joystick Y controls drive-motor speed.
- Right joystick X controls steering-motor position.
- MPU-6050 yaw PID attempts to maintain a straight heading while the right
  joystick is centered.
- Steering motor uses encoder position feedback and thermal-protection pulses.
- Button A records movement inputs; Button B replays the saved recording.
- The most recent recording is stored in ESP32 NVS and survives a restart.
- Re-recording replaces the previous saved movement.

## Control Mapping

| Control | Function |
| --- | --- |
| Left joystick Y | Forward/reverse drive speed |
| Right joystick X | Left/right steering position |
| Button A (`0x0001`) | Start recording; press again to stop and save |
| Button B (`0x0002`) | Start playback; press again to cancel |

Movement is sampled every 20 ms. The default maximum recording length is 60
seconds.

## CAN Mapping

| Device | Command ID | Feedback ID | Mode |
| --- | ---: | ---: | --- |
| Drive motor | `0x04` | `104` | Speed |
| Steering motor | `0x03` | `103` | Position |

The sketch is configured for:

- CAN bitrate: `1 Mbps`
- MCP2515 oscillator: `8 MHz`
- MCP2515 chip-select pin: `GPIO 5`

If the MCP2515 module has a 16 MHz crystal, change `MCP_8MHZ` to `MCP_16MHZ`
in the sketch.

## MPU-6050 Wiring

| MPU-6050 | ESP32 |
| --- | --- |
| VCC | 3.3 V |
| GND | GND |
| SDA | GPIO 21 |
| SCL | GPIO 22 |

The firmware checks I2C addresses `0x68` and `0x69`, calibrates the Z-axis
gyro during startup, and integrates gyro rate into relative yaw. Keep the
robot completely still while it starts.

## Required Software

The versions below are recommended pinned versions for this repository. If the
competition robot was validated with different versions, pin those exact
working versions before an event.

| Software or library | Version | Installation |
| --- | --- | --- |
| [Arduino IDE](https://www.arduino.cc/en/software) | 2.3.x | Arduino website |
| [ESP32 + Bluepad32 board package](https://bluepad32.readthedocs.io/en/latest/plat_arduino/) | 4.2.0 | Arduino Boards Manager |
| [`arduino-mcp2515` by autowp](https://github.com/autowp/arduino-mcp2515) | 1.3.1 | Arduino Library Manager or GitHub release |
| `Preferences` | Bundled | Included with the ESP32 Arduino core |
| `SPI` | Bundled | Included with the Arduino/ESP32 core |
| `Wire` | Bundled | Included with the Arduino/ESP32 core |
| SmartDriver | Private team version | Not distributed |

### Board Manager URLs

Add these URLs in Arduino IDE under **File → Preferences → Additional Boards
Manager URLs**:

```text
https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
https://raw.githubusercontent.com/ricardoquesada/esp32-arduino-lib-builder/master/bluepad32_files/package_esp32_bluepad32_index.json
```

Then install and select the matching **ESP32 + Bluepad32 Arduino** board.

## Upload Instructions

1. Install the ESP32 and ESP32 + Bluepad32 board packages.
2. Install `arduino-mcp2515` version 1.3.1.
3. Authorized team members install the private SmartDriver library locally.
4. Open `CAN_Motor_Joystick_MPU6050_PID.ino` in Arduino IDE.
5. Select the correct **ESP32 + Bluepad32 Arduino** board and COM port.
6. Keep the robot raised or disconnect motor power for the first upload test.
7. Upload the sketch and open Serial Monitor at `115200` baud.
8. Keep the steering physically centered and the robot still during startup.

## How the Robot Operates

At startup, the ESP32 initializes the MPU-6050, MCP2515, saved movement data,
and Bluepad32. The current steering encoder reading from feedback ID `103` is
captured as the steering center. The first gamepad to connect is authorized;
later controllers are rejected until the ESP32 restarts.

During manual operation, left-stick input becomes a speed command for CAN ID
`0x04`. Right-stick input becomes a steering-position offset for CAN ID `0x03`.
When the right stick is centered and the robot is moving, MPU yaw feedback is
used to calculate a limited steering correction.

Recording stores the left-stick and right-stick movement inputs at fixed 20 ms
intervals. Stopping the recording writes it to ESP32 non-volatile storage with
metadata and checksum validation. Playback applies those inputs at the same
nominal interval and stops automatically at the end.

## Playback Limitations

Playback repeats controller inputs; it does not store an exact physical path.
Results can change because of:

- Starting position or heading differences
- Tire slip and floor friction
- Battery voltage and motor temperature
- Steering backlash or mechanical alignment
- Robot load and weight distribution
- MPU gyro drift and command timing

For accurate autonomous path repetition, add closed-loop trajectory tracking
using drive-encoder distance and a recorded yaw profile. The MPU-6050 cannot
detect pure sideways displacement when the robot's heading does not change.

## Competition and Disclosure Notice

This public repository intentionally excludes the proprietary SmartDriver
implementation. Be aware that publishing the main sketch still reveals CAN
IDs, pin assignments, control mappings, PID behavior, and other robot logic.
Use a private GitHub repository or publish a sanitized version if competition
rules require those details to remain confidential.

