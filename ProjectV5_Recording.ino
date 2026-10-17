#include <Bluepad32.h>
#include <Preferences.h>
#include <SPI.h>
#include <Wire.h>
#include <mcp2515.h>
#include <SmartDriver.h>

constexpr uint8_t CAN_CS_PIN = 5;
constexpr uint8_t MPU_SDA_PIN = 21;
constexpr uint8_t MPU_SCL_PIN = 22;

constexpr uint16_t DRIVE_MOTOR_ID = 0x04;
constexpr uint16_t STEERING_MOTOR_ID = 0x03;
constexpr uint16_t DRIVE_FEEDBACK_ID = 104;
constexpr uint16_t STEERING_FEEDBACK_ID = 103;

constexpr float MAX_DRIVE_SPEED = 10.0f;
// Start with a safe steering travel. Increase only after confirming the
// mechanism can move through the requested angle without hitting a hard stop.
constexpr float MAX_STEERING_TRAVEL_RAD = 2.0f;
constexpr float STEERING_DIRECTION = -1.0f;  // Change to -1.0f to reverse.
// Positive means a small physical-right trim. STEERING_DIRECTION converts it
// into the correct motor-position sign.
constexpr float STEERING_CENTER_TRIM_RAD = -0.08f;
constexpr int JOYSTICK_DEADZONE = 50;
constexpr uint32_t COMMAND_PERIOD_MS = 20;
constexpr uint32_t CENTER_FEEDBACK_TIMEOUT_MS = 1500;

// Record / playback controls and storage settings.
constexpr uint16_t RECORD_BUTTON_MASK = 0x0001;    // Controller button A.
constexpr uint16_t PLAYBACK_BUTTON_MASK = 0x0002;  // Controller button B.
constexpr uint16_t RECORDING_VERSION = 1;
constexpr uint32_t RECORDING_MAGIC = 0x52424F54;  // "RBOT".
constexpr uint32_t MAX_RECORDING_SECONDS = 120;
constexpr uint32_t MAX_RECORDING_FRAMES =
    (MAX_RECORDING_SECONDS * 1000UL) / COMMAND_PERIOD_MS;
constexpr float RECORDING_INPUT_SCALE = 10000.0f;
constexpr char RECORDING_NAMESPACE[] = "robotrec";

// MPU-6050 / straight-line heading-hold settings.
constexpr uint8_t MPU_ADDRESS_LOW = 0x68;
constexpr uint8_t MPU_ADDRESS_HIGH = 0x69;
constexpr float MPU_GYRO_SCALE = 131.0f;  // LSB per degree/s at +/-250 dps.
constexpr float GYRO_Z_DIRECTION = 1.0f;  // Use -1.0f if MPU is upside down.
// Keep this below the slowest turn that must be detected. One raw gyro count
// is about 0.0076 degree/s at the selected +/-250 degree/s range.
constexpr float GYRO_Z_NOISE_FLOOR_DPS = 0.008f;
constexpr float GYRO_Z_FILTER_ALPHA = 0.18f;
// Slowly follow temperature-related gyro offset only while drive is stopped.
constexpr float STATIONARY_BIAS_ALPHA = 0.001f;
constexpr float MIN_HEADING_HOLD_SPEED = 1.0f;
constexpr float HEADING_KP = 0.3f;       // rad of steering per degree error.
constexpr float HEADING_KI = 0.005f;       // rad per degree-second.
constexpr float HEADING_KD = 0.02f;       // rad per degree/second yaw rate.
constexpr float HEADING_ERROR_DEADBAND_DEG = 0.05f;
constexpr float MAX_HEADING_INTEGRAL = 25.0f;
constexpr float MAX_AUTO_STEERING_RAD = 0.50f;
// A correction smaller than this may not overcome gearbox/linkage stiction.
constexpr float MIN_AUTO_STEERING_RAD = 0.04f;
constexpr float HEADING_CORRECTION_DIRECTION = 1.0f; // change to -1.0f if it correcting in the wrong direction 

// Stop energizing the steering motor once the robot is straight and the
// steering mechanism has reached its target. MPU yaw wakes it when needed.
constexpr float STEERING_SETTLED_POSITION_RAD = 0.06f;
constexpr float YAW_RELAX_ERROR_DEG = 0.06f;
constexpr float YAW_WAKE_ERROR_DEG = 0.15f;
constexpr float YAW_RELAX_RATE_DPS = 0.12f;
constexpr float YAW_WAKE_RATE_DPS = 0.18f;
constexpr uint32_t STEERING_RELAX_DELAY_MS = 350;
// Automatic correction is pulsed so a jammed or unreachable position cannot
// leave the steering motor energized continuously.
constexpr uint32_t MAX_AUTO_STEERING_ACTIVE_MS = 900;
constexpr uint32_t STEERING_COOLDOWN_MS = 300;

MCP2515 mcp2515(CAN_CS_PIN);
// Keep independent command state for the two different control modes.
SmartDriver driveMotor;
SmartDriver steeringMotor;
ControllerPtr myControllers[BP32_MAX_GAMEPADS];
// Competition safety lock: the first controller connected after boot owns the
// robot until the ESP32 is restarted. A disconnect does not release the lock.
ControllerPtr authorizedController = nullptr;
bool controllerSlotLocked = false;

// Each 20 ms frame stores the two movement inputs. At four bytes per frame,
// the default 60-second recording uses 12 KB of RAM and flash.
struct MovementFrame {
    int16_t throttle;
    int16_t steering;
};
static_assert(sizeof(MovementFrame) == 4,
              "MovementFrame must remain four bytes");

MovementFrame recordedFrames[MAX_RECORDING_FRAMES];
uint32_t recordedFrameCount = 0;
uint32_t playbackFrameIndex = 0;
bool recordingActive = false;
bool playbackActive = false;
bool previousRecordButtonPressed = false;
bool previousPlaybackButtonPressed = false;

struct can_frame txFrame;
struct can_frame rxFrame;
uint8_t driveCommandData[8];
uint8_t steeringCommandData[8];

float driveSpeedGoal = 0.0f;
float steeringPositionGoal = 0.0f;
float steeringCenterPosition = 0.0f;
float manualSteeringInput = 0.0f;
float automaticSteeringCorrection = 0.0f;
bool steeringCenterCaptured = false;
float steeringFeedbackPosition = 0.0f;
bool steeringFeedbackValid = false;
bool steeringRelaxed = false;
uint32_t steeringStableStartTime = 0;
uint32_t steeringAutoActiveStartTime = 0;
uint32_t steeringCooldownUntil = 0;
uint32_t lastCommandTime = 0;
uint32_t centerFeedbackStartTime = 0;
uint32_t lastCanFeedbackLogTime = 0;

uint8_t mpuAddress = MPU_ADDRESS_LOW;
bool mpuAvailable = false;
float gyroZBiasRaw = 0.0f;
float yawDegrees = 0.0f;
float yawRateDegreesPerSecond = 0.0f;
float filteredYawRateDegreesPerSecond = 0.0f;
uint32_t lastImuSampleMicros = 0;

bool headingHoldActive = false;
float headingTargetDegrees = 0.0f;
float headingIntegral = 0.0f;
float headingPTerm = 0.0f;
float headingITerm = 0.0f;
float headingDTerm = 0.0f;
uint32_t lastHeadingPidMicros = 0;
uint32_t lastHeadingTelemetryTime = 0;

float normalizeJoystickAxis(int axisValue) {
    int magnitude = abs(axisValue);
    if (magnitude <= JOYSTICK_DEADZONE) {
        return 0.0f;
    }

    float normalized =
        static_cast<float>(magnitude - JOYSTICK_DEADZONE) /
        static_cast<float>(512 - JOYSTICK_DEADZONE);

    if (normalized > 1.0f) {
        normalized = 1.0f;
    }

    return (axisValue < 0) ? -normalized : normalized;
}

float clampFloat(float value, float minimum, float maximum) {
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

float wrapAngleDegrees(float angle) {
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

uint32_t calculateRecordingCrc32(const uint8_t* data, size_t length) {
    uint32_t crc = 0xFFFFFFFFUL;
    for (size_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            crc = (crc >> 1) ^
                  ((crc & 1U) ? 0xEDB88320UL : 0UL);
        }
    }
    return ~crc;
}

bool saveRecordingToFlash() {
    if (recordedFrameCount == 0 ||
        recordedFrameCount > MAX_RECORDING_FRAMES) {
        Serial.println("Recording not saved: no valid movement frames");
        return false;
    }

    const size_t dataSize =
        recordedFrameCount * sizeof(MovementFrame);
    const uint32_t crc = calculateRecordingCrc32(
        reinterpret_cast<const uint8_t*>(recordedFrames), dataSize);

    Preferences preferences;
    if (!preferences.begin(RECORDING_NAMESPACE, false)) {
        Serial.println("Recording save failed: cannot open ESP32 storage");
        return false;
    }

    // The data is written first and the identifying metadata last. Loading
    // checks the size and CRC, so an interrupted write is never played.
    const bool dataSaved =
        preferences.putBytes("frames", recordedFrames, dataSize) == dataSize;
    const bool countSaved =
        preferences.putUInt("count", recordedFrameCount) == sizeof(uint32_t);
    const bool intervalSaved =
        preferences.putUShort("period", COMMAND_PERIOD_MS) == sizeof(uint16_t);
    const bool versionSaved =
        preferences.putUShort("version", RECORDING_VERSION) ==
            sizeof(uint16_t);
    const bool crcSaved =
        preferences.putUInt("crc", crc) == sizeof(uint32_t);
    const bool magicSaved =
        preferences.putUInt("magic", RECORDING_MAGIC) == sizeof(uint32_t);
    preferences.end();

    const bool saveSucceeded =
        dataSaved && countSaved && intervalSaved && versionSaved &&
        crcSaved && magicSaved;
    if (saveSucceeded) {
        Serial.printf(
            "Recording saved permanently: %lu frames, %.2f seconds\n",
            static_cast<unsigned long>(recordedFrameCount),
            recordedFrameCount * COMMAND_PERIOD_MS / 1000.0f);
    } else {
        Serial.println("Recording save failed: ESP32 storage write error");
    }
    return saveSucceeded;
}

bool loadRecordingFromFlash() {
    Preferences preferences;
    if (!preferences.begin(RECORDING_NAMESPACE, true)) {
        Serial.println("No saved recording found");
        return false;
    }

    const uint32_t magic = preferences.getUInt("magic", 0);
    const uint16_t version = preferences.getUShort("version", 0);
    const uint16_t period = preferences.getUShort("period", 0);
    const uint32_t frameCount = preferences.getUInt("count", 0);
    const uint32_t savedCrc = preferences.getUInt("crc", 0);
    const size_t storedSize = preferences.getBytesLength("frames");
    const size_t expectedSize = frameCount * sizeof(MovementFrame);

    const bool metadataValid =
        magic == RECORDING_MAGIC && version == RECORDING_VERSION &&
        period == COMMAND_PERIOD_MS && frameCount > 0 &&
        frameCount <= MAX_RECORDING_FRAMES && storedSize == expectedSize;
    if (!metadataValid) {
        preferences.end();
        Serial.println("No valid saved recording found");
        return false;
    }

    const size_t bytesRead =
        preferences.getBytes("frames", recordedFrames, expectedSize);
    preferences.end();
    if (bytesRead != expectedSize) {
        recordedFrameCount = 0;
        Serial.println("Saved recording rejected: incomplete flash read");
        return false;
    }

    const uint32_t calculatedCrc = calculateRecordingCrc32(
        reinterpret_cast<const uint8_t*>(recordedFrames), expectedSize);
    if (calculatedCrc != savedCrc) {
        recordedFrameCount = 0;
        Serial.println("Saved recording rejected: checksum mismatch");
        return false;
    }

    recordedFrameCount = frameCount;
    Serial.printf("Saved recording loaded: %lu frames, %.2f seconds\n",
                  static_cast<unsigned long>(recordedFrameCount),
                  recordedFrameCount * COMMAND_PERIOD_MS / 1000.0f);
    return true;
}

bool mpuWriteRegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(mpuAddress);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
}

bool mpuReadRegisters(uint8_t reg, uint8_t* data, size_t length) {
    Wire.beginTransmission(mpuAddress);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;

    const size_t received = Wire.requestFrom(mpuAddress, length, true);
    if (received != length) {
        while (Wire.available()) Wire.read();
        return false;
    }

    for (size_t i = 0; i < length; i++) {
        data[i] = Wire.read();
    }
    return true;
}

bool detectMpu6050() {
    const uint8_t addresses[] = {MPU_ADDRESS_LOW, MPU_ADDRESS_HIGH};

    for (uint8_t address : addresses) {
        mpuAddress = address;
        Wire.beginTransmission(mpuAddress);
        if (Wire.endTransmission() != 0) continue;

        uint8_t whoAmI = 0xFF;
        if (mpuReadRegisters(0x75, &whoAmI, 1)) {
            Serial.printf("I2C device at 0x%02X, WHO_AM_I=0x%02X\n",
                          mpuAddress, whoAmI);
        } else {
            Serial.printf(
                "I2C device at 0x%02X, but WHO_AM_I read failed\n",
                mpuAddress);
        }
        return true;
    }

    Serial.println("MPU initialization failed: no ACK at 0x68 or 0x69");
    return false;
}

bool readGyroZRaw(int16_t& rawGyroZ) {
    uint8_t data[2];
    if (!mpuReadRegisters(0x47, data, sizeof(data))) return false;

    rawGyroZ = static_cast<int16_t>(
        (static_cast<uint16_t>(data[0]) << 8) | data[1]);
    return true;
}

bool calibrateGyroZ() {
    constexpr int CALIBRATION_SAMPLES = 1000;
    int validSamples = 0;
    int64_t sum = 0;

    Serial.println("Keep robot still: calibrating MPU-6050 gyro...");
    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        int16_t rawGyroZ = 0;
        if (readGyroZRaw(rawGyroZ)) {
            sum += rawGyroZ;
            validSamples++;
        }
        delay(2);
    }

    if (validSamples < 900) {
        Serial.printf(
            "MPU calibration failed: only %d/1000 gyro reads succeeded\n",
            validSamples);
        return false;
    }

    gyroZBiasRaw = static_cast<float>(sum) /
                   static_cast<float>(validSamples);
    Serial.printf("MPU-6050 ready at 0x%02X, gyro Z bias=%.2f\n",
                  mpuAddress, gyroZBiasRaw);
    return true;
}

bool initializeMpu6050() {
    Wire.begin(MPU_SDA_PIN, MPU_SCL_PIN);
    Wire.setClock(100000);
    delay(100);

    if (!detectMpu6050()) return false;

    // Wake up using the internal clock, apply a low-pass filter,
    // sample at 200 Hz, and select the +/-250 degree/s gyro range.
    if (!mpuWriteRegister(0x6B, 0x00)) {
        Serial.println("MPU initialization failed: PWR_MGMT_1 write");
        return false;
    }
    if (!mpuWriteRegister(0x1A, 0x03)) {
        Serial.println("MPU initialization failed: CONFIG write");
        return false;
    }
    if (!mpuWriteRegister(0x19, 0x04)) {
        Serial.println("MPU initialization failed: SMPLRT_DIV write");
        return false;
    }
    if (!mpuWriteRegister(0x1B, 0x00)) {
        Serial.println("MPU initialization failed: GYRO_CONFIG write");
        return false;
    }
    delay(100);

    if (!calibrateGyroZ()) return false;

    yawDegrees = 0.0f;
    yawRateDegreesPerSecond = 0.0f;
    filteredYawRateDegreesPerSecond = 0.0f;
    lastImuSampleMicros = micros();
    return true;
}

void updateImuYaw() {
    if (!mpuAvailable) return;

    const uint32_t nowMicros = micros();
    const uint32_t elapsedMicros = nowMicros - lastImuSampleMicros;
    if (elapsedMicros < 4000) return;  // Approximately 250 Hz maximum.

    int16_t rawGyroZ = 0;
    if (!readGyroZRaw(rawGyroZ)) return;

    lastImuSampleMicros = nowMicros;
    const float dt = elapsedMicros * 0.000001f;
    if (dt <= 0.0f || dt > 0.1f) return;

    // Track slow temperature-related bias changes only when the drive motor is
    // stopped. Never adapt the bias while driving because that would teach the
    // filter that a real slow turn is zero.
    if (fabsf(driveSpeedGoal) < MIN_HEADING_HOLD_SPEED) {
        gyroZBiasRaw += STATIONARY_BIAS_ALPHA *
                        (static_cast<float>(rawGyroZ) - gyroZBiasRaw);
    }

    const float measuredYawRate =
        GYRO_Z_DIRECTION *
        ((static_cast<float>(rawGyroZ) - gyroZBiasRaw) / MPU_GYRO_SCALE);

    // A low-pass filter removes vibration without discarding genuine slow
    // rotation. The old 0.05 degree/s cutoff could hide a gradual drift.
    filteredYawRateDegreesPerSecond += GYRO_Z_FILTER_ALPHA *
        (measuredYawRate - filteredYawRateDegreesPerSecond);
    yawRateDegreesPerSecond = filteredYawRateDegreesPerSecond;
    if (fabsf(yawRateDegreesPerSecond) < GYRO_Z_NOISE_FLOOR_DPS) {
        yawRateDegreesPerSecond = 0.0f;
    }

    yawDegrees = wrapAngleDegrees(
        yawDegrees + yawRateDegreesPerSecond * dt);
}

void sendMotorCommand(SmartDriver& command,
                      uint8_t* commandBytes,
                      uint16_t canId,
                      float goal,
                      bool useSpeedMode,
                      bool stopMotor) {
    command.speedMode = useSpeedMode;
    command.positionMode = !useSpeedMode;
    command.voltageMode = false;
    command.reset = false;
    command.stop = stopMotor;
    command.goal = goal;
    command.getCommand(commandBytes);

    txFrame.can_id = canId;
    txFrame.can_dlc = 8;
    memcpy(txFrame.data, commandBytes, 8);

    MCP2515::ERROR result = mcp2515.sendMessage(&txFrame);
    if (result != MCP2515::ERROR_OK) {
        Serial.printf("CAN transmit failed: ID=%u error=%d\n",
                      canId, static_cast<int>(result));
    }
}

void sendDriveCommand() {
    const bool stopDrive = (driveSpeedGoal == 0.0f);
    sendMotorCommand(driveMotor, driveCommandData, DRIVE_MOTOR_ID,
                     driveSpeedGoal, true, stopDrive);
}

void requestSteeringFeedback() {
    // Before position mode starts, send a zero-speed command to ID 3. This
    // safely requests its feedback so the current encoder position can be
    // captured as the physical steering center.
    sendMotorCommand(steeringMotor, steeringCommandData, STEERING_MOTOR_ID,
                     0.0f, true, true);
}

void sendSteeringStopCommand() {
    // Leave position mode and remove active holding torque. A geared steering
    // mechanism normally keeps its last position mechanically.
    sendMotorCommand(steeringMotor, steeringCommandData, STEERING_MOTOR_ID,
                     0.0f, true, true);
}

void updateSteeringGoal() {
    steeringPositionGoal =
        steeringCenterPosition +
        STEERING_DIRECTION *
        ((manualSteeringInput * MAX_STEERING_TRAVEL_RAD) +
         STEERING_CENTER_TRIM_RAD) +
        automaticSteeringCorrection;
}

void resetHeadingHold() {
    headingHoldActive = false;
    headingIntegral = 0.0f;
    headingPTerm = 0.0f;
    headingITerm = 0.0f;
    headingDTerm = 0.0f;
    lastHeadingPidMicros = 0;
    automaticSteeringCorrection = 0.0f;
}

void updateHeadingHold() {
    const bool steeringIsNeutral = (manualSteeringInput == 0.0f);
    const bool robotIsMoving =
        fabsf(driveSpeedGoal) >= MIN_HEADING_HOLD_SPEED;

    if (!mpuAvailable || !steeringIsNeutral || !robotIsMoving) {
        resetHeadingHold();
        updateSteeringGoal();
        return;
    }

    if (!headingHoldActive) {
        headingHoldActive = true;
        headingTargetDegrees = yawDegrees;
        headingIntegral = 0.0f;
        headingPTerm = 0.0f;
        headingITerm = 0.0f;
        headingDTerm = 0.0f;
        lastHeadingPidMicros = micros();
        automaticSteeringCorrection = 0.0f;
        Serial.printf("Heading hold locked at %.2f degrees\n",
                      headingTargetDegrees);
    }

    // Positive error means the robot has yawed left from the locked heading.
    float headingError =
        wrapAngleDegrees(yawDegrees - headingTargetDegrees);

    if (fabsf(headingError) < HEADING_ERROR_DEADBAND_DEG) {
        headingError = 0.0f;
    }

    const uint32_t nowMicros = micros();
    float pidDt = (nowMicros - lastHeadingPidMicros) * 0.000001f;
    lastHeadingPidMicros = nowMicros;
    if (pidDt <= 0.0f || pidDt > 0.1f) {
        pidDt = COMMAND_PERIOD_MS * 0.001f;
    }

    headingPTerm = HEADING_KP * headingError;

    // The heading target is fixed during heading hold, so measured yaw rate
    // is the derivative of heading error and is less noisy than subtracting
    // two integrated yaw values.
    headingDTerm = HEADING_KD * yawRateDegreesPerSecond;

    // Test a new integral value before accepting it. If the output is already
    // saturated, only integrate when the error would move it away from the
    // limit. This prevents integral windup.
    const float candidateIntegral = clampFloat(
        headingIntegral + headingError * pidDt,
        -MAX_HEADING_INTEGRAL, MAX_HEADING_INTEGRAL);
    const float candidateITerm = HEADING_KI * candidateIntegral;
    const float candidateOutput =
        headingPTerm + candidateITerm + headingDTerm;

    const bool outputInsideLimit =
        fabsf(candidateOutput) <= MAX_AUTO_STEERING_RAD;
    const bool errorReducesHighSaturation =
        candidateOutput > MAX_AUTO_STEERING_RAD && headingError < 0.0f;
    const bool errorReducesLowSaturation =
        candidateOutput < -MAX_AUTO_STEERING_RAD && headingError > 0.0f;

    if (outputInsideLimit || errorReducesHighSaturation ||
        errorReducesLowSaturation) {
        headingIntegral = candidateIntegral;
    }

    headingITerm = HEADING_KI * headingIntegral;
    float pidOutput = clampFloat(
        headingPTerm + headingITerm + headingDTerm,
        -MAX_AUTO_STEERING_RAD, MAX_AUTO_STEERING_RAD);

    // The JGB37 gearbox and steering linkage can ignore a mathematically valid
    // but tiny target change. Give non-zero heading errors a minimum nudge.
    if (headingError != 0.0f &&
        fabsf(pidOutput) < MIN_AUTO_STEERING_RAD) {
        const float correctionSign =
            (pidOutput != 0.0f) ? pidOutput : headingError;
        pidOutput = copysignf(MIN_AUTO_STEERING_RAD, correctionSign);
    }

    automaticSteeringCorrection =
        HEADING_CORRECTION_DIRECTION * STEERING_DIRECTION *
        ((driveSpeedGoal >= 0.0f) ? 1.0f : -1.0f) * pidOutput;
    updateSteeringGoal();
}

void updateSteeringRelaxation(uint32_t now) {
    const bool manualControl = (manualSteeringInput != 0.0f);
    const bool robotIsMoving =
        fabsf(driveSpeedGoal) >= MIN_HEADING_HOLD_SPEED;

    if (manualControl || !steeringCenterCaptured) {
        steeringRelaxed = false;
        steeringStableStartTime = 0;
        steeringAutoActiveStartTime = 0;
        steeringCooldownUntil = 0;
        return;
    }

    const float yawError = headingHoldActive
        ? fabsf(wrapAngleDegrees(yawDegrees - headingTargetDegrees))
        : 0.0f;
    const float yawRate = fabsf(yawRateDegreesPerSecond);
    const bool steeringAtTarget =
        !steeringFeedbackValid ||
        fabsf(steeringFeedbackPosition - steeringPositionGoal) <=
            STEERING_SETTLED_POSITION_RAD;

    if (steeringRelaxed) {
        // Heading error is integrated continuously, so even a yaw rate below
        // YAW_WAKE_RATE_DPS eventually crosses YAW_WAKE_ERROR_DEG. The rate
        // threshold just reacts sooner to a faster turn.
        const bool cooldownFinished =
            static_cast<int32_t>(now - steeringCooldownUntil) >= 0;
        if (robotIsMoving && cooldownFinished &&
            (yawError >= YAW_WAKE_ERROR_DEG ||
             yawRate >= YAW_WAKE_RATE_DPS)) {
            steeringRelaxed = false;
            steeringStableStartTime = 0;
            steeringAutoActiveStartTime = now;
            Serial.println("Steering PID reactivated by MPU yaw");
        }
        return;
    }

    if (robotIsMoving && steeringAutoActiveStartTime == 0) {
        steeringAutoActiveStartTime = now;
    }

    // Thermal protection: if feedback never reaches the requested position,
    // stop driving the steering motor briefly instead of holding stall current.
    if (robotIsMoving && steeringAutoActiveStartTime != 0 &&
        now - steeringAutoActiveStartTime >= MAX_AUTO_STEERING_ACTIVE_MS) {
        steeringRelaxed = true;
        steeringStableStartTime = 0;
        steeringAutoActiveStartTime = 0;
        steeringCooldownUntil = now + STEERING_COOLDOWN_MS;
        Serial.println("Steering correction paused for motor cooldown");
        return;
    }

    const bool yawIsStable =
        !robotIsMoving ||
        (headingHoldActive &&
         yawError <= YAW_RELAX_ERROR_DEG &&
         yawRate <= YAW_RELAX_RATE_DPS);

    if (yawIsStable && steeringAtTarget) {
        if (steeringStableStartTime == 0) {
            steeringStableStartTime = now;
        } else if (now - steeringStableStartTime >=
                   STEERING_RELAX_DELAY_MS) {
            steeringRelaxed = true;
            steeringAutoActiveStartTime = 0;
            Serial.println("Steering motor relaxed; MPU yaw monitoring active");
        }
    } else {
        steeringStableStartTime = 0;
    }
}

void sendSteeringPositionCommand() {
    updateSteeringGoal();

    // SmartDriver requires its dedicated positionMode flag. Merely setting
    // speedMode=false does not enable position control.
    sendMotorCommand(steeringMotor, steeringCommandData, STEERING_MOTOR_ID,
                     steeringPositionGoal, false, false);
}

void sendCurrentCommands() {
    sendDriveCommand();

    if (steeringCenterCaptured) {
        if (steeringRelaxed && manualSteeringInput == 0.0f) {
            sendSteeringStopCommand();
        } else {
            sendSteeringPositionCommand();
        }
    } else {
        requestSteeringFeedback();
    }
}

void stopAndCenter() {
    driveSpeedGoal = 0.0f;
    manualSteeringInput = 0.0f;
    resetHeadingHold();
    updateSteeringGoal();
    sendCurrentCommands();
}

int16_t encodeRecordedInput(float input) {
    const float limited = clampFloat(input, -1.0f, 1.0f);
    return static_cast<int16_t>(limited * RECORDING_INPUT_SCALE);
}

float decodeRecordedInput(int16_t input) {
    return clampFloat(
        static_cast<float>(input) / RECORDING_INPUT_SCALE,
        -1.0f, 1.0f);
}

void applyMovementInputs(float throttle, float steering) {
    driveSpeedGoal = clampFloat(throttle, -1.0f, 1.0f) * MAX_DRIVE_SPEED;
    manualSteeringInput = clampFloat(steering, -1.0f, 1.0f);

    if (manualSteeringInput != 0.0f) {
        resetHeadingHold();
        steeringRelaxed = false;
        steeringStableStartTime = 0;
    }
    updateSteeringGoal();
}

void startMovementRecording() {
    if (playbackActive) {
        Serial.println("Cannot record while playback is active");
        return;
    }

    recordedFrameCount = 0;
    recordingActive = true;
    Serial.printf(
        "RECORDING STARTED: press A again to stop (maximum %lu seconds)\n",
        static_cast<unsigned long>(MAX_RECORDING_SECONDS));
}

void stopMovementRecordingAndSave() {
    if (!recordingActive) return;

    recordingActive = false;
    if (recordedFrameCount == 0) {
        Serial.println("Recording empty; previous saved recording retained");
        loadRecordingFromFlash();
        return;
    }

    // Flash writes can briefly block the loop, so stop the motors before
    // committing the recording instead of leaving the last command active.
    stopAndCenter();
    Serial.println("RECORDING STOPPED: saving to ESP32 flash...");
    saveRecordingToFlash();
}

void captureMovementFrame() {
    if (!recordingActive) return;

    if (recordedFrameCount >= MAX_RECORDING_FRAMES) {
        Serial.println("Recording reached maximum length; stopping");
        stopMovementRecordingAndSave();
        return;
    }

    MovementFrame& frame = recordedFrames[recordedFrameCount++];
    frame.throttle = encodeRecordedInput(
        (MAX_DRIVE_SPEED != 0.0f) ? driveSpeedGoal / MAX_DRIVE_SPEED : 0.0f);
    frame.steering = encodeRecordedInput(manualSteeringInput);
}

void stopMovementPlayback(const char* reason) {
    if (!playbackActive) return;

    playbackActive = false;
    playbackFrameIndex = 0;
    stopAndCenter();
    Serial.printf("PLAYBACK STOPPED: %s\n", reason);
}

void startMovementPlayback() {
    if (recordingActive) {
        Serial.println("Cannot play while recording is active");
        return;
    }
    if (recordedFrameCount == 0) {
        Serial.println("No saved movement recording to play");
        return;
    }

    playbackFrameIndex = 0;
    playbackActive = true;
    stopAndCenter();
    Serial.printf(
        "PLAYBACK STARTED: %lu frames, %.2f seconds; press B to cancel\n",
        static_cast<unsigned long>(recordedFrameCount),
        recordedFrameCount * COMMAND_PERIOD_MS / 1000.0f);
}

void applyNextPlaybackFrame() {
    if (!playbackActive) return;

    if (playbackFrameIndex >= recordedFrameCount) {
        stopMovementPlayback("recording complete");
        return;
    }

    const MovementFrame& frame = recordedFrames[playbackFrameIndex++];
    applyMovementInputs(decodeRecordedInput(frame.throttle),
                        decodeRecordedInput(frame.steering));
}

void readCanFeedback() {
    while (mcp2515.readMessage(&rxFrame) == MCP2515::ERROR_OK) {
        const uint16_t receiveId = rxFrame.can_id & 0x07FF;

        if ((receiveId == DRIVE_FEEDBACK_ID ||
             receiveId == STEERING_FEEDBACK_ID) &&
            rxFrame.can_dlc >= 8) {
            float position = 0.0f;
            float speed = 0.0f;
            memcpy(&position, rxFrame.data, sizeof(float));
            memcpy(&speed, rxFrame.data + 4, sizeof(float));

            if (receiveId == STEERING_FEEDBACK_ID) {
                steeringFeedbackPosition = position;
                steeringFeedbackValid = true;
            }

            if (receiveId == STEERING_FEEDBACK_ID &&
                !steeringCenterCaptured) {
                steeringCenterPosition = position;
                steeringCenterCaptured = true;
                updateSteeringGoal();

                Serial.printf(
                    "Steering center captured from ID 103: %.3f rad\n",
                    steeringCenterPosition);
            }

            const uint32_t now = millis();
            if (now - lastCanFeedbackLogTime >= 250) {
                lastCanFeedbackLogTime = now;
                Serial.printf(
                    "Feedback ID=%u position=%.3f rad speed=%.3f\n",
                    receiveId, position, speed);
            }
        }
    }
}

void onConnectedController(ControllerPtr ctl) {
    if (controllerSlotLocked) {
        Serial.printf(
            "SECURITY: rejected extra controller at index %d: %s\n",
            ctl->index(), ctl->getModelName().c_str());
        ctl->disconnect();
        return;
    }

    // Whichever controller completes its connection first after startup is
    // accepted. From this point onward, only an ESP32 restart clears the lock.
    controllerSlotLocked = true;
    authorizedController = ctl;
    myControllers[0] = ctl;

    // Stop discovery/pairing. Previously bonded controllers may still attempt
    // to reconnect, so the rejection path above remains necessary.
    BP32.enableNewBluetoothConnections(false);

    ControllerProperties properties = ctl->getProperties();
    Serial.printf(
        "AUTHORIZED controller locked: index=%d model=%s "
        "BT=%02X:%02X:%02X:%02X:%02X:%02X\n",
        ctl->index(), ctl->getModelName().c_str(),
        properties.btaddr[0], properties.btaddr[1], properties.btaddr[2],
        properties.btaddr[3], properties.btaddr[4], properties.btaddr[5]);
    Serial.println("Controller lock remains active until ESP32 restart");

    // Player LED 1 identifies the controller that owns the robot.
    ctl->setPlayerLEDs(0x01);
}

void onDisconnectedController(ControllerPtr ctl) {
    if (ctl == authorizedController) {
        Serial.println(
            "AUTHORIZED controller disconnected; robot stopped and "
            "locked until ESP32 restart");
        if (recordingActive) {
            stopMovementRecordingAndSave();
        }
        if (playbackActive) {
            stopMovementPlayback("controller disconnected");
        }
        authorizedController = nullptr;
        myControllers[0] = nullptr;
        stopAndCenter();
        return;
    }

    // Do not change controllerSlotLocked for rejected controllers.
    Serial.printf("Rejected controller disconnected from index %d\n",
                  ctl->index());
}

void processGamepad(ControllerPtr ctl) {
    const uint16_t buttons = ctl->buttons();
    const bool recordButtonPressed =
        (buttons & RECORD_BUTTON_MASK) != 0;
    const bool playbackButtonPressed =
        (buttons & PLAYBACK_BUTTON_MASK) != 0;
    const bool recordButtonRising =
        recordButtonPressed && !previousRecordButtonPressed;
    const bool playbackButtonRising =
        playbackButtonPressed && !previousPlaybackButtonPressed;

    previousRecordButtonPressed = recordButtonPressed;
    previousPlaybackButtonPressed = playbackButtonPressed;

    // B starts playback and pressing B again is the emergency cancel.
    if (playbackButtonRising) {
        if (playbackActive) {
            stopMovementPlayback("cancelled with button B");
            return;
        }
        if (recordingActive) {
            Serial.println("Button B ignored while recording; press A to save");
        } else {
            startMovementPlayback();
            return;
        }
    }

    // Live joystick input cannot interfere with an active playback.
    if (playbackActive) {
        if (recordButtonRising) {
            Serial.println("Button A ignored during playback; press B to cancel");
        }
        return;
    }

    // A toggles recording. Button edges are used so holding A does not
    // immediately start and stop the recording.
    if (recordButtonRising) {
        if (recordingActive) {
            stopMovementRecordingAndSave();
        } else {
            startMovementRecording();
        }
    }

    // LEFT joystick Y keeps controlling drive speed. Pushing it upward
    // normally produces a negative value, so invert Y.
    const float throttle = -normalizeJoystickAxis(ctl->axisY());

    // RIGHT joystick X controls steering position.
    const float steering = normalizeJoystickAxis(ctl->axisRX());
    applyMovementInputs(throttle, steering);

    sendCurrentCommands();

    Serial.printf(
        "leftY=%d rightX=%d driveSpeed=%.2f steeringTarget=%.3f rad "
        "centerReady=%s\n",
        ctl->axisY(), ctl->axisRX(), driveSpeedGoal, steeringPositionGoal,
        steeringCenterCaptured ? "yes" : "no");
}

void processControllers() {
    // Never process input from a controller that did not win the startup lock.
    ControllerPtr controller = authorizedController;
    if (controller && controller->isConnected() && controller->hasData() &&
        controller->isGamepad()) {
        processGamepad(controller);
    }
}

void setup() {
    Serial.begin(115200);
    delay(300);

    loadRecordingFromFlash();

    mpuAvailable = initializeMpu6050();
    if (!mpuAvailable) {
        Serial.println(
            "WARNING: MPU initialization failed; manual steering only");
    }

    SPI.begin();
    mcp2515.reset();

    // Change MCP_8MHZ to MCP_16MHZ if the MCP2515 crystal is 16 MHz.
    MCP2515::ERROR bitrateResult =
        mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);
    if (bitrateResult != MCP2515::ERROR_OK) {
        Serial.printf("MCP2515 bitrate setup failed: error=%d\n",
                      static_cast<int>(bitrateResult));
    }

    mcp2515.setNormalMode();
    centerFeedbackStartTime = millis();

    BP32.setup(&onConnectedController, &onDisconnectedController);
    BP32.enableVirtualDevice(false);

    // Do not call BP32.forgetBluetoothKeys() here; retain controller pairing.
    stopAndCenter();

    Serial.println("Joystick model ready");
    Serial.println("Left Y: ID 4 speed, Right X: ID 3 position");
    Serial.println("Right stick centered while moving: heading hold enabled");
    Serial.println("Keep steering physically centered during startup");
    Serial.println(
        "SECURITY: first controller connected owns robot until restart");
    Serial.println("Button A (0x0001): start/stop and save recording");
    Serial.println("Button B (0x0002): start/cancel saved playback");
}

void loop() {
    updateImuYaw();

    if (BP32.update()) {
        processControllers();
    }

    const uint32_t now = millis();

    // If ID 103 does not return feedback, fall back to 0 rad as center so the
    // steering still receives position commands. The Serial Monitor reports
    // this condition so CAN feedback configuration can be checked.
    if (!steeringCenterCaptured &&
        now - centerFeedbackStartTime >= CENTER_FEEDBACK_TIMEOUT_MS) {
        steeringCenterPosition = 0.0f;
        steeringCenterCaptured = true;
        updateSteeringGoal();
        Serial.println(
            "WARNING: no ID 103 feedback; using 0 rad as steering center");
    }

    if (now - lastCommandTime >= COMMAND_PERIOD_MS) {
        lastCommandTime = now;

        if (playbackActive) {
            applyNextPlaybackFrame();
        } else if (recordingActive) {
            captureMovementFrame();
        }

        updateHeadingHold();
        updateSteeringRelaxation(now);
        sendCurrentCommands();

        if (headingHoldActive &&
            now - lastHeadingTelemetryTime >= 250) {
            lastHeadingTelemetryTime = now;
            const float error =
                wrapAngleDegrees(yawDegrees - headingTargetDegrees);
            Serial.printf(
                "PID yaw=%.2f target=%.2f error=%.2f rate=%.3f dps "
                "P=%.3f I=%.3f D=%.3f output=%.3f rad relaxed=%s\n",
                yawDegrees, headingTargetDegrees, error,
                yawRateDegreesPerSecond,
                headingPTerm, headingITerm, headingDTerm,
                automaticSteeringCorrection,
                steeringRelaxed ? "yes" : "no");
        }
    }

    readCanFeedback();
    delay(1);
}
