#include <Adafruit_BMP085.h>
#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <string.h>

namespace {

constexpr uint8_t GRIP_RELAY_PIN = D1;
constexpr uint8_t RELEASE_RELAY_PIN = D3;
constexpr uint8_t RELAY_ON = LOW;
constexpr uint8_t RELAY_OFF = HIGH;
constexpr unsigned long RELEASE_PULSE_MS = 1500;
constexpr size_t COMMAND_BUFFER_SIZE = 32;

// BMP180 (I2C 0x77) on the Waveshare 10 DOF IMU Sensor, I2C on D4 (SDA) /
// D5 (SCL). Each reading blocks for about 13 ms (temperature + pressure).
constexpr unsigned long PRESSURE_SAMPLE_MS = 50;
// readPressure() cannot report I2C errors (the library ignores the bus
// results), so a loose wire gives garbage. A reading only counts if the chip
// still answers with its ID afterwards and the value is inside the BMP180's
// 300-1100 hPa range; otherwise the pressure is NAN.
constexpr uint8_t BMP180_ADDRESS = 0x77;
constexpr uint8_t BMP180_CHIP_ID_REGISTER = 0xD0;
constexpr uint8_t BMP180_CHIP_ID = 0x55;
constexpr float MIN_VALID_HPA = 300.0f;
constexpr float MAX_VALID_HPA = 1100.0f;
// Invalid readings for this long while gripping -> GRIP UNKNOWN. Shorter
// glitches (1-9 samples) are ignored: the held state is kept meanwhile.
constexpr unsigned long SENSOR_LOST_MS = 500;
// A sensor missing at startup is probed again this often, so fixing the
// wire brings it back without a reset.
constexpr unsigned long SENSOR_RETRY_MS = 2000;
// Pressure drop below the pre-grip baseline that marks a sealed (held)
// object, with hysteresis. Measured over 10 grip cycles: open cup settles at
// 71-99 hPa, a held object at 213-395 hPa. HOLD turns on at the ON threshold
// and off again only below the OFF threshold.
constexpr float HOLD_ON_THRESHOLD_HPA = 180.0f;
constexpr float HOLD_OFF_THRESHOLD_HPA = 120.0f;
// After GRIP, report GRIP FAIL if no object is held within this time. In the
// 10-cycle test an object sealed 0.9-5.4 s after GRIP.
constexpr unsigned long GRIP_CONFIRM_TIMEOUT_MS = 8000;

enum class State { IDLE, GRIPPING, RELEASING };

State state = State::IDLE;
unsigned long releaseStartedAt = 0;
char commandBuffer[COMMAND_BUFFER_SIZE] = {};
size_t commandLength = 0;
bool commandOverflow = false;

Adafruit_BMP085 bmp;
bool sensorReady = false;
float pressureHpa = NAN;
float baselineHpa = NAN;
bool holding = false;
unsigned long lastPressureSampleAt = 0;
unsigned long lastValidSampleAt = 0;
unsigned long lastSensorRetryAt = 0;
bool sensorLostReported = false;
unsigned long gripStartedAt = 0;
bool gripResultSent = false;

bool chipAnswers() {
  Wire.beginTransmission(BMP180_ADDRESS);
  Wire.write(BMP180_CHIP_ID_REGISTER);
  if (Wire.endTransmission() != 0) {
    return false;
  }
  if (Wire.requestFrom(BMP180_ADDRESS, static_cast<uint8_t>(1)) != 1) {
    return false;
  }
  return Wire.read() == BMP180_CHIP_ID;
}

void samplePressure() {
  lastPressureSampleAt = millis();
  if (!sensorReady) {
    pressureHpa = NAN;
    if (lastPressureSampleAt - lastSensorRetryAt >= SENSOR_RETRY_MS) {
      lastSensorRetryAt = lastPressureSampleAt;
      sensorReady = bmp.begin(BMP085_STANDARD);
    }
    if (!sensorReady) {
      return;
    }
  }
  float hpa = bmp.readPressure() / 100.0f;
  bool valid = chipAnswers() && hpa >= MIN_VALID_HPA && hpa <= MAX_VALID_HPA;
  pressureHpa = valid ? hpa : NAN;
  if (valid) {
    lastValidSampleAt = lastPressureSampleAt;
  }
}

float vacuumHpa() { return baselineHpa - pressureHpa; }

// A failed reading while gripping keeps the held state (no false GRIP LOST);
// if the sensor stays silent for SENSOR_LOST_MS, report GRIP UNKNOWN once.
void updateSensorLost() {
  if (sensorLostReported || millis() - lastValidSampleAt < SENSOR_LOST_MS) {
    return;
  }
  sensorLostReported = true;
  gripResultSent = true;  // no GRIP FAIL either, the result is unknown now
  Serial.println("GRIP UNKNOWN");
}

// Reports GRIP OK when an object becomes held and GRIP LOST when it is lost
// while the grip relay is still on.
void updateHolding() {
  bool wasHolding = holding;
  if (state == State::GRIPPING && !isnan(baselineHpa)) {
    if (isnan(pressureHpa)) {
      updateSensorLost();
      return;
    }
    sensorLostReported = false;  // report the next outage again
  }

  if (state != State::GRIPPING || isnan(baselineHpa)) {
    holding = false;
  } else if (vacuumHpa() >= HOLD_ON_THRESHOLD_HPA) {
    holding = true;
  } else if (vacuumHpa() < HOLD_OFF_THRESHOLD_HPA) {
    holding = false;
  }

  if (state != State::GRIPPING || holding == wasHolding) {
    return;
  }
  if (holding) {
    gripResultSent = true;
    Serial.println("GRIP OK");
  } else {
    Serial.println("GRIP LOST");
  }
}

void updateGripTimeout() {
  if (state != State::GRIPPING || gripResultSent ||
      millis() - gripStartedAt < GRIP_CONFIRM_TIMEOUT_MS) {
    return;
  }

  gripResultSent = true;
  Serial.println("GRIP FAIL");
}

void updatePressure() {
  if (millis() - lastPressureSampleAt >= PRESSURE_SAMPLE_MS) {
    samplePressure();
    updateHolding();
  }
}

void setIdle() {
  digitalWrite(GRIP_RELAY_PIN, RELAY_OFF);
  digitalWrite(RELEASE_RELAY_PIN, RELAY_OFF);
  state = State::IDLE;
  holding = false;
}

void startGrip() {
  setIdle();
  samplePressure();
  baselineHpa = pressureHpa;
  digitalWrite(GRIP_RELAY_PIN, RELAY_ON);
  state = State::GRIPPING;
  gripStartedAt = millis();
  gripResultSent = false;
  sensorLostReported = false;
  Serial.println("DONE GRIP");
  if (!sensorReady || isnan(baselineHpa)) {
    gripResultSent = true;
    Serial.println("GRIP UNKNOWN");
  }
}

void startRelease() {
  setIdle();
  releaseStartedAt = millis();
  digitalWrite(RELEASE_RELAY_PIN, RELAY_ON);
  state = State::RELEASING;
}

void updateRelease() {
  if (state != State::RELEASING ||
      millis() - releaseStartedAt < RELEASE_PULSE_MS) {
    return;
  }

  setIdle();
  Serial.println("DONE RELEASE");
}

void handleCommand(const char *command) {
  if (strcmp(command, "STATUS") == 0) {
    switch (state) {
      case State::IDLE:
        Serial.println("STATE IDLE");
        break;
      case State::GRIPPING:
        Serial.println("STATE GRIPPING");
        break;
      case State::RELEASING:
        Serial.println("STATE RELEASING");
        break;
    }
  } else if (strcmp(command, "HOLD") == 0) {
    if (!sensorReady || isnan(pressureHpa)) {
      Serial.println("ERR NO_SENSOR");
    } else if (holding) {
      Serial.println("HOLD YES");
    } else {
      Serial.println("HOLD NO");
    }
  } else if (strcmp(command, "PRESSURE") == 0) {
    if (!sensorReady || isnan(pressureHpa)) {
      Serial.println("ERR NO_SENSOR");
    } else if (state == State::GRIPPING && !isnan(baselineHpa)) {
      Serial.printf("PRESSURE %.2f BASE %.2f VACUUM %.2f\n", pressureHpa,
                    baselineHpa, vacuumHpa());
    } else {
      Serial.printf("PRESSURE %.2f\n", pressureHpa);
    }
  } else if (strcmp(command, "GRIP") == 0) {
    if (state != State::IDLE) {
      Serial.println("ERR BUSY");
      return;
    }
    startGrip();
  } else if (strcmp(command, "RELEASE") == 0) {
    if (state == State::RELEASING) {
      Serial.println("ERR BUSY");
      return;
    }
    startRelease();
  } else {
    Serial.println("ERR UNKNOWN_COMMAND");
  }
}

void readCommands() {
  while (Serial.available() > 0) {
    char ch = static_cast<char>(Serial.read());
    if (ch == '\n') {
      if (commandOverflow) {
        Serial.println("ERR LINE_TOO_LONG");
      } else if (commandLength > 0) {
        commandBuffer[commandLength] = '\0';
        handleCommand(commandBuffer);
      }
      commandLength = 0;
      commandOverflow = false;
    } else if (ch == '\r') {
      continue;
    } else if (!commandOverflow) {
      if (commandLength < COMMAND_BUFFER_SIZE - 1) {
        commandBuffer[commandLength++] = ch;
      } else {
        commandOverflow = true;
      }
    }
  }
}

}  // namespace

void setup() {
  digitalWrite(GRIP_RELAY_PIN, RELAY_OFF);
  digitalWrite(RELEASE_RELAY_PIN, RELAY_OFF);
  pinMode(GRIP_RELAY_PIN, OUTPUT);
  pinMode(RELEASE_RELAY_PIN, OUTPUT);
  setIdle();
  Serial.begin(115200);
  Wire.begin();
  sensorReady = bmp.begin(BMP085_STANDARD);
  lastSensorRetryAt = millis();
  samplePressure();
}

void loop() {
  updatePressure();
  updateGripTimeout();
  updateRelease();
  readCommands();
  updateRelease();
}
