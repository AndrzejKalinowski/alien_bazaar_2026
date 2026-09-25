#include <Arduino.h>
#include <string.h>

namespace {

constexpr uint8_t GRIP_RELAY_PIN = D1;
constexpr uint8_t RELEASE_RELAY_PIN = D3;
constexpr uint8_t RELAY_ON = LOW;
constexpr uint8_t RELAY_OFF = HIGH;
constexpr unsigned long RELEASE_PULSE_MS = 500;
constexpr size_t COMMAND_BUFFER_SIZE = 32;

enum class State { IDLE, GRIPPING, RELEASING };

State state = State::IDLE;
unsigned long releaseStartedAt = 0;
char commandBuffer[COMMAND_BUFFER_SIZE] = {};
size_t commandLength = 0;
bool commandOverflow = false;

void setIdle() {
  digitalWrite(GRIP_RELAY_PIN, RELAY_OFF);
  digitalWrite(RELEASE_RELAY_PIN, RELAY_OFF);
  state = State::IDLE;
}

void startGrip() {
  setIdle();
  digitalWrite(GRIP_RELAY_PIN, RELAY_ON);
  state = State::GRIPPING;
  Serial.println("DONE GRIP");
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
}

void loop() {
  updateRelease();
  readCommands();
  updateRelease();
}
