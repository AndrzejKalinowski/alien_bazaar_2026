/*
  Minimalistic example to to get the wheels turning.
  Look at HoverboardAPI.c to learn which other functions are available.

  Further information on https://github.com/bipropellant

  Control over USB serial: send a line like "a100 b20" and press enter.
    a<value>  -> speed/pwm  (e.g. a100, a-100, a0)
    b<value>  -> steer/turn (e.g. b20, b-20, b0)
  Either token can be omitted; omitted values keep their last setting.
  If no command is received for CMD_TIMEOUT_MS, the robot stops (a=0, b=0).
*/


#include <HoverboardAPI.h>


int serialWrapper(unsigned char *data, int len) {
 return (int) Serial1.write(data,len);
}
HoverboardAPI hoverboard = HoverboardAPI(serialWrapper);

const unsigned long CMD_TIMEOUT_MS = 500;

int pwmSpeed = 0;
int pwmSteer = 0;
unsigned long lastCmdMillis = 0;

String inputBuffer;

void parseCommand(const String &line) {
  int i = 0;
  int len = line.length();
  while (i < len) {
    char c = line[i];
    if (c == 'a' || c == 'b') {
      int start = i + 1;
      int j = start;
      if (j < len && (line[j] == '-' || line[j] == '+')) j++;
      while (j < len && isDigit(line[j])) j++;
      if (j > start) {
        int value = line.substring(start, j).toInt();
        if (c == 'a') {
          pwmSpeed = value;
        } else {
          pwmSteer = value;
        }
        lastCmdMillis = millis();
      }
      i = j;
    } else {
      i++;
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  lastCmdMillis = millis();
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (inputBuffer.length() > 0) {
        parseCommand(inputBuffer);
        inputBuffer = "";
      }
    } else {
      inputBuffer += c;
    }
  }

  if (millis() - lastCmdMillis > CMD_TIMEOUT_MS) {
    pwmSpeed = 0;
    pwmSteer = 0;
  }

  hoverboard.sendPWM(pwmSpeed, pwmSteer, PROTOCOL_SOM_NOACK);
  delay(30);
}