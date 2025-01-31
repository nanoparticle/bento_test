#include <Arduino.h>
#include "util.h"

void setup() {
  init_dfu_trigger_handling();
  SerialUSB.begin(115200);
  pinMode(PC6, OUTPUT);
}

void loop() {
  digitalWrite(PC6, HIGH);
  SerialUSB.println("On");
  delay(500);
  digitalWrite(PC6, LOW);
  SerialUSB.println("Off");
  delay(500);
}