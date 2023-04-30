#include <Arduino.h>
#include "fsm.h"

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  // digitalWrite(LED_BUILTIN, HIGH);
  // delay(5000);
  // digitalWrite(LED_BUILTIN, LOW);
  // delay(5000);

  Serial.println("Current State:");
  Serial.println(current_state);

  // Serial.println("Next State:");
  // Serial.println(next_state);
}