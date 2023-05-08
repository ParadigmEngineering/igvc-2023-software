#include <Arduino.h>
#include "twai.h"
#include "pwm.h"
#include "fsm.h"
#include "led.h"

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);

  setup_twai();
  stop_twai();

  pwm_init();
}

void loop() {
  // AUTONOMOUS
  // fsm_get_next_state(0, false);

  // MANUAL
  // fsm_get_next_state(1, false);

  // STANDBY
  fsm_get_next_state(2, false);

  flash_leds();
}
