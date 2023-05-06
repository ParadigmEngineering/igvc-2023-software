#include "led.h"
#include "fsm.h"

long int previous_time = millis();
bool previous_toggle = false;

void flash_leds(void)
{
    switch (current_state)
    {
    case FSM_STANDBY:
        digitalWrite(LED_BUILTIN, HIGH);
        break;
    case FSM_AUTONOMOUS:
        // Toggle LEDs once every 250ms
        if ((millis() - previous_time) > 250)
        {
            digitalWrite(LED_BUILTIN, previous_toggle);
            previous_toggle = !previous_toggle;
            previous_time = millis();
        }
        break;
    case FSM_MANUAL:
        // Toggle LEDs once every 2s
        if ((millis() - previous_time) > 2000)
        {
            digitalWrite(LED_BUILTIN, previous_toggle);
            previous_toggle = !previous_toggle;
            previous_time = millis(); 
        }
        break;
    default:
        digitalWrite(LED_BUILTIN, LOW);
        break;
    }
}
