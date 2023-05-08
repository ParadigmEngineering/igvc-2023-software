/**  @file pwm.h
 *   @brief The PWM (Pulse Width Modulation) driver for Paradigm's 2023 IGVC bot.
 * 
 *   Reference Documentation: https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-reference/peripherals/ledc.html#ledc-api-change-pwm-frequency
 *          
 *   Interface for actuating motor controls, i.e. PWM Duty Cycle 50% means the motors will spin at half of max speed.
 *   Makes use of the ledc library, actuating PWM through a led control intensity peripheral.
*/

#ifndef PWM_H
#define PWM_H

#include <Arduino.h>
#include <driver/ledc.h>
#include <stdio.h>
#include <esp_err.h>

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          GPIO_NUM_14 // Define the output GPIO
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY               (4095) // Set duty to 50%. ((2 ** 13) - 1) * 50% = 4095
#define LEDC_FREQUENCY          (4000) // Frequency in Hertz. Set frequency at 4 kHz

/*
 * @brief Configures and initializes PWM/LEDC drivers.
 */
void pwm_init(void);

/*
 * @brief Sets the target duty cycle percent output.
 */
void pwm_set_duty_pct(void);

#endif // PWM_H

