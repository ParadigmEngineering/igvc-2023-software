/**  @file twai.h
 *   @brief The TWAI (Two-Wire Automotive Interface) driver for Paradigm's 2023 IGVC bot.
 * 
 *   Reference Documentation: https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-reference/peripherals/twai.html#examples
 *          
 *   Based on control messages received with this protocol, the bot will establish a heartbeat with the
 *   control desktop, the motors will actuate or the bot will change its control state.
*/

#ifndef TWAI_H
#define TWAI_H

#include <Arduino.h>
#include <driver/gpio.h>
#include <driver/twai.h>

/*
 * @brief Configures and initializes TWAI drivers.
 */
void setup_twai(void);

/*
 * @brief Send TWAI messages over configured pins.
 */
void tx_twai(void);

/*
 * @brief Receive TWAI messages over configured pins.
 */
void rx_twai(void);

/*
 * @brief Stops and uninstalls the TWAI drivers. 
 */
void stop_twai(void);

#endif // TWAI_H
