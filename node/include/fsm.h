/**  @file fsm.h
 *   @brief The Finite State Machine that ensures safe operation of the vehicle.
 *          
 *   fsm_run_state_machine(int id) processes a CAN id, such that the computer
 *   can command the bot to enter into autonomous/manual control mode, as well
 *   as interrupt and power down the bot via remote/hardware estops.
*/

#ifndef FSM_H
#define FSM_H

#include <stdbool.h>
#include <Arduino.h>

enum fsm_state_E {
    FSM_STANDBY = 0,
    FSM_AUTONOMOUS,
    FSM_MANUAL,
    FSM_BOOT,

    FSM_STATE_COUNT
};

extern enum fsm_state_E current_state;
extern enum fsm_state_E next_state;

/*
 * @brief Runs the finite state machine, determines whether bot is in a
 *        BOOT state, or a STANDBY state waiting to receive AUTONOMOUS state or
 *        MANUAL state control messages.
 */
void fsm_get_next_state(int id, bool estop);

#endif // FSM_H
