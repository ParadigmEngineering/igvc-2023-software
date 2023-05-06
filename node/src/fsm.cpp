#include "fsm.h"

enum fsm_state_E current_state = FSM_BOOT;
enum fsm_state_E next_state = FSM_BOOT;

/*  ID MAP:
 *    0 == AUTONOMOUS_REQUEST_CAN_ID
 *    1 == MANUAL_REQUEST_CAN_ID
 *    2 == STANDBY_REQUEST_CAN_ID
 */
void fsm_get_next_state(int id, bool estop)
{
    switch(current_state)
    {
        case FSM_STANDBY:
        {
            if (id == 0)
            {
                next_state = FSM_AUTONOMOUS;
                break;
            }
            if (id == 1)
            {
                next_state = FSM_MANUAL;
                break;
            }
            break;
        }
        case FSM_AUTONOMOUS: // Fallthrough
        case FSM_MANUAL:
        {
            if (id == 2)
            {
                next_state = FSM_STANDBY;
            }
            break;
        }
        case FSM_BOOT:
        {
            next_state = FSM_STANDBY;
            break;
        }
        case FSM_STATE_COUNT: // Fallthrough
        default:
        {
            next_state = FSM_BOOT;
            break;
        }
    }

    if (estop)
    {
        next_state = FSM_BOOT;
    }

    current_state = next_state;
}
