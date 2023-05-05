#include <Arduino.h>
#include <unity.h>
#include "fsm.h"

void setUp(void) {
    // set global vars up here
}

void tearDown(void) {
    // clean up global vars here
}

void test_initial_fsm(void)
{
    TEST_ASSERT_TRUE((current_state == FSM_BOOT));
    TEST_ASSERT_TRUE((next_state == FSM_BOOT));
}

void test_boot_transition_fsm(void)
{
    TEST_ASSERT_TRUE((current_state == FSM_BOOT));

    // BOOT -> STANDBY
    fsm_get_next_state(0, false);

    TEST_ASSERT_TRUE((current_state == FSM_STANDBY));
}

// Test STANDBY <-> AUTONOMOUS and STANDBY <-> MANUAL control transitions
void test_standby_transitions_fsm(void)
{
    TEST_ASSERT_TRUE((current_state == FSM_STANDBY));

    // STANDBY -> MANUAL
    fsm_get_next_state(1, false);

    TEST_ASSERT_TRUE((current_state == FSM_MANUAL));

    // MANUAL -> STANDBY
    fsm_get_next_state(2, false);

    TEST_ASSERT_TRUE((current_state == FSM_STANDBY));

    // STANDBY -> AUTONOMOUS
    fsm_get_next_state(0, false);

    TEST_ASSERT_TRUE((current_state == FSM_AUTONOMOUS));

    // AUTONOMOUS -> STANDBY
    fsm_get_next_state(2, false);

    TEST_ASSERT_TRUE((current_state == FSM_STANDBY));
}

void test_estop_fsm(void)
{
    TEST_ASSERT_TRUE((current_state == FSM_STANDBY));

    // STANDBY -> BOOT
    fsm_get_next_state(0, true);

    TEST_ASSERT_TRUE((current_state == FSM_BOOT));

    // BOOT -> BOOT
    fsm_get_next_state(0, true);

    TEST_ASSERT_TRUE((current_state == FSM_BOOT));

    // Get bot to AUTONOMOUS state
    fsm_get_next_state(0, false);
    fsm_get_next_state(0, false);

    TEST_ASSERT_TRUE((current_state == FSM_AUTONOMOUS));

    // AUTONOMOUS -> BOOT
    fsm_get_next_state(0, true);

    TEST_ASSERT_TRUE((current_state == FSM_BOOT));

    // Get bot to MANUAL state
    fsm_get_next_state(1, false);
    fsm_get_next_state(1, false);

    TEST_ASSERT_TRUE((current_state == FSM_MANUAL));

    // MANUAL -> BOOT
    fsm_get_next_state(1, true);

    TEST_ASSERT_TRUE((current_state == FSM_BOOT));
}

void setup()
{
    delay(2000); // service delay
    UNITY_BEGIN();

    RUN_TEST(test_initial_fsm);
    RUN_TEST(test_boot_transition_fsm);
    RUN_TEST(test_standby_transitions_fsm);
    RUN_TEST(test_estop_fsm);

    UNITY_END(); // stop unit testing
}

void loop()
{
}