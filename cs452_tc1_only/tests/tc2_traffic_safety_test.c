#include <assert.h>
#include <limits.h>
#include <stdio.h>

#include "tc2_traffic_safety.h"

static tc2_traffic_safety_input base_input(void) {
        tc2_traffic_safety_input input = {
                .train_a_route_scalar_mm = 1000,
                .train_b_route_scalar_mm = 1500,
                .train_a_effective_speed = 80,
                .train_b_effective_speed = 60,
                .train_a_braking_envelope_mm = 300,
                .train_b_braking_envelope_mm = 150,
                .gap_mm = 1000,
                .relation =
                        TC2_TRAFFIC_RELATION_SAME_DIRECTION_A_REAR,
                .rear_stop_latched = 0,
                .rear_has_movement_authority = 1,
        };
        return input;
}

static void test_head_on_boundary(void) {
        tc2_traffic_safety_input input = base_input();
        input.relation = TC2_TRAFFIC_RELATION_HEAD_ON;

        /* Fixed 1200 mm boundary: equality is unsafe. */
        input.gap_mm = 1200;
        assert(Tc2TrafficSafetyEvaluate(&input) ==
               TC2_TRAFFIC_ACTION_STOP_BOTH);
        input.gap_mm = 1201;
        assert(Tc2TrafficSafetyEvaluate(&input) ==
               TC2_TRAFFIC_ACTION_NONE);

}

static void test_head_on_uses_fixed_pair_distance(void) {
        tc2_traffic_safety_input input = base_input();
        input.relation = TC2_TRAFFIC_RELATION_HEAD_ON;
        input.train_a_braking_envelope_mm = 950;
        input.train_b_braking_envelope_mm = 55;

        /* Individual commands do not alter fixed protection. */
        input.gap_mm = 1200;
        assert(Tc2TrafficSafetyEvaluate(&input) ==
               TC2_TRAFFIC_ACTION_STOP_BOTH);
        input.gap_mm = 1201;
        assert(Tc2TrafficSafetyEvaluate(&input) ==
               TC2_TRAFFIC_ACTION_NONE);

        input.train_a_braking_envelope_mm = 55;
        input.train_b_braking_envelope_mm = 950;
        input.gap_mm = 1200;
        assert(Tc2TrafficSafetyEvaluate(&input) ==
               TC2_TRAFFIC_ACTION_STOP_BOTH);
        input.gap_mm = 1201;
        assert(Tc2TrafficSafetyEvaluate(&input) ==
               TC2_TRAFFIC_ACTION_NONE);
}

static void test_following_rear_selection_and_boundary(void) {
        tc2_traffic_safety_input input = base_input();

        /* Both rear orientations use the fixed 300 mm boundary. */
        input.gap_mm = 300;
        assert(Tc2TrafficSafetyEvaluate(&input) ==
               TC2_TRAFFIC_ACTION_STOP_REAR);
        input.gap_mm = 301;
        assert(Tc2TrafficSafetyEvaluate(&input) ==
               TC2_TRAFFIC_ACTION_NONE);

        input.relation =
                TC2_TRAFFIC_RELATION_SAME_DIRECTION_B_REAR;
        input.gap_mm = 300;
        assert(Tc2TrafficSafetyEvaluate(&input) ==
               TC2_TRAFFIC_ACTION_STOP_REAR);
        input.gap_mm = 301;
        assert(Tc2TrafficSafetyEvaluate(&input) ==
               TC2_TRAFFIC_ACTION_NONE);
}

static void test_following_300_500_hysteresis_sequence(void) {
        tc2_traffic_safety_input input = base_input();
        input.train_a_effective_speed = 0;
        input.train_b_effective_speed = 0;
        input.train_a_braking_envelope_mm = 0;
        input.train_b_braking_envelope_mm = 0;

        /* Current speed does not change the fixed stop edge. */
        input.gap_mm = 300;
        assert(Tc2TrafficSafetyEvaluate(&input) ==
               TC2_TRAFFIC_ACTION_STOP_REAR);
        input.gap_mm = 301;
        assert(Tc2TrafficSafetyEvaluate(&input) ==
               TC2_TRAFFIC_ACTION_NONE);

        /*
         * Once stopped, the rear remains latched through the dead band and
         * resumes only at 500 mm.  Clearing the latch at 500 mm must not
         * immediately request another stop.
         */
        input.rear_stop_latched = 1;
        input.gap_mm = 300;
        assert(Tc2TrafficSafetyEvaluate(&input) ==
               TC2_TRAFFIC_ACTION_KEEP_HOLD);
        input.gap_mm = 499;
        assert(Tc2TrafficSafetyEvaluate(&input) ==
               TC2_TRAFFIC_ACTION_KEEP_HOLD);
        input.gap_mm = 500;
        assert(Tc2TrafficSafetyEvaluate(&input) ==
               TC2_TRAFFIC_ACTION_RESUME_REAR);
        input.rear_stop_latched = 0;
        assert(Tc2TrafficSafetyEvaluate(&input) ==
               TC2_TRAFFIC_ACTION_NONE);
        input.gap_mm = 300;
        assert(Tc2TrafficSafetyEvaluate(&input) ==
               TC2_TRAFFIC_ACTION_STOP_REAR);
}

static void test_equal_speed_following_still_protects_rear(void) {
        tc2_traffic_safety_input input = base_input();
        input.train_a_effective_speed = 60;
        input.train_b_effective_speed = 60;
        input.train_a_braking_envelope_mm = 100;
        input.train_b_braking_envelope_mm = 100;

        /*
         * Equal speed does not remove collision protection: topology chooses
         * the rear train, and the fixed 300 mm boundary applies.
         */
        input.gap_mm = 300;
        assert(Tc2TrafficSafetyEvaluate(&input) ==
               TC2_TRAFFIC_ACTION_STOP_REAR);
        input.gap_mm = 301;
        assert(Tc2TrafficSafetyEvaluate(&input) ==
               TC2_TRAFFIC_ACTION_NONE);

        input.relation =
                TC2_TRAFFIC_RELATION_SAME_DIRECTION_B_REAR;
        input.gap_mm = 300;
        assert(Tc2TrafficSafetyEvaluate(&input) ==
               TC2_TRAFFIC_ACTION_STOP_REAR);
        input.gap_mm = 301;
        assert(Tc2TrafficSafetyEvaluate(&input) ==
               TC2_TRAFFIC_ACTION_NONE);
}

static void test_following_hold_and_resume_hysteresis(void) {
        tc2_traffic_safety_input input = base_input();
        input.train_a_effective_speed = 0;
        input.train_a_braking_envelope_mm = 100;
        input.rear_stop_latched = 1;

        input.gap_mm = 499;
        assert(Tc2TrafficSafetyEvaluate(&input) ==
               TC2_TRAFFIC_ACTION_KEEP_HOLD);
        input.gap_mm = 500;
        assert(Tc2TrafficSafetyEvaluate(&input) ==
               TC2_TRAFFIC_ACTION_RESUME_REAR);

        input.rear_has_movement_authority = 0;
        assert(Tc2TrafficSafetyEvaluate(&input) ==
               TC2_TRAFFIC_ACTION_KEEP_HOLD);
        input.rear_has_movement_authority = 1;
        input.train_a_effective_speed = 20;
        assert(Tc2TrafficSafetyEvaluate(&input) ==
               TC2_TRAFFIC_ACTION_KEEP_HOLD);

        /*
         * Restart uses the fixed 500 mm boundary.
         */
        input.train_a_effective_speed = 0;
        input.train_a_braking_envelope_mm = 300;
        input.gap_mm = 499;
        assert(Tc2TrafficSafetyEvaluate(&input) ==
               TC2_TRAFFIC_ACTION_KEEP_HOLD);
        input.gap_mm = 500;
        assert(Tc2TrafficSafetyEvaluate(&input) ==
               TC2_TRAFFIC_ACTION_RESUME_REAR);
}

static void test_invalid_input_fails_closed(void) {
        tc2_traffic_safety_input input = base_input();
        assert(Tc2TrafficSafetyEvaluate(0) ==
               TC2_TRAFFIC_ACTION_STOP_BOTH);

        input.train_a_route_scalar_mm = -1;
        assert(Tc2TrafficSafetyEvaluate(&input) ==
               TC2_TRAFFIC_ACTION_STOP_BOTH);
        input = base_input();
        input.train_b_effective_speed = -1;
        assert(Tc2TrafficSafetyEvaluate(&input) ==
               TC2_TRAFFIC_ACTION_STOP_BOTH);
        input = base_input();
        input.train_a_braking_envelope_mm = -1;
        assert(Tc2TrafficSafetyEvaluate(&input) ==
               TC2_TRAFFIC_ACTION_STOP_BOTH);
        input = base_input();
        input.train_a_braking_envelope_mm = 0;
        assert(Tc2TrafficSafetyEvaluate(&input) ==
               TC2_TRAFFIC_ACTION_STOP_BOTH);
        input = base_input();
        input.gap_mm = -1;
        assert(Tc2TrafficSafetyEvaluate(&input) ==
               TC2_TRAFFIC_ACTION_STOP_BOTH);
        input = base_input();
        input.rear_stop_latched = 2;
        assert(Tc2TrafficSafetyEvaluate(&input) ==
               TC2_TRAFFIC_ACTION_STOP_BOTH);
        input = base_input();
        input.rear_has_movement_authority = -1;
        assert(Tc2TrafficSafetyEvaluate(&input) ==
               TC2_TRAFFIC_ACTION_STOP_BOTH);
        input = base_input();
        input.relation = (tc2_traffic_relation)99;
        assert(Tc2TrafficSafetyEvaluate(&input) ==
               TC2_TRAFFIC_ACTION_STOP_BOTH);
}

static void test_stationary_vehicle_allows_zero_envelope(void) {
        tc2_traffic_safety_input input = base_input();
        input.train_a_effective_speed = 0;
        input.train_a_braking_envelope_mm = 0;
        input.gap_mm = TC2_TRAFFIC_FOLLOWING_STOP_DISTANCE_MM;
        assert(Tc2TrafficSafetyEvaluate(&input) ==
               TC2_TRAFFIC_ACTION_STOP_REAR);
}

int main(void) {
        assert(TC2_TRAFFIC_HEAD_ON_EXTRA_CLEARANCE_MM == 200);
        assert(TC2_TRAFFIC_FOLLOWING_STOP_CLEARANCE_MM == 200);
        assert(TC2_TRAFFIC_FOLLOWING_RESUME_CLEARANCE_MM == 400);
        assert(TC2_TRAFFIC_HEAD_ON_STOP_DISTANCE_MM == 1200);
        assert(TC2_TRAFFIC_FOLLOWING_STOP_DISTANCE_MM == 300);
        assert(TC2_TRAFFIC_FOLLOWING_RESUME_DISTANCE_MM == 500);

        test_head_on_boundary();
        test_head_on_uses_fixed_pair_distance();
        test_following_rear_selection_and_boundary();
        test_following_300_500_hysteresis_sequence();
        test_equal_speed_following_still_protects_rear();
        test_following_hold_and_resume_hysteresis();
        test_invalid_input_fails_closed();
        test_stationary_vehicle_allows_zero_envelope();

        puts("tc2_traffic_safety_test: PASS");
        return 0;
}
