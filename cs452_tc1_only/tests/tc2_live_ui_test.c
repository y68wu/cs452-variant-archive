#include <stdio.h>
#include <string.h>

#include "tc2_live_ui.h"

static char output[32768];
static size_t output_length;

static int sink(void *context, const char *bytes, size_t length) {
        (void)context;
        if (!bytes || output_length + length > sizeof(output)) return -1;
        memcpy(output + output_length, bytes, length);
        output_length += length;
        return 0;
}

static int fail(const char *message) {
        fprintf(stderr, "tc2_live_ui_test: %s\n", message);
        return 1;
}

int main(void) {
        tc2_live_ui ui;
        tc2_dispatch_projection_header header;
        tc2_dispatch_projection_waypoint waypoints[3];
        tc2_dispatch_snapshot dispatch;
        train_sensor_snapshot_t sensors;
        tc2_track_d_directed_sensor_cell a1;
        tc2_track_d_directed_sensor_cell a2;
        tc2_track_d_directed_sensor_cell d3;
        const tc2_track_d_start_layout *start =
                Tc2TrackDStartLayout(0);
        const tc2_track_d_destination_layout *destination =
                Tc2TrackDDestinationLayout(0);
        if (!start || !destination ||
            Tc2TrackDDirectedSensorCell(0, &a1) < 0 ||
            Tc2TrackDDirectedSensorCell(1, &a2) < 0 ||
            Tc2TrackDDirectedSensorCell(50, &d3) < 0) {
                return fail("layout fixture unavailable");
        }

        memset(&header, 0, sizeof(header));
        memset(waypoints, 0, sizeof(waypoints));
        header.valid = 1;
        header.train = 14;
        header.plan_generation = 3;
        header.launch_epoch = 1;
        header.start_index = 0;
        header.destination_index = 0;
        header.speed = 80;
        header.waypoint_count = 3;
        waypoints[0].sensor_index = -1;
        waypoints[0].ui_row = start->row;
        waypoints[0].kind = TC2_ROUTE_WAYPOINT_START;
        waypoints[0].ui_column = start->column;
        waypoints[0].ui_width = 1;
        waypoints[1].sensor_index = 0;
        waypoints[1].kind = TC2_ROUTE_WAYPOINT_SENSOR;
        waypoints[1].distance_um = 500000;
        waypoints[1].ui_row = a1.row;
        waypoints[1].ui_column = a1.column;
        waypoints[1].ui_width = a1.width;
        waypoints[2].sensor_index = -1;
        waypoints[2].kind = TC2_ROUTE_WAYPOINT_DESTINATION;
        waypoints[2].destination_index = 0;
        waypoints[2].distance_um = 1500000;
        waypoints[2].ui_row = destination->row;
        waypoints[2].ui_column = destination->column;
        waypoints[2].ui_width = 1;

        Tc2LiveUiInitialize(&ui, 100);
        if (Tc2LiveUiAcceptProjection(
                    &ui, &header, waypoints, 3, 100) < 0 ||
            !Tc2LiveUiHasPlan(&ui, 14, 3)) {
                return fail("projection was not accepted");
        }

        memset(&dispatch, 0, sizeof(dispatch));
        dispatch.jobs[0].train = 14;
        dispatch.jobs[0].state = TC2_JOB_RUNNING;
        dispatch.jobs[0].plan_generation = 3;
        dispatch.jobs[0].start_index = 0;
        dispatch.jobs[0].destination_index = 0;
        dispatch.jobs[0].speed = 80;
        memset(&sensors, 0, sizeof(sensors));
        sensors.last_attributed_train_by_sensor[0] = 14;
        sensors.last_attributed_generation_by_sensor[0] = 3;
        sensors.last_attributed_sequence_by_sensor[0] = 7;
        if (Tc2LiveUiSync(&ui, &dispatch, &sensors, 110) < 0) {
                return fail("live sensor sync failed");
        }
        const tc2_ui_train_overlay *train =
                Tc2UiOverlayFindTrain(&ui.overlay, 14);
        const tc2_ui_sensor_overlay *flash =
                Tc2UiOverlaySensor(&ui.overlay, 0);
        if (!train ||
            train->position_kind != TC2_UI_POSITION_PREDICTED ||
            train->position_quality != TC2_UI_QUALITY_PREDICTED ||
            train->row != a1.row ||
            train->column != a1.column ||
            !flash || !flash->active ||
            flash->evidence_source != TC2_UI_SOURCE_LIVE_CAN) {
                return fail(
                        "physical sensor did not advance the marker and flash");
        }

        if (Tc2LiveUiSync(&ui, &dispatch, &sensors, 111) < 0 ||
            !(train = Tc2UiOverlayFindTrain(&ui.overlay, 14)) ||
            train->position_kind != TC2_UI_POSITION_PREDICTED ||
            train->row != a1.row ||
            train->column != a1.column ||
            ui.trains[0].current_speed != 80) {
                return fail(
                        "marker moved without a new physical sensor");
        }

        dispatch.jobs[0].prediction_velocity_um_per_tick = 10000;
        dispatch.jobs[0].prediction_physical_destination_distance_um =
                1500000;
        if (Tc2LiveUiSync(&ui, &dispatch, &sensors, 160) < 0 ||
            !(train = Tc2UiOverlayFindTrain(&ui.overlay, 14)) ||
            train->position_kind != TC2_UI_POSITION_PREDICTED ||
            !Tc2TrackDLayoutCellIsOccupied(
                    train->row, train->column) ||
            (train->row == a1.row &&
             train->column == a1.column)) {
                return fail(
                        "live prediction left the track or did not animate");
        }
        sensors.last_attributed_sequence_by_sensor[0] = 8;
        if (Tc2LiveUiSync(&ui, &dispatch, &sensors, 161) < 0 ||
            !(train = Tc2UiOverlayFindTrain(&ui.overlay, 14)) ||
            train->position_kind != TC2_UI_POSITION_PREDICTED ||
            ui.trains[0].displayed_distance_um < 1000000 ||
            !(flash = Tc2UiOverlaySensor(&ui.overlay, 0)) ||
            !flash->active || flash->sequence != 8) {
                return fail(
                        "delayed sensor moved prediction backward or did "
                        "not flash");
        }
        int held_row = train->row;
        int held_column = train->column;
        dispatch.jobs[0].state = TC2_JOB_WAITING;
        if (Tc2LiveUiSync(&ui, &dispatch, &sensors, 250) < 0 ||
            !(train = Tc2UiOverlayFindTrain(&ui.overlay, 14)) ||
            train->row != held_row ||
            train->column != held_column) {
                return fail("route wait advanced the predicted train");
        }
        dispatch.jobs[0].state = TC2_JOB_RUNNING;
        if (Tc2LiveUiSync(&ui, &dispatch, &sensors, 300) < 0 ||
            !(train = Tc2UiOverlayFindTrain(&ui.overlay, 14)) ||
            train->row != held_row ||
            train->column != held_column) {
                return fail("route wait time leaked into travel time");
        }
        if (Tc2LiveUiSync(&ui, &dispatch, &sensors, 350) < 0 ||
            !(train = Tc2UiOverlayFindTrain(&ui.overlay, 14)) ||
            train->position_kind != TC2_UI_POSITION_PREDICTED ||
            !Tc2TrackDLayoutCellIsOccupied(
                    train->row, train->column) ||
            (train->row == a1.row &&
             train->column == a1.column)) {
                return fail("resumed prediction did not advance");
        }
        dispatch.jobs[0].state = TC2_JOB_BRAKING;
        if (Tc2LiveUiSync(&ui, &dispatch, &sensors, 360) < 0 ||
            !(train = Tc2UiOverlayFindTrain(&ui.overlay, 14)) ||
            train->position_kind != TC2_UI_POSITION_PREDICTED ||
            !Tc2TrackDLayoutCellIsOccupied(
                    train->row, train->column)) {
                return fail("braking coast froze the predicted marker");
        }

        tc2_live_ui_render_result rendered;
        output_length = 0;
        if (Tc2LiveUiRender(
                    &ui, "\033[0m", 4, &rendered) < 0 ||
            Tc2LiveUiDrain(&ui, sink, 0, 256, 0) < 0 ||
            rendered.renderer.invalid_evidence != 0 ||
            output_length == 0) {
                return fail("live frame did not render and drain");
        }

        int canceled_row = train->row;
        int canceled_column = train->column;
        dispatch.jobs[0].state = TC2_JOB_STOPPED;
        if (Tc2LiveUiSync(&ui, &dispatch, &sensors, 369) < 0 ||
            !(train = Tc2UiOverlayFindTrain(&ui.overlay, 14)) ||
            train->position_kind == TC2_UI_POSITION_DESTINATION ||
            train->row != canceled_row ||
            train->column != canceled_column) {
                return fail(
                        "cancelled train jumped to its unvisited destination");
        }
        dispatch.jobs[0].state = TC2_JOB_ARRIVED;
        if (Tc2LiveUiSync(&ui, &dispatch, &sensors, 370) < 0 ||
            !(train = Tc2UiOverlayFindTrain(&ui.overlay, 14)) ||
            train->position_kind != TC2_UI_POSITION_DESTINATION ||
            train->position_quality != TC2_UI_QUALITY_ESTIMATED ||
            ui.trains[0].current_speed != 0) {
                return fail(
                        "arrival did not settle at the simulated destination");
        }

        /*
         * A destination-clipped projection cannot contain a detector beyond
         * the requested endpoint.  If CAN nevertheless attributes D3 after
         * the modeled arrival, physical evidence must replace the virtual
         * destination marker and remain authoritative on later refreshes.
         */
        sensors.last_attributed_train_by_sensor[50] = 14;
        sensors.last_attributed_generation_by_sensor[50] = 3;
        sensors.last_attributed_sequence_by_sensor[50] = 9;
        if (Tc2LiveUiSync(&ui, &dispatch, &sensors, 371) < 0 ||
            !(train = Tc2UiOverlayFindTrain(&ui.overlay, 14)) ||
            train->position_kind != TC2_UI_POSITION_SENSOR ||
            train->row != d3.row || train->column != d3.column ||
            Tc2LiveUiSync(&ui, &dispatch, &sensors, 372) < 0 ||
            !(train = Tc2UiOverlayFindTrain(&ui.overlay, 14)) ||
            train->position_kind != TC2_UI_POSITION_SENSOR ||
            train->row != d3.row || train->column != d3.column) {
                return fail(
                        "post-destination physical sensor was hidden by "
                        "the virtual arrival marker");
        }

        /*
         * A traffic stop is an intermediate protected hold, not an arrival.
         * Clamp the predicted marker to the dispatcher's coast-complete
         * distance across repeated refreshes.  Resuming must re-anchor time
         * at that exact scalar position so neither the first RUNNING frame nor
         * later motion can move '@' backwards.
         */
        header.plan_generation = 6;
        header.launch_epoch = 2;
        Tc2LiveUiInitialize(&ui, 380);
        if (Tc2LiveUiAcceptProjection(
                    &ui, &header, waypoints, 3, 380) < 0) {
                return fail("traffic-hold projection was not accepted");
        }
        memset(&dispatch, 0, sizeof(dispatch));
        dispatch.jobs[0].train = 14;
        dispatch.jobs[0].state = TC2_JOB_RUNNING;
        dispatch.jobs[0].plan_generation = 6;
        dispatch.jobs[0].start_index = 0;
        dispatch.jobs[0].destination_index = 0;
        dispatch.jobs[0].speed = 80;
        dispatch.jobs[0].command_speed = 80;
        dispatch.jobs[0].prediction_velocity_um_per_tick = 10000;
        dispatch.jobs[0].prediction_physical_destination_distance_um =
                1500000;
        memset(&sensors, 0, sizeof(sensors));
        if (Tc2LiveUiSync(&ui, &dispatch, &sensors, 380) < 0 ||
            Tc2LiveUiSync(&ui, &dispatch, &sensors, 400) < 0 ||
            ui.trains[0].displayed_distance_um != 200000) {
                return fail("traffic-hold setup did not reach its approach");
        }

        dispatch.jobs[0].state = TC2_JOB_TRAFFIC_HOLD;
        dispatch.jobs[0].command_speed = 0;
        dispatch.jobs[0].traffic_hold_active = 1;
        dispatch.jobs[0].traffic_hold_distance_um = 350000;
        int traffic_hold_row;
        int traffic_hold_column;
        if (!Tc2LiveUiInterpolateTrackCell(
                    &waypoints[0], &waypoints[1],
                    350000, 500000,
                    &traffic_hold_row, &traffic_hold_column) ||
            Tc2LiveUiSync(&ui, &dispatch, &sensors, 420) < 0 ||
            !(train = Tc2UiOverlayFindTrain(&ui.overlay, 14)) ||
            train->position_kind != TC2_UI_POSITION_PREDICTED ||
            train->row != traffic_hold_row ||
            train->column != traffic_hold_column ||
            ui.trains[0].displayed_distance_um != 350000 ||
            ui.trains[0].current_speed != 0) {
                return fail(
                        "traffic hold did not clamp '@' at its coast distance");
        }
        for (uint32_t tick = 430; tick <= 500; tick += 10) {
                if (Tc2LiveUiSync(
                            &ui, &dispatch, &sensors, tick) < 0 ||
                    !(train = Tc2UiOverlayFindTrain(
                            &ui.overlay, 14)) ||
                    train->row != traffic_hold_row ||
                    train->column != traffic_hold_column ||
                    ui.trains[0].displayed_distance_um != 350000) {
                        return fail(
                                "repeated traffic-hold refresh moved '@'");
                }
        }

        dispatch.jobs[0].state = TC2_JOB_RUNNING;
        dispatch.jobs[0].command_speed = 80;
        dispatch.jobs[0].traffic_hold_active = 0;
        if (Tc2LiveUiSync(&ui, &dispatch, &sensors, 501) < 0 ||
            !(train = Tc2UiOverlayFindTrain(&ui.overlay, 14)) ||
            ui.trains[0].displayed_distance_um != 350000 ||
            train->row != traffic_hold_row ||
            train->column != traffic_hold_column ||
            ui.trains[0].current_speed != 80) {
                return fail(
                        "traffic-hold resume frame did not preserve position");
        }
        int64_t resumed_distance =
                ui.trains[0].displayed_distance_um;
        if (Tc2LiveUiSync(&ui, &dispatch, &sensors, 511) < 0 ||
            !(train = Tc2UiOverlayFindTrain(&ui.overlay, 14)) ||
            ui.trains[0].displayed_distance_um <= resumed_distance ||
            !Tc2TrackDLayoutCellIsOccupied(
                    train->row, train->column)) {
                return fail(
                        "resumed traffic hold did not advance monotonically");
        }
        resumed_distance = ui.trains[0].displayed_distance_um;
        if (Tc2LiveUiSync(&ui, &dispatch, &sensors, 521) < 0 ||
            ui.trains[0].displayed_distance_um != 499999 ||
            ui.trains[0].displayed_distance_um <= resumed_distance) {
                return fail(
                        "prediction did not stop immediately before A1");
        }
        if (Tc2LiveUiSync(&ui, &dispatch, &sensors, 531) < 0 ||
            ui.trains[0].displayed_distance_um != 499999) {
                return fail(
                        "unconfirmed A1 gate allowed prediction to pass");
        }
        sensors.last_attributed_train_by_sensor[0] = 14;
        sensors.last_attributed_generation_by_sensor[0] = 6;
        sensors.last_attributed_sequence_by_sensor[0] = 1;
        if (Tc2LiveUiSync(&ui, &dispatch, &sensors, 532) < 0 ||
            ui.trains[0].displayed_distance_um < 500000 ||
            !(flash = Tc2UiOverlaySensor(&ui.overlay, 0)) ||
            !flash->active) {
                return fail(
                        "physical A1 edge did not flash and open its gate");
        }
        if (Tc2LiveUiSync(&ui, &dispatch, &sensors, 542) < 0 ||
            ui.trains[0].displayed_distance_um <= 500000) {
                return fail(
                        "prediction did not continue after physical A1");
        }

        /*
         * A->d1 reaches E12 and then climbs to the upper C4/C3 row before
         * C8.  This segment must follow drawn rail cells, and a later stale
         * sensor report must not pull the marker back off the route.
         */
        tc2_dispatch_projection_waypoint a_d1_waypoints[5];
        tc2_track_d_directed_sensor_cell d9;
        tc2_track_d_directed_sensor_cell e12;
        tc2_track_d_directed_sensor_cell c8;
        if (Tc2TrackDDirectedSensorCell(56, &d9) < 0 ||
            Tc2TrackDDirectedSensorCell(75, &e12) < 0 ||
            Tc2TrackDDirectedSensorCell(39, &c8) < 0) {
                return fail("A-to-d1 sensor fixture unavailable");
        }
        memset(a_d1_waypoints, 0, sizeof(a_d1_waypoints));
        header.plan_generation = 4;
        header.waypoint_count = 5;
        a_d1_waypoints[0].sensor_index = -1;
        a_d1_waypoints[0].kind = TC2_ROUTE_WAYPOINT_START;
        a_d1_waypoints[0].ui_row = start->row;
        a_d1_waypoints[0].ui_column = start->column;
        a_d1_waypoints[0].ui_width = 1;
        a_d1_waypoints[1].sensor_index = 56;
        a_d1_waypoints[1].kind = TC2_ROUTE_WAYPOINT_SENSOR;
        a_d1_waypoints[1].distance_um = 100000;
        a_d1_waypoints[1].ui_row = d9.row;
        a_d1_waypoints[1].ui_column = d9.column;
        a_d1_waypoints[1].ui_width = d9.width;
        a_d1_waypoints[2].sensor_index = 75;
        a_d1_waypoints[2].kind = TC2_ROUTE_WAYPOINT_SENSOR;
        a_d1_waypoints[2].distance_um = 700000;
        a_d1_waypoints[2].ui_row = e12.row;
        a_d1_waypoints[2].ui_column = e12.column;
        a_d1_waypoints[2].ui_width = e12.width;
        a_d1_waypoints[3].sensor_index = 39;
        a_d1_waypoints[3].kind = TC2_ROUTE_WAYPOINT_SENSOR;
        a_d1_waypoints[3].distance_um = 1700000;
        a_d1_waypoints[3].ui_row = c8.row;
        a_d1_waypoints[3].ui_column = c8.column;
        a_d1_waypoints[3].ui_width = c8.width;
        a_d1_waypoints[4].sensor_index = -1;
        a_d1_waypoints[4].kind =
                TC2_ROUTE_WAYPOINT_DESTINATION;
        a_d1_waypoints[4].destination_index = 0;
        a_d1_waypoints[4].distance_um = 2600000;
        a_d1_waypoints[4].ui_row = destination->row;
        a_d1_waypoints[4].ui_column = destination->column;
        a_d1_waypoints[4].ui_width = 1;
        Tc2LiveUiInitialize(&ui, 400);
        if (Tc2LiveUiAcceptProjection(
                    &ui, &header, a_d1_waypoints, 5, 400) < 0) {
                return fail("A-to-d1 projection was not accepted");
        }
        memset(&dispatch, 0, sizeof(dispatch));
        dispatch.jobs[0].train = 14;
        dispatch.jobs[0].state = TC2_JOB_RUNNING;
        dispatch.jobs[0].plan_generation = 4;
        dispatch.jobs[0].start_index = 0;
        dispatch.jobs[0].destination_index = 0;
        dispatch.jobs[0].speed = 80;
        dispatch.jobs[0].prediction_velocity_um_per_tick = 100000;
        dispatch.jobs[0].prediction_physical_destination_distance_um =
                2600000;
        memset(&sensors, 0, sizeof(sensors));
        sensors.last_attributed_train_by_sensor[56] = 14;
        sensors.last_attributed_generation_by_sensor[56] = 4;
        sensors.last_attributed_sequence_by_sensor[56] = 1;
        if (Tc2LiveUiSync(&ui, &dispatch, &sensors, 410) < 0 ||
            !(train = Tc2UiOverlayFindTrain(&ui.overlay, 14)) ||
            train->position_kind != TC2_UI_POSITION_PREDICTED ||
            train->row != d9.row || train->column != d9.column ||
            !(flash = Tc2UiOverlaySensor(&ui.overlay, 56)) ||
            !flash->active) {
                return fail("D9 did not anchor A-to-d1 visualization");
        }
        static const int d9_e12_rows[5] = {10, 8, 7, 5, 5};
        for (int tick = 411; tick <= 415; ++tick) {
                if (Tc2LiveUiSync(
                            &ui, &dispatch, &sensors,
                            (uint32_t)tick) < 0 ||
                    !(train = Tc2UiOverlayFindTrain(
                            &ui.overlay, 14)) ||
                    train->position_kind !=
                            TC2_UI_POSITION_PREDICTED ||
                    train->row != d9_e12_rows[tick - 411] ||
                    !Tc2TrackDLayoutCellIsOccupied(
                            train->row, train->column)) {
                        return fail(
                                "D9-to-E12 marker skipped a drawn rail cell");
                }
        }
        sensors.last_attributed_train_by_sensor[75] = 14;
        sensors.last_attributed_generation_by_sensor[75] = 4;
        sensors.last_attributed_sequence_by_sensor[75] = 2;
        if (Tc2LiveUiSync(&ui, &dispatch, &sensors, 417) < 0 ||
            !(train = Tc2UiOverlayFindTrain(&ui.overlay, 14)) ||
            train->position_kind != TC2_UI_POSITION_PREDICTED ||
            ui.trains[0].displayed_distance_um < 700000 ||
            !Tc2TrackDLayoutCellIsOccupied(
                    train->row, train->column) ||
            !(flash = Tc2UiOverlaySensor(&ui.overlay, 75)) ||
            !flash->active) {
                return fail("E12 did not confirm without a backward jump");
        }
        if (Tc2LiveUiSync(&ui, &dispatch, &sensors, 419) < 0 ||
            !(train = Tc2UiOverlayFindTrain(&ui.overlay, 14)) ||
            train->position_kind != TC2_UI_POSITION_PREDICTED ||
            train->row != 5 ||
            !Tc2TrackDLayoutCellIsOccupied(
                    train->row, train->column)) {
                return fail(
                        "E12-to-C8 marker skipped the E11/E12 track row");
        }
        if (Tc2LiveUiSync(&ui, &dispatch, &sensors, 420) < 0 ||
            !(train = Tc2UiOverlayFindTrain(&ui.overlay, 14)) ||
            train->position_kind != TC2_UI_POSITION_PREDICTED ||
            train->row != 1 ||
            !Tc2TrackDLayoutCellIsOccupied(
                    train->row, train->column)) {
                return fail(
                        "E12-to-C8 marker did not enter the upper track row");
        }
        int upper_column = train->column;
        sensors.last_attributed_train_by_sensor[0] = 14;
        sensors.last_attributed_generation_by_sensor[0] = 4;
        sensors.last_attributed_sequence_by_sensor[0] = 3;
        if (Tc2LiveUiSync(&ui, &dispatch, &sensors, 421) < 0 ||
            !(train = Tc2UiOverlayFindTrain(&ui.overlay, 14)) ||
            train->position_kind != TC2_UI_POSITION_PREDICTED ||
            train->row != 1 ||
            train->column < upper_column) {
                return fail(
                        "stale sensor event pulled marker off/back on route");
        }
        if (Tc2LiveUiSync(&ui, &dispatch, &sensors, 440) < 0 ||
            !(train = Tc2UiOverlayFindTrain(&ui.overlay, 14)) ||
            train->position_kind != TC2_UI_POSITION_PREDICTED ||
            ui.trains[0].displayed_distance_um != 1699999 ||
            !Tc2TrackDLayoutCellIsOccupied(
                    train->row, train->column)) {
                return fail(
                        "prediction did not wait immediately before C8");
        }
        int64_t before_delayed_c8_distance =
                ui.trains[0].displayed_distance_um;
        sensors.last_attributed_train_by_sensor[39] = 14;
        sensors.last_attributed_generation_by_sensor[39] = 4;
        sensors.last_attributed_sequence_by_sensor[39] = 4;
        if (Tc2LiveUiSync(&ui, &dispatch, &sensors, 441) < 0 ||
            !(train = Tc2UiOverlayFindTrain(&ui.overlay, 14)) ||
            train->position_kind != TC2_UI_POSITION_PREDICTED ||
            ui.trains[0].displayed_distance_um <
                    before_delayed_c8_distance ||
            ui.trains[0].displayed_distance_um < 1700000 ||
            !Tc2TrackDLayoutCellIsOccupied(
                    train->row, train->column) ||
            !(flash = Tc2UiOverlaySensor(&ui.overlay, 39)) ||
            !flash->active) {
                return fail(
                        "physical C8 confirmation did not open its gate");
        }

        /*
         * Exercise the complete physical C->d1 shortest route.  Sensor
         * evidence flashes and rebases distance, but every red-train cell
         * must continue through the ordered '-', '/', '\\', and '|' cells:
         *
         * B7 -> A10 -> C7 -> E11 -> D10 -> D8 -> E8 -> C14 -> A15 -> d1.
         */
        enum { C_D1_SENSOR_COUNT = 9 };
        static const int c_d1_sensors[C_D1_SENSOR_COUNT] = {
                22, 9, 38, 74, 57, 55, 71, 45, 14
        };
        static const int c_d1_distance_mm[C_D1_SENSOR_COUNT] = {
                43, 332, 1062, 1862, 2231,
                3011, 3387, 4262, 4960
        };
        static const int c_d1_boundary_row[C_D1_SENSOR_COUNT] = {
                9, 9, 1, 5, 11, 33, 35, 35, 29
        };
        static const int c_d1_boundary_column[C_D1_SENSOR_COUNT] = {
                106, 94, 61, 7, 0, 0, 18, 64, 93
        };
        tc2_dispatch_projection_waypoint
                c_d1_waypoints[C_D1_SENSOR_COUNT + 2];
        const tc2_track_d_start_layout *start_c =
                Tc2TrackDStartLayout(2);
        if (!start_c) {
                return fail("C-to-d1 sensor fixture unavailable");
        }
        memset(c_d1_waypoints, 0, sizeof(c_d1_waypoints));
        header.plan_generation = 5;
        header.start_index = 2;
        header.speed = 100;
        header.waypoint_count = C_D1_SENSOR_COUNT + 2;
        c_d1_waypoints[0].sensor_index = -1;
        c_d1_waypoints[0].kind = TC2_ROUTE_WAYPOINT_START;
        c_d1_waypoints[0].ui_row = start_c->row;
        c_d1_waypoints[0].ui_column = start_c->column;
        c_d1_waypoints[0].ui_width = 1;
        for (int index = 0;
             index < C_D1_SENSOR_COUNT; ++index) {
                tc2_track_d_directed_sensor_cell cell;
                if (Tc2TrackDDirectedSensorCell(
                            c_d1_sensors[index], &cell) < 0) {
                        return fail(
                                "C-to-d1 directed sensor unavailable");
                }
                c_d1_waypoints[index + 1].sensor_index =
                        c_d1_sensors[index];
                c_d1_waypoints[index + 1].kind =
                        TC2_ROUTE_WAYPOINT_SENSOR;
                c_d1_waypoints[index + 1].distance_um =
                        (int64_t)c_d1_distance_mm[index] * 1000;
                c_d1_waypoints[index + 1].ui_row = cell.row;
                c_d1_waypoints[index + 1].ui_column =
                        cell.column;
                c_d1_waypoints[index + 1].ui_width =
                        cell.width;
        }
        c_d1_waypoints[C_D1_SENSOR_COUNT + 1].sensor_index = -1;
        c_d1_waypoints[C_D1_SENSOR_COUNT + 1].kind =
                TC2_ROUTE_WAYPOINT_DESTINATION;
        c_d1_waypoints[C_D1_SENSOR_COUNT + 1]
                .destination_index = 0;
        c_d1_waypoints[C_D1_SENSOR_COUNT + 1].distance_um =
                5341000;
        c_d1_waypoints[C_D1_SENSOR_COUNT + 1].ui_row =
                destination->row;
        c_d1_waypoints[C_D1_SENSOR_COUNT + 1].ui_column =
                destination->column;
        c_d1_waypoints[C_D1_SENSOR_COUNT + 1].ui_width = 1;
        Tc2LiveUiInitialize(&ui, 500);
        if (Tc2LiveUiAcceptProjection(
                    &ui, &header, c_d1_waypoints,
                    C_D1_SENSOR_COUNT + 2, 500) < 0) {
                return fail("C-to-d1 projection was not accepted");
        }
        memset(&dispatch, 0, sizeof(dispatch));
        dispatch.jobs[0].train = 14;
        dispatch.jobs[0].state = TC2_JOB_RUNNING;
        dispatch.jobs[0].plan_generation = 5;
        dispatch.jobs[0].start_index = 2;
        dispatch.jobs[0].destination_index = 0;
        dispatch.jobs[0].speed = 100;
        dispatch.jobs[0].prediction_velocity_um_per_tick = 1000;
        dispatch.jobs[0].prediction_physical_destination_distance_um =
                5341000;
        memset(&sensors, 0, sizeof(sensors));
        int sensor_cursor = 0;
        int prior_row = start_c->row;
        int prior_column = start_c->column;
        int distinct_cells = 0;
        for (int distance_mm = 1;
             distance_mm <= 5341; ++distance_mm) {
                int boundary = sensor_cursor < C_D1_SENSOR_COUNT &&
                        distance_mm ==
                                c_d1_distance_mm[sensor_cursor];
                if (boundary) {
                        int sensor =
                                c_d1_sensors[sensor_cursor];
                        sensors.last_attributed_train_by_sensor[
                                sensor] = 14;
                        sensors.last_attributed_generation_by_sensor[
                                sensor] = 5;
                        sensors.last_attributed_sequence_by_sensor[
                                sensor] =
                                (unsigned int)sensor_cursor + 1;
                }
                if (Tc2LiveUiSync(
                            &ui, &dispatch, &sensors,
                            (uint32_t)(500 + distance_mm)) < 0 ||
                    !(train = Tc2UiOverlayFindTrain(
                            &ui.overlay, 14)) ||
                    train->position_kind !=
                            TC2_UI_POSITION_PREDICTED ||
                    !Tc2TrackDLayoutCellIsOccupied(
                            train->row, train->column)) {
                        return fail(
                                "C-to-d1 marker left the drawn rail");
                }
                if (boundary) {
                        int sensor =
                                c_d1_sensors[sensor_cursor];
                        flash = Tc2UiOverlaySensor(
                                &ui.overlay, sensor);
                        if (train->row !=
                                    c_d1_boundary_row[
                                            sensor_cursor] ||
                            train->column !=
                                    c_d1_boundary_column[
                                            sensor_cursor] ||
                            !flash || !flash->active ||
                            flash->sequence !=
                                    (unsigned int)sensor_cursor +
                                            1) {
                                return fail(
                                        "C-to-d1 sensor boundary moved "
                                        "or failed to flash");
                        }
                        ++sensor_cursor;
                }
                if (train->row != prior_row ||
                    train->column != prior_column) {
                        ++distinct_cells;
                }
                prior_row = train->row;
                prior_column = train->column;
        }
        if (sensor_cursor != C_D1_SENSOR_COUNT ||
            train->row != destination->row ||
            train->column != destination->column ||
            distinct_cells < 150) {
                return fail(
                        "C-to-d1 route skipped or snapped between rails");
        }

        /*
         * A CURRENT recovery replaces the dispatch generation while the
         * physical train remains on the same rail. A one-cell localization
         * handoff must retain that marker and still flash attributed CAN
         * sensors. The later exact projection must begin at the retained
         * cell, not jump to its graph anchor or briefly disappear.
         */
        int current_row = train->row;
        int current_column = train->column;
        int current_color = train->color_slot;
        tc2_dispatch_projection_header current_header;
        tc2_dispatch_projection_waypoint current_waypoints[3];
        memset(&current_header, 0, sizeof(current_header));
        memset(current_waypoints, 0, sizeof(current_waypoints));
        current_header.valid = 1;
        current_header.train = 14;
        current_header.plan_generation = 6;
        current_header.launch_epoch = 3;
        current_header.start_index =
                TC2_DISPATCH_START_CURRENT;
        current_header.destination_index = 1;
        current_header.speed = 40;
        current_header.waypoint_count = 1;
        current_waypoints[0].kind =
                TC2_ROUTE_WAYPOINT_START;
        current_waypoints[0].sensor_index = -1;
        current_waypoints[0].ui_row = a1.row;
        current_waypoints[0].ui_column = a1.column;
        current_waypoints[0].ui_width = 1;
        if (Tc2LiveUiAcceptProjection(
                    &ui, &current_header,
                    current_waypoints, 1, 6000) < 0 ||
            Tc2LiveUiHasPlan(&ui, 14, 6) ||
            !(train = Tc2UiOverlayFindTrain(
                    &ui.overlay, 14)) ||
            train->row != current_row ||
            train->column != current_column ||
            train->color_slot != current_color ||
            ui.trains[0].start_index !=
                    TC2_DISPATCH_START_CURRENT ||
            ui.trains[0].waypoints[0].ui_row !=
                    current_row ||
            ui.trains[0].waypoints[0].ui_column !=
                    current_column) {
                return fail(
                        "CURRENT localization generation moved or removed "
                        "the train marker");
        }
        memset(&dispatch, 0, sizeof(dispatch));
        dispatch.jobs[0].train = 14;
        dispatch.jobs[0].state = TC2_JOB_TRAFFIC_HOLD;
        dispatch.jobs[0].plan_generation = 6;
        dispatch.jobs[0].start_index =
                TC2_DISPATCH_START_CURRENT;
        dispatch.jobs[0].destination_index = 1;
        dispatch.jobs[0].speed = 40;
        dispatch.jobs[0].prediction_velocity_um_per_tick = 1000;
        dispatch.jobs[0].prediction_physical_destination_distance_um =
                1000000;
        dispatch.jobs[0].traffic_hold_active = 1;
        dispatch.jobs[0].traffic_hold_distance_um = 0;
        memset(&sensors, 0, sizeof(sensors));
        sensors.last_attributed_train_by_sensor[0] = 14;
        sensors.last_attributed_generation_by_sensor[0] = 6;
        sensors.last_attributed_sequence_by_sensor[0] = 20;
        if (Tc2LiveUiSync(
                    &ui, &dispatch, &sensors, 6001) < 0 ||
            !(train = Tc2UiOverlayFindTrain(
                    &ui.overlay, 14)) ||
            train->row != current_row ||
            train->column != current_column ||
            !(flash = Tc2UiOverlaySensor(
                    &ui.overlay, 0)) ||
            !flash->active || flash->sequence != 20) {
                return fail(
                        "CURRENT localization did not freeze '@' and flash "
                        "its physical sensor");
        }

        const tc2_track_d_destination_layout *destination_d2 =
                Tc2TrackDDestinationLayout(1);
        if (!destination_d2) {
                return fail("d2 layout unavailable");
        }
        current_header.waypoint_count = 3;
        /* Coordinate equality is not physical localization. */
        current_waypoints[0].ui_row = (uint8_t)current_row;
        current_waypoints[0].ui_column = (uint8_t)current_column;
        current_waypoints[1].kind =
                TC2_ROUTE_WAYPOINT_SENSOR;
        current_waypoints[1].sensor_index = 0;
        current_waypoints[1].distance_um = 500000;
        current_waypoints[1].ui_row = a1.row;
        current_waypoints[1].ui_column = a1.column;
        current_waypoints[1].ui_width = a1.width;
        current_waypoints[2].kind =
                TC2_ROUTE_WAYPOINT_DESTINATION;
        current_waypoints[2].sensor_index = -1;
        current_waypoints[2].destination_index = 1;
        current_waypoints[2].distance_um = 1000000;
        current_waypoints[2].ui_row =
                destination_d2->row;
        current_waypoints[2].ui_column =
                destination_d2->column;
        current_waypoints[2].ui_width = 1;
        /*
         * The exact projection normally appears after the one-cell
         * localization record but keeps the same plan generation. It must
         * upgrade that placeholder atomically rather than being ignored as
         * a duplicate.
         */
        if (Tc2LiveUiAcceptProjection(
                    &ui, &current_header,
                    current_waypoints, 3, 6010) < 0 ||
            !Tc2LiveUiHasPlan(&ui, 14, 6) ||
            !(train = Tc2UiOverlayFindTrain(
                    &ui.overlay, 14)) ||
            train->row != current_row ||
            train->column != current_column ||
            train->color_slot != current_color ||
            ui.trains[0].waypoint_count != 3 ||
            ui.trains[0].waypoints[0].ui_row !=
                    current_row ||
            ui.trains[0].waypoints[0].ui_column !=
                    current_column ||
            !ui.trains[0].awaiting_current_sensor) {
                return fail(
                        "exact CURRENT route did not continue from the "
                        "retained physical cell");
        }

        /*
         * CURRENT is the real start kind for every reroute.  It is not an
         * invalid A-F index: the retained predicted rail cell is its visual
         * origin.  The renderer must therefore keep both the train header
         * and '@' marker renderable instead of painting the header as a red
         * warning and suppressing the marker for the whole reroute.
         */
        tc2_ui_render_result current_render_result;
        output_length = 0;
        Tc2UiRendererInit(&ui.renderer);
        if (Tc2UiRendererRender(
                    &ui.renderer, &ui.overlay, 6010,
                    sink, 0, &current_render_result) < 0 ||
            current_render_result.invalid_evidence != 0) {
                return fail("CURRENT reroute frame did not render");
        }
        int retained_marker_visible = 0;
        for (size_t byte = 0;
             byte < output_length; ++byte) {
                if (output[byte] == '@') {
                        retained_marker_visible = 1;
                        break;
                }
        }
        if (!retained_marker_visible) {
                return fail(
                        "CURRENT reroute marker was hidden by renderer "
                        "start validation");
        }
        memset(&dispatch, 0, sizeof(dispatch));
        dispatch.jobs[0].train = 14;
        dispatch.jobs[0].state = TC2_JOB_RUNNING;
        dispatch.jobs[0].plan_generation = 6;
        dispatch.jobs[0].start_index =
                TC2_DISPATCH_START_CURRENT;
        dispatch.jobs[0].destination_index = 1;
        dispatch.jobs[0].speed = 40;
        dispatch.jobs[0].command_speed = 40;
        dispatch.jobs[0].prediction_velocity_um_per_tick = 1000;
        dispatch.jobs[0].prediction_physical_destination_distance_um =
                1000000;
        for (uint32_t tick = 6011; tick < 6020; ++tick) {
                if (Tc2LiveUiSync(
                            &ui, &dispatch,
                            &sensors, tick) < 0 ||
                    !(train = Tc2UiOverlayFindTrain(
                            &ui.overlay, 14)) ||
                    train->row != current_row ||
                    train->column != current_column) {
                        return fail(
                                "CURRENT moved before its replacement "
                                "route was physically confirmed");
                }
        }
        sensors.last_attributed_sequence_by_sensor[0] = 21;
        if (Tc2LiveUiSync(
                    &ui, &dispatch, &sensors, 6020) < 0 ||
            !(train = Tc2UiOverlayFindTrain(
                    &ui.overlay, 14)) ||
            train->row != a1.row ||
            train->column != a1.column ||
            ui.trains[0].awaiting_current_sensor) {
                return fail(
                        "CURRENT did not transfer monotonically at its "
                        "first confirmed route sensor");
        }
        current_row = train->row;
        current_column = train->column;

        current_header.plan_generation = 7;
        current_header.launch_epoch = 4;
        if (Tc2LiveUiAcceptProjection(
                    &ui, &current_header,
                    current_waypoints, 3, 6030) < 0 ||
            !(train = Tc2UiOverlayFindTrain(
                    &ui.overlay, 14)) ||
            train->row != current_row ||
            train->column != current_column ||
            train->color_slot != current_color) {
                return fail(
                        "new CURRENT generation did not atomically retain "
                        "the physical marker");
        }
        current_header.plan_generation = 6;
        if (Tc2LiveUiAcceptProjection(
                    &ui, &current_header,
                    current_waypoints, 3, 6031) != 0 ||
            !Tc2LiveUiHasPlan(&ui, 14, 7) ||
            !(train = Tc2UiOverlayFindTrain(
                    &ui.overlay, 14)) ||
            train->row != current_row ||
            train->column != current_column) {
                return fail(
                        "stale CURRENT generation replaced the live route");
        }

        /*
         * An old dispatch snapshot must not delete a newer accepted
         * projection, and a new snapshot whose projection is not published
         * yet must not delete the prior marker. Both are normal IPC races.
         */
        memset(&dispatch, 0, sizeof(dispatch));
        dispatch.jobs[0].train = 14;
        dispatch.jobs[0].state = TC2_JOB_TRAFFIC_HOLD;
        dispatch.jobs[0].plan_generation = 6;
        if (Tc2LiveUiSync(
                    &ui, &dispatch, &sensors, 6040) < 0 ||
            !Tc2LiveUiHasPlan(&ui, 14, 7) ||
            !(train = Tc2UiOverlayFindTrain(
                    &ui.overlay, 14)) ||
            train->row != current_row ||
            train->column != current_column ||
            train->color_slot != current_color) {
                return fail(
                        "older dispatch snapshot deleted a newer UI plan");
        }
        dispatch.jobs[0].plan_generation = 8;
        if (Tc2LiveUiSync(
                    &ui, &dispatch, &sensors, 6041) < 0 ||
            !Tc2LiveUiHasPlan(&ui, 14, 7) ||
            !(train = Tc2UiOverlayFindTrain(
                    &ui.overlay, 14)) ||
            train->row != current_row ||
            train->column != current_column) {
                return fail(
                        "new snapshot without projection caused a UI gap");
        }

        /*
         * Validate every newer payload before touching the overlay
         * generation high-water mark. A corrected retry with the same
         * generation must remain admissible after a malformed attempt.
         */
        tc2_live_ui before_malformed;
        memcpy(&before_malformed, &ui, sizeof(ui));
        current_header.plan_generation = 8;
        current_header.launch_epoch = 5;
        current_waypoints[2].distance_um = -1;
        if (Tc2LiveUiAcceptProjection(
                    &ui, &current_header,
                    current_waypoints, 3, 6050) != -1 ||
            memcmp(&before_malformed, &ui, sizeof(ui)) != 0) {
                return fail(
                        "malformed projection partially mutated the UI");
        }
        current_waypoints[2].distance_um = 1000000;
        if (Tc2LiveUiAcceptProjection(
                    &ui, &current_header,
                    current_waypoints, 3, 6051) < 0 ||
            !Tc2LiveUiHasPlan(&ui, 14, 8) ||
            !(train = Tc2UiOverlayFindTrain(
                    &ui.overlay, 14)) ||
            train->row != current_row ||
            train->column != current_column ||
            train->color_slot != current_color) {
                return fail(
                        "corrected same-generation projection was poisoned "
                        "by a malformed predecessor");
        }

        /*
         * The physical S88 contact can be attributed under the canonical
         * half of a directed sensor pair while the active projection uses
         * its reverse half.  The flash stays on the reported contact, but
         * route progress must advance through the paired waypoint.
         */
        Tc2LiveUiInitialize(&ui, 7000);
        tc2_dispatch_projection_header pair_header;
        tc2_dispatch_projection_waypoint pair_waypoints[3];
        memset(&pair_header, 0, sizeof(pair_header));
        memset(pair_waypoints, 0, sizeof(pair_waypoints));
        pair_header.valid = 1;
        pair_header.train = 14;
        pair_header.plan_generation = 20;
        pair_header.launch_epoch = 20;
        pair_header.start_index = 0;
        pair_header.destination_index = 0;
        pair_header.speed = 80;
        pair_header.waypoint_count = 3;
        pair_waypoints[0].kind = TC2_ROUTE_WAYPOINT_START;
        pair_waypoints[0].sensor_index = -1;
        pair_waypoints[0].ui_row = start->row;
        pair_waypoints[0].ui_column = start->column;
        pair_waypoints[0].ui_width = 1;
        pair_waypoints[1].kind = TC2_ROUTE_WAYPOINT_SENSOR;
        pair_waypoints[1].sensor_index = 1;
        pair_waypoints[1].distance_um = 500000;
        pair_waypoints[1].ui_row = a2.row;
        pair_waypoints[1].ui_column = a2.column;
        pair_waypoints[1].ui_width = a2.width;
        pair_waypoints[2].kind =
                TC2_ROUTE_WAYPOINT_DESTINATION;
        pair_waypoints[2].sensor_index = -1;
        pair_waypoints[2].destination_index = 0;
        pair_waypoints[2].distance_um = 1500000;
        pair_waypoints[2].ui_row = destination->row;
        pair_waypoints[2].ui_column = destination->column;
        pair_waypoints[2].ui_width = 1;
        if (Tc2LiveUiAcceptProjection(
                    &ui, &pair_header,
                    pair_waypoints, 3, 7000) < 0) {
                return fail("paired-sensor projection was not accepted");
        }
        memset(&dispatch, 0, sizeof(dispatch));
        dispatch.jobs[0].train = 14;
        dispatch.jobs[0].state = TC2_JOB_TRAFFIC_HOLD;
        dispatch.jobs[0].plan_generation = 20;
        dispatch.jobs[0].start_index = pair_header.start_index;
        dispatch.jobs[0].destination_index =
                pair_header.destination_index;
        dispatch.jobs[0].speed = pair_header.speed;
        dispatch.jobs[0].traffic_hold_active = 1;
        dispatch.jobs[0].traffic_hold_distance_um = 500000;
        memset(&sensors, 0, sizeof(sensors));
        sensors.last_attributed_train_by_sensor[0] = 14;
        sensors.last_attributed_generation_by_sensor[0] = 20;
        sensors.last_attributed_sequence_by_sensor[0] = 77;
        if (Tc2LiveUiSync(
                    &ui, &dispatch, &sensors, 7001) < 0 ||
            ui.trains[0].confirmed_sensor_distance_um != 500000 ||
            ui.trains[0].displayed_distance_um != 500000 ||
            !(train = Tc2UiOverlayFindTrain(
                    &ui.overlay, 14)) ||
            train->row != a2.row ||
            train->column != a2.column ||
            !(flash = Tc2UiOverlaySensor(
                    &ui.overlay, 0)) ||
            !flash->active || flash->sequence != 77) {
                return fail(
                        "physical sensor pair did not advance the directed "
                        "UI route");
        }

        /*
         * CURRENT reverse-first routes use the opposite directed half of
         * every physical detector.  A11 -> C7 is the physical reverse of
         * the catalogued C8 -> A12 segment and must walk the same drawn rail
         * instead of freezing '@' at A11.
         */
        tc2_dispatch_projection_waypoint reverse_first;
        tc2_dispatch_projection_waypoint reverse_second;
        tc2_track_d_directed_sensor_cell reverse_first_cell;
        tc2_track_d_directed_sensor_cell reverse_second_cell;
        memset(&reverse_first, 0, sizeof(reverse_first));
        memset(&reverse_second, 0, sizeof(reverse_second));
        if (Tc2TrackDDirectedSensorCell(
                    10, &reverse_first_cell) < 0 ||
            Tc2TrackDDirectedSensorCell(
                    38, &reverse_second_cell) < 0) {
                return fail("reverse CURRENT sensor layout is missing");
        }
        reverse_first.kind = TC2_ROUTE_WAYPOINT_SENSOR;
        reverse_first.sensor_index = 10;
        reverse_first.ui_row = reverse_first_cell.row;
        reverse_first.ui_column = reverse_first_cell.column;
        reverse_first.ui_width = reverse_first_cell.width;
        reverse_second.kind = TC2_ROUTE_WAYPOINT_SENSOR;
        reverse_second.sensor_index = 38;
        reverse_second.ui_row = reverse_second_cell.row;
        reverse_second.ui_column = reverse_second_cell.column;
        reverse_second.ui_width = reverse_second_cell.width;
        int reverse_row = -1;
        int reverse_column = -1;
        if (!Tc2LiveUiInterpolateTrackCell(
                    &reverse_first, &reverse_second,
                    500000, 1000000,
                    &reverse_row, &reverse_column) ||
            !Tc2TrackDLayoutCellIsOccupied(
                    reverse_row, reverse_column) ||
            (reverse_row == reverse_first.ui_row &&
             reverse_column == reverse_first.ui_column) ||
            (reverse_row == reverse_second.ui_row &&
             reverse_column == reverse_second.ui_column)) {
                return fail(
                        "reverse CURRENT segment did not animate on rail");
        }

        printf("tc2_live_ui_test: PASS "
               "(predicted motion, physical sensor correction/flash, "
               "full C-to-d1 rail cells, no stale-event jump, fixed T14 "
               "red, traffic hold/resume, atomic CURRENT continuity/race "
               "recovery, simulated arrival)\n");
        return 0;
}
