#include <assert.h>
#include <stdio.h>

#include "../tc2_turnout_interlock.h"

int main(void) {
        tc2_turnout_interlock_table table;
        Tc2TurnoutInterlockInit(&table);

        assert(Tc2TurnoutInterlockResourceForSwitch(5) == 5);
        assert(Tc2TurnoutInterlockResourceForSwitch(153) == 19);
        assert(Tc2TurnoutInterlockResourceForSwitch(154) == 19);
        assert(Tc2TurnoutInterlockResourceForSwitch(155) == 20);
        assert(Tc2TurnoutInterlockResourceForSwitch(156) == 20);

        assert(Tc2TurnoutInterlockEnqueue(
                       &table, 5, 14, 1) == 0);
        assert(Tc2TurnoutInterlockEnqueue(
                       &table, 5, 13, 0) == 0);
        assert(Tc2TurnoutInterlockEnqueue(
                       &table, 5, 14, 1) == 0);
        assert(Tc2TurnoutInterlockIsQueued(
                       &table, 5, 14));
        int direction = 1;
        assert(Tc2TurnoutInterlockGrantNext(
                       &table, 5, &direction) == 14);
        assert(Tc2TurnoutInterlockOwner(&table, 5) == 14);
        assert(Tc2TurnoutInterlockMarkSetting(
                       &table, 5, 14) == 0);
        assert(Tc2TurnoutInterlockMarkCrossing(
                       &table, 5, 14) == 0);
        assert(Tc2TurnoutInterlockMarkTailClear(
                       &table, 5, 14, 100, 300) == 0);
        assert(Tc2TurnoutInterlockTick(&table, 399) == 0);
        assert(Tc2TurnoutInterlockOwner(&table, 5) == 14);
        assert(Tc2TurnoutInterlockTick(&table, 400) == 1);
        assert(Tc2TurnoutInterlockOwner(&table, 5) == 0);

        direction = 0;
        assert(Tc2TurnoutInterlockGrantNext(
                       &table, 5, &direction) == 13);
        assert(Tc2TurnoutInterlockOwner(&table, 5) == 13);

        /*
         * A committed CURRENT handoff may retire only its old FIFO ticket.
         * It must neither release the physical owner nor disturb peers.
         */
        assert(Tc2TurnoutInterlockEnqueue(
                       &table, 6, 14, 1) == 0);
        assert(Tc2TurnoutInterlockEnqueue(
                       &table, 6, 15, 0) == 0);
        direction = 0;
        assert(Tc2TurnoutInterlockGrantNext(
                       &table, 6, &direction) == 14);
        assert(Tc2TurnoutInterlockOwner(&table, 6) == 14);
        assert(Tc2TurnoutInterlockRemoveWaiter(
                       &table, 6, 15) == 1);
        assert(!Tc2TurnoutInterlockIsQueued(
                       &table, 6, 15));
        assert(Tc2TurnoutInterlockOwner(&table, 6) == 14);
        assert(Tc2TurnoutInterlockRemoveWaiter(
                       &table, 6, 15) == 0);
        assert(Tc2TurnoutInterlockRemoveWaiter(
                       &table, 6, 14) == 0);
        assert(Tc2TurnoutInterlockOwner(&table, 6) == 14);

        assert(Tc2TurnoutInterlockEnqueue(
                       &table, 153, 15, 1) == 0);
        assert(Tc2TurnoutInterlockIsQueued(
                       &table, 154, 15));
        Tc2TurnoutInterlockRemoveTrain(&table, 15, 0);
        assert(!Tc2TurnoutInterlockIsQueued(
                       &table, 153, 15));

        Tc2TurnoutInterlockRemoveTrain(&table, 13, 1);
        assert(Tc2TurnoutInterlockOwner(&table, 5) == 0);

        puts("tc2 turnout interlock tests passed");
        return 0;
}
