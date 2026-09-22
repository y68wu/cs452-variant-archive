#ifndef _tc2_turnout_interlock_h_
#define _tc2_turnout_interlock_h_ 1

#define TC2_TURNOUT_INTERLOCK_MAX_RESOURCES 20
#define TC2_TURNOUT_INTERLOCK_MAX_WAITERS 16

typedef enum {
        TC2_TURNOUT_INTERLOCK_FREE = 0,
        TC2_TURNOUT_INTERLOCK_GRANTED,
        TC2_TURNOUT_INTERLOCK_SETTING,
        TC2_TURNOUT_INTERLOCK_CROSSING,
        TC2_TURNOUT_INTERLOCK_RELEASE_DELAY
} tc2_turnout_interlock_state;

typedef struct {
        int train;
        int direction;
        unsigned int sequence;
} tc2_turnout_interlock_waiter;

typedef struct {
        int resource;
        int owner_train;
        int owner_direction;
        int state;
        int release_at_tick;
        tc2_turnout_interlock_waiter
                waiters[TC2_TURNOUT_INTERLOCK_MAX_WAITERS];
        int waiter_count;
} tc2_turnout_interlock_resource;

typedef struct {
        tc2_turnout_interlock_resource
                resources[TC2_TURNOUT_INTERLOCK_MAX_RESOURCES];
        unsigned int next_sequence;
} tc2_turnout_interlock_table;

/*
 * The two middle double-turnout assemblies are physical resources, not four
 * independent switches.  Every other turnout number maps to itself.
 */
int Tc2TurnoutInterlockResourceForSwitch(int switch_number);

void Tc2TurnoutInterlockInit(tc2_turnout_interlock_table *table);

/*
 * Queue exactly once.  FIFO order is global arrival order at this resource.
 * A train may update its requested direction without losing its queue place.
 */
int Tc2TurnoutInterlockEnqueue(
        tc2_turnout_interlock_table *table, int switch_number,
        int train, int direction);

/*
 * Grant the oldest waiter when the resource is free.  Returns the owner train,
 * zero when no waiter is ready, or -1 for invalid input.  `direction` receives
 * the granted waiter's requested S/C direction.
 */
int Tc2TurnoutInterlockGrantNext(
        tc2_turnout_interlock_table *table, int switch_number,
        int *direction);

int Tc2TurnoutInterlockMarkSetting(
        tc2_turnout_interlock_table *table, int switch_number,
        int train);
int Tc2TurnoutInterlockMarkCrossing(
        tc2_turnout_interlock_table *table, int switch_number,
        int train);
int Tc2TurnoutInterlockMarkTailClear(
        tc2_turnout_interlock_table *table, int switch_number,
        int train, int now, int release_delay_ticks);

/*
 * Finish every expired three-second release delay.  Returns the number of
 * resources made free.
 */
int Tc2TurnoutInterlockTick(
        tc2_turnout_interlock_table *table, int now);

int Tc2TurnoutInterlockOwner(
        const tc2_turnout_interlock_table *table,
        int switch_number);
int Tc2TurnoutInterlockState(
        const tc2_turnout_interlock_table *table,
        int switch_number);
int Tc2TurnoutInterlockIsQueued(
        const tc2_turnout_interlock_table *table,
        int switch_number, int train);

/*
 * Drop one pre-owner FIFO ticket without touching any current owner.  This
 * is used after an atomic CURRENT route replacement commits: the old runtime
 * no longer has a state machine which could consume its former ticket, while
 * every rollback path must leave that ticket byte-for-byte intact.
 *
 * Returns one when a waiter was removed, zero when it was not queued, or -1
 * for invalid input.
 */
int Tc2TurnoutInterlockRemoveWaiter(
        tc2_turnout_interlock_table *table, int switch_number,
        int train);

/*
 * Removing a train drops its queued requests.  Ownership is released only
 * when release_owner is non-zero; callers use that only after physical
 * removal, never for an ordinary cancel while the train remains on track.
 */
void Tc2TurnoutInterlockRemoveTrain(
        tc2_turnout_interlock_table *table, int train,
        int release_owner);

#endif
