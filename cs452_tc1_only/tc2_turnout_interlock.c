#include "tc2_turnout_interlock.h"

static int tick_reached_local(int now, int deadline) {
        return deadline >= 0 &&
                (unsigned int)(now - deadline) < 0x80000000u;
}

int Tc2TurnoutInterlockResourceForSwitch(int switch_number) {
        if (switch_number >= 1 && switch_number <= 18) {
                return switch_number;
        }
        if (switch_number == 153 || switch_number == 154) {
                return 19;
        }
        if (switch_number == 155 || switch_number == 156) {
                return 20;
        }
        return -1;
}

static tc2_turnout_interlock_resource *resource_mutable(
        tc2_turnout_interlock_table *table, int switch_number) {
        int resource =
                Tc2TurnoutInterlockResourceForSwitch(switch_number);
        if (!table || resource < 1 ||
            resource > TC2_TURNOUT_INTERLOCK_MAX_RESOURCES) {
                return 0;
        }
        return &table->resources[resource - 1];
}

static const tc2_turnout_interlock_resource *resource_const(
        const tc2_turnout_interlock_table *table,
        int switch_number) {
        int resource =
                Tc2TurnoutInterlockResourceForSwitch(switch_number);
        if (!table || resource < 1 ||
            resource > TC2_TURNOUT_INTERLOCK_MAX_RESOURCES) {
                return 0;
        }
        return &table->resources[resource - 1];
}

void Tc2TurnoutInterlockInit(tc2_turnout_interlock_table *table) {
        if (!table) return;
        table->next_sequence = 0;
        for (int index = 0;
             index < TC2_TURNOUT_INTERLOCK_MAX_RESOURCES;
             ++index) {
                tc2_turnout_interlock_resource *resource =
                        &table->resources[index];
                resource->resource = index + 1;
                resource->owner_train = 0;
                resource->owner_direction = -1;
                resource->state = TC2_TURNOUT_INTERLOCK_FREE;
                resource->release_at_tick = -1;
                resource->waiter_count = 0;
                for (int waiter = 0;
                     waiter < TC2_TURNOUT_INTERLOCK_MAX_WAITERS;
                     ++waiter) {
                        resource->waiters[waiter].train = 0;
                        resource->waiters[waiter].direction = -1;
                        resource->waiters[waiter].sequence = 0;
                }
        }
}

int Tc2TurnoutInterlockEnqueue(
        tc2_turnout_interlock_table *table, int switch_number,
        int train, int direction) {
        tc2_turnout_interlock_resource *resource =
                resource_mutable(table, switch_number);
        if (!resource || train < 1 || train > 255 ||
            (direction != 0 && direction != 1)) {
                return -1;
        }
        if (resource->owner_train == train) {
                resource->owner_direction = direction;
                return 0;
        }
        for (int index = 0;
             index < resource->waiter_count; ++index) {
                if (resource->waiters[index].train == train) {
                        resource->waiters[index].direction =
                                direction;
                        return 0;
                }
        }
        if (resource->waiter_count >=
            TC2_TURNOUT_INTERLOCK_MAX_WAITERS) {
                return -1;
        }
        unsigned int sequence = ++table->next_sequence;
        if (sequence == 0) sequence = ++table->next_sequence;
        tc2_turnout_interlock_waiter *waiter =
                &resource->waiters[resource->waiter_count++];
        waiter->train = train;
        waiter->direction = direction;
        waiter->sequence = sequence;
        return 0;
}

int Tc2TurnoutInterlockGrantNext(
        tc2_turnout_interlock_table *table, int switch_number,
        int *direction) {
        tc2_turnout_interlock_resource *resource =
                resource_mutable(table, switch_number);
        if (!resource || !direction) return -1;
        if (resource->state != TC2_TURNOUT_INTERLOCK_FREE ||
            resource->owner_train != 0) {
                return resource->owner_train;
        }
        if (resource->waiter_count == 0) return 0;
        int train = resource->waiters[0].train;
        int granted_direction =
                resource->waiters[0].direction;
        for (int index = 1;
             index < resource->waiter_count; ++index) {
                resource->waiters[index - 1] =
                        resource->waiters[index];
        }
        --resource->waiter_count;
        resource->owner_train = train;
        resource->owner_direction = granted_direction;
        *direction = granted_direction;
        resource->state = TC2_TURNOUT_INTERLOCK_GRANTED;
        resource->release_at_tick = -1;
        return train;
}

static int mark_owner_state(
        tc2_turnout_interlock_table *table, int switch_number,
        int train, int expected, int next) {
        tc2_turnout_interlock_resource *resource =
                resource_mutable(table, switch_number);
        if (!resource || resource->owner_train != train ||
            resource->state != expected) {
                return -1;
        }
        resource->state = next;
        return 0;
}

int Tc2TurnoutInterlockMarkSetting(
        tc2_turnout_interlock_table *table, int switch_number,
        int train) {
        return mark_owner_state(
                table, switch_number, train,
                TC2_TURNOUT_INTERLOCK_GRANTED,
                TC2_TURNOUT_INTERLOCK_SETTING);
}

int Tc2TurnoutInterlockMarkCrossing(
        tc2_turnout_interlock_table *table, int switch_number,
        int train) {
        return mark_owner_state(
                table, switch_number, train,
                TC2_TURNOUT_INTERLOCK_SETTING,
                TC2_TURNOUT_INTERLOCK_CROSSING);
}

int Tc2TurnoutInterlockMarkTailClear(
        tc2_turnout_interlock_table *table, int switch_number,
        int train, int now, int release_delay_ticks) {
        tc2_turnout_interlock_resource *resource =
                resource_mutable(table, switch_number);
        if (!resource || resource->owner_train != train ||
            resource->state != TC2_TURNOUT_INTERLOCK_CROSSING ||
            release_delay_ticks < 0) {
                return -1;
        }
        resource->state =
                TC2_TURNOUT_INTERLOCK_RELEASE_DELAY;
        resource->release_at_tick =
                now + release_delay_ticks;
        return 0;
}

int Tc2TurnoutInterlockTick(
        tc2_turnout_interlock_table *table, int now) {
        if (!table) return -1;
        int released = 0;
        for (int index = 0;
             index < TC2_TURNOUT_INTERLOCK_MAX_RESOURCES;
             ++index) {
                tc2_turnout_interlock_resource *resource =
                        &table->resources[index];
                if (resource->state !=
                            TC2_TURNOUT_INTERLOCK_RELEASE_DELAY ||
                    !tick_reached_local(
                            now, resource->release_at_tick)) {
                        continue;
                }
                resource->owner_train = 0;
                resource->owner_direction = -1;
                resource->state =
                        TC2_TURNOUT_INTERLOCK_FREE;
                resource->release_at_tick = -1;
                ++released;
        }
        return released;
}

int Tc2TurnoutInterlockOwner(
        const tc2_turnout_interlock_table *table,
        int switch_number) {
        const tc2_turnout_interlock_resource *resource =
                resource_const(table, switch_number);
        return resource ? resource->owner_train : -1;
}

int Tc2TurnoutInterlockState(
        const tc2_turnout_interlock_table *table,
        int switch_number) {
        const tc2_turnout_interlock_resource *resource =
                resource_const(table, switch_number);
        return resource ? resource->state : -1;
}

int Tc2TurnoutInterlockIsQueued(
        const tc2_turnout_interlock_table *table,
        int switch_number, int train) {
        const tc2_turnout_interlock_resource *resource =
                resource_const(table, switch_number);
        if (!resource || train < 1 || train > 255) return 0;
        for (int index = 0;
             index < resource->waiter_count; ++index) {
                if (resource->waiters[index].train == train) {
                        return 1;
                }
        }
        return 0;
}

int Tc2TurnoutInterlockRemoveWaiter(
        tc2_turnout_interlock_table *table, int switch_number,
        int train) {
        tc2_turnout_interlock_resource *resource =
                resource_mutable(table, switch_number);
        if (!resource || train < 1 || train > 255) return -1;
        int removed = 0;
        int write = 0;
        for (int read = 0;
             read < resource->waiter_count; ++read) {
                if (resource->waiters[read].train == train) {
                        removed = 1;
                        continue;
                }
                if (write != read) {
                        resource->waiters[write] =
                                resource->waiters[read];
                }
                ++write;
        }
        resource->waiter_count = write;
        return removed;
}

void Tc2TurnoutInterlockRemoveTrain(
        tc2_turnout_interlock_table *table, int train,
        int release_owner) {
        if (!table || train < 1 || train > 255) return;
        for (int resource_index = 0;
             resource_index <
                     TC2_TURNOUT_INTERLOCK_MAX_RESOURCES;
             ++resource_index) {
                tc2_turnout_interlock_resource *resource =
                        &table->resources[resource_index];
                int write = 0;
                for (int read = 0;
                     read < resource->waiter_count; ++read) {
                        if (resource->waiters[read].train == train) {
                                continue;
                        }
                        if (write != read) {
                                resource->waiters[write] =
                                        resource->waiters[read];
                        }
                        ++write;
                }
                resource->waiter_count = write;
                if (release_owner &&
                    resource->owner_train == train) {
                        resource->owner_train = 0;
                        resource->owner_direction = -1;
                        resource->state =
                                TC2_TURNOUT_INTERLOCK_FREE;
                        resource->release_at_tick = -1;
                }
        }
}
