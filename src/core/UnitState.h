#pragma once

#include <stdint.h>
#include "MMU_Defs.h"



// --- Shared Data Structures ---

struct LaneState
{
    float meters = 0;
    uint16_t pressure = 0;
    LanePresenceStatus status = LanePresenceStatus::offline;
    LaneMotionSet motion_set = LaneMotionSet::idle;
};

// UnitState class removed. Use MMU_Logic accessors instead.
