#pragma once

#include <stdint.h>

// Unit Info shared across modules
#define DEVICE_MODEL "BMCU370"
#define DEVICE_VERSION "00.00.09.00"
#define DEVICE_SERIAL "00000000000000"

// Lane Presence Status
enum class LanePresenceStatus
{
    offline,
    online,
    NFC_waiting
};

// Lane Motion State
enum class LaneMotionSet
{
    before_pull_back, // 0
    need_pull_back,   // 1
    need_send_out,    // 2
    in_use,           // 3
    idle              // 4
};
