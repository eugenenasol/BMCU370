# BMCU370 Code Review Fixes — Implementation Plan

> **For Hermes:** Use subagent-driven-development skill. All work in branch `review`. Each task = one sub-session.

**Goal:** Fix 7 issues found in full code review (2 critical, 3 medium, 2 cleanup).

**Architecture:** Targeted fixes in MMU_Logic, BMCU_Hardware, KlipperCLI. No refactoring — minimal changes to fix specific bugs.

**Branch:** `review` (created from `main`)

---

### Task 1: 🔴 Fix Watchdog Reset (BMCU_Hardware)

**Objective:** Implement actual IWDG reload in `WatchdogReset()`

**Files:**
- Modify: `src/hal/BMCU_Hardware.cpp:43-45`

**Code change:**
```cpp
// Replace the empty stub:
void BMCU_Hardware::WatchdogReset() {
    // Hardware::Watchdog_Reset(); // If implemented
}

// With:
void BMCU_Hardware::WatchdogReset() {
    IWDG->CTLR = 0xAAAA; // Reload Independent Watchdog
}
```

**Verification:** Compile: `cd /root/BMCU370 && pio run 2>&1 | tail -5`. Expect: SUCCESS.

**Commit:** `fix: implement watchdog reset in BMCU_Hardware`

---

### Task 2: 🔴 Fix LED Show Chain (MMU_Logic / BMCU_Hardware)

**Objective:** Ensure LED_Show is called after SetLED to push pixel buffer to physical strip

**Files:**
- Modify: `src/core/MMU_Logic.cpp` — remove dev-commentary TODOs on lines 741-762
- Verify: `src/hal/BMCU_Hardware.cpp` — confirm LED_Show() delegates to Hardware::LED_Show() correctly (line 120-122)

**Code change in MMU_Logic.cpp:**
Replace the large comment block (lines 741-762) with a single clean comment:
```cpp
// LED_Show() called at end of Run() — pushes buffered pixels to strip
```

**Verification:** Confirm `BMCU_Hardware::LED_Show()` calls `Hardware::LED_Show()` → `pixels.show()`. Read `src/hal/Hardware.cpp` and find `LED_Show` implementation.

**Commit:** `fix: clean up LED show chain comments and verify hardware path`

---

### Task 3: 🟡 Fix Encoder Drift in meters Calculation

**Objective:** Only accumulate filament meters when motor is actually moving

**Files:**
- Modify: `src/core/MMU_Logic.cpp:236`

**Current code:**
```cpp
data_save.filament[i].meters += dist_E / 1000.0f;
```

**New code — wrap in motion check:**
```cpp
// Only accumulate meters when filament is actively moving (not idling/pressure-control)
if (motors[i].motion != filament_motion_enum::stop &&
    motors[i].motion != filament_motion_enum::pressure_ctrl_idle &&
    motors[i].motion != filament_motion_enum::pressure_ctrl_in_use) {
    data_save.filament[i].meters += dist_E / 1000.0f;
}
```

**Verification:** Compile and check no regressions: `cd /root/BMCU370 && pio run`

**Commit:** `fix: prevent encoder noise from accumulating in filament meters during idle`

---

### Task 4: 🟡 Escape JSON Special Characters in Filament Names

**Objective:** Escape `"` and `\` in filament names/IDs before embedding in hand-built JSON

**Files:**
- Modify: `src/api/KlipperCLI.cpp` — add escaping in `HandleStatus` and `HandleGetFilamentInfo`

**Approach:** Add a small helper that replaces `"` → `\"` and `\` → `\\`, OR better: port the STATUS response to use LiteJSON serialization instead of hand-built `snprintf`.

**Decision:** The safer fix for now is to add a sanitization pass that replaces `"` with `'` in filament strings (since filament names don't legitimately contain double quotes). This avoids the risky full refactor to LiteJSON serialization.

**Code change — in `HandleStatus` (after line 138), add:**
```cpp
// Escape JSON-breaking characters in filament strings
for(int j=0; j<20 && safe_name[j]; j++) {
    if(safe_name[j] == '"' || safe_name[j] == '\\') safe_name[j] = '\'';
}
for(int j=0; j<8 && safe_id[j]; j++) {
    if(safe_id[j] == '"' || safe_id[j] == '\\') safe_id[j] = '\'';
}
```

Same fix in `HandleGetFilamentInfo` after the sanitization loop (line 362).

**Verification:** Compile and check.

**Commit:** `fix: escape JSON-breaking chars in filament name/id strings`

---

### Task 5: 🟡 Fix GetLanePIDOutput Stub

**Objective:** Either implement the method or remove it from the public API

**Files:**
- Modify: `src/core/MMU_Logic.cpp:1023-1028`
- Possibly modify: `src/core/MMU_Logic.h` — remove declaration if method removed

**Decision:** Remove the method since the last PID output isn't tracked. Check if anything calls it:
```bash
cd /root/BMCU370 && grep -r "GetLanePIDOutput" src/
```
If nothing calls it → remove declaration from `.h` and implementation from `.cpp`.

**Commit:** `fix: remove unimplemented GetLanePIDOutput stub`

---

### Task 6: 🟢 Move Static Locals to Member Variables

**Objective:** Replace `static` locals in member functions with proper member variables

**Files:**
- Modify: `src/core/MMU_Logic.h` — add members
- Modify: `src/core/MMU_Logic.cpp` — remove statics, use members

**Changes:**
1. `static uint64_t time_end = 0` (line 649) → member `uint64_t _motion_switch_time_end`
2. `static uint64_t last_run = 0` (line 690) → member `uint64_t _last_run_us`
3. `static uint64_t last_led_update = 0` + `static bool toggle` (lines 730-732) → members `uint64_t _last_led_update` and `bool _led_toggle`

**Commit:** `refactor: move static locals to member variables in MMU_Logic`

---

### Task 7: 🟢 Replace extern with Proper Include

**Objective:** Replace `extern bool KlipperCLI_IsSerialIdle(uint32_t)` with `#include`

**Files:**
- Modify: `src/core/MMU_Logic.cpp:707`
- Check: `src/api/KlipperCLI.h` has the function declaration

**Code change:**
```cpp
// Remove line 707:
// extern bool KlipperCLI_IsSerialIdle(uint32_t);

// Add at top of file:
#include "../api/KlipperCLI.h"
```

**Verification:** Compile — if KlipperCLI.h doesn't have the declaration, add it.

**Commit:** `refactor: replace extern forward-declaration with proper #include`

---

## Execution Order

1. Task 1 — Watchdog (critical)
2. Task 2 — LED chain (critical)
3. Task 3 — Encoder drift (medium)
4. Task 4 — JSON escaping (medium)
5. Task 5 — GetLanePIDOutput stub (medium)
6. Task 6 — Static→members (cleanup)
7. Task 7 — extern→#include (cleanup)

**After all tasks:** Run `pio run` for final build check, then offer to commit and push.