#pragma once

#include "../interfaces/I_MMU_Hardware.h"
#include "MMU_Defs.h"
#include "UnitState.h"
#include <stdint.h>

class I_MMU_Hardware;

// --- Internal Configuration Constants ---
// (Could be moved to a config file)
#define STRUCT_VERSION 16
#define MOTOR_SPEED_SEND 1500
#define MOTOR_SPEED_AMS_LITE_SEND 1000
#define MOTOR_SPEED_SLOW_SEND 800
#define MOTOR_SPEED_PULL 2000

// --- PID Helper Class ---
class MOTOR_PID {
  float P = 0, I = 0, D = 0, I_save = 0, E_last = 0;
  float pid_MAX = 1000, pid_MIN = -1000, pid_range = 1000;

public:
  MOTOR_PID(float P_set, float I_set, float D_set) {
    Init(P_set, I_set, D_set);
  }
  MOTOR_PID() {}

  void Init(float P_set, float I_set, float D_set) {
    P = P_set;
    I = I_set;
    D = D_set;
    I_save = 0;
    E_last = 0;
  }
  void Clear() {
    I_save = 0;
    E_last = 0;
  }

  float Calculate(float E, float time_E) {
    I_save += I * E * time_E;
    if (I_save > pid_range)
      I_save = pid_range;
    else if (I_save < -pid_range)
      I_save = -pid_range;

    float output;
    if (time_E != 0)
      output = P * E + I_save + D * (E - E_last) / time_E;
    else
      output = P * E + I_save;

    if (output > pid_MAX)
      output = pid_MAX;
    if (output < pid_MIN)
      output = pid_MIN;
    E_last = E;
    return output;
  }
};

enum class LaneMotionState {
  stop,
  send,
  pull,
  slow_send,
  pressure_ctrl_idle,
  pressure_ctrl_in_use,
  pressure_ctrl_on_use,
  velocity_control,
  feed_to_extruder
};

struct FeedToExtruderState {
  bool active = false;
  int lane = -1;
  int cmd_id = 0;
  float speed = 0.0f;
  float max_mm = 0.0f;
  float pressure_threshold = 0.0f;
  uint32_t stall_ms = 300;
  uint64_t start_ms = 0;
  bool done = false;
  enum class StopReason { none, stall, pressure, distance, timeout } reason =
      StopReason::none;
  float dist_moved = 0.0f;
};

enum class pressure_control_enum { less_pressure, all, over_pressure, reverse };

// --- Motor Channel Class ---
class MotorChannel {
public:
  int CHx;
  LaneMotionState motion = LaneMotionState::stop;
  uint64_t motor_stop_time = 0;
  pressure_control_enum pressure_ctrl = pressure_control_enum::all;
  MOTOR_PID PID_speed;
  MOTOR_PID PID_pressure;
  float pwm_zero = 500;
  float dir = 0;
  float target_velocity = 0;
  float target_distance = 0;
  float accumulated_distance = 0;
  float current_velocity_set = 0;
  uint32_t stall_timer = 0;
  bool SET_AUTO_FEED = false;
  uint64_t boost_end_time = 0;
  bool pull_state_old = false;
  bool use_overflow = false;

  MotorChannel() : CHx(0) {} // Default
  MotorChannel(int ch) : CHx(ch) {
    PID_speed.Init(2, 20, 0);
    PID_pressure.Init(1500, 0, 0);
  }

  void Init(int ch) {
    CHx = ch;
    PID_speed.Init(1.0f, 5.0f, 0);
    PID_pressure.Init(1500, 0, 0);
  }

  void SetMotion(LaneMotionState m) {
    if (motion != m) {
      motion = m;
      PID_speed.Clear();
      accumulated_distance = 0;
      current_velocity_set = 0;
      stall_timer = 0;
    }
  }

  // Logic specific calculation - needs access to sensor data?
  // In ControlLogic, GetXByPressure accessed global MC_PULL_stu_raw.
  // If we move this to MMU_Logic, we should pass the sensor value in.
  float CalculatePressureOutput(float current_pressure, float control_voltage,
                                float time_E,
                                pressure_control_enum control_type, float sign,
                                float gain, float min_pwm);

  // Main Run requires logic context (sensors).
  // We will separate logic: MMU_Logic updates Motors.
  // So MotorChannel is just state + PID. The "Run" logic should belong to
  // MMU_Logic or passed dependencies. For simplicity, we'll keep simple state
  // here.
};

enum LanePositionState {
  filament_idle,
  filament_sending_out,
  filament_using,
  filament_pulling_back,
  filament_redetect,
  filament_loading,
  filament_unloading,
};

struct alignas(4) flash_save_struct {
  uint32_t version = STRUCT_VERSION;
  uint32_t check = 0x40614061;
  float pressure_zero[4];
  float pressure_tolerance;
  float pressure_gain;
  float pressure_min_pwm;
  float pressure_offset;
  float boost_threshold;
  float boost_pwm;
  uint32_t boost_time_ms;
  float retract_deadzone;
  float move_p;
  float move_i;
  float move_d;
  float move_pwm_zero;
};

struct Motion_control_save_struct {
  uint32_t check;
  int Motion_control_dir[4];
  uint8_t padding[64];
};

struct CalibrateResult {
  bool ok;
  float value;
  const char *error_msg;
  bool flash_ok = true;
};

// --- MMU Logic Class ---
class MMU_Logic {
public:
  MMU_Logic(I_MMU_Hardware *hal);

  void Init();
  void Run();

  // Connectivity
  void UpdateConnectivity(bool online);

  // Actions
  void StartLoadFilament(int tray, int length_mm = -1);
  void StartUnloadFilament(int tray, int length_mm = -1);
  void SetAutoFeed(int lane, bool enable, bool overflow = false);
  void SetActiveLaneIndex(int index);
  CalibrateResult CalibratePressure(int lane);

  // FTE Command
  using StopReason = FeedToExtruderState::StopReason;
  void StartFeedToExtruder(int lane, float speed, float max_mm,
                           float pressure_threshold, uint32_t stall_ms, int cmd_id);
  StopReason GetFTEResult(float &dist_moved_mm);
  void ClearFTEResult();
  int GetFTECmdId() { return _fte.cmd_id; }

  // Klipper Primitives
  void MoveAxis(int axis, float dist_mm, float speed);
  void StopAll();

  uint16_t GetSensorState();
  int GetLaneMotion(int lane);
  bool GetLaneAutoFeed(int lane) { return (lane >= 0 && lane < 4) ? motors[lane].SET_AUTO_FEED : false; }
  bool GetLaneOverflow(int lane) { return (lane >= 0 && lane < 4) ? motors[lane].use_overflow : false; }

  // Diagnostic Tools
  void DiagnosticMotorControl(int lane, int pwm, uint32_t duration_ms);
  bool IsDiagnosticActive(int lane) {
    return (lane >= 0 && lane < 4) ? diag_active[lane] : false;
  }
  // Accessors (Replacement for UnitState)
  LaneState &GetLane(int index);
  int GetActiveLaneIndex();
  float GetPressureZero(int lane);
  float GetPressureTolerance() { return data_save.pressure_tolerance; }
  void SetPressureTolerance(float tol);
  void SetPressureGain(float gain);
  void SetPressureMinPWM(float pwm);
  void SetPressureOffset(float offset);
  void SetBoostThreshold(float threshold);
  void SetBoostPWM(float pwm);
  void SetBoostTime(uint32_t ms);
  void SetRetractDeadzone(float deadzone);

  float GetPressureGain() { return data_save.pressure_gain; }
  float GetPressureMinPWM() { return data_save.pressure_min_pwm; }
  float GetMoveP() { return data_save.move_p; }
  float GetMoveI() { return data_save.move_i; }
  float GetMoveD() { return data_save.move_d; }
  float GetMovePwmZero() { return data_save.move_pwm_zero; }
  float GetPressureOffset() { return data_save.pressure_offset; }
  float GetBoostThreshold() { return data_save.boost_threshold; }
  float GetBoostPWM() { return data_save.boost_pwm; }
  uint32_t GetBoostTime() { return data_save.boost_time_ms; }
  float GetRetractDeadzone() { return data_save.retract_deadzone; }

  // Persistence
  void SaveSettings();
  void LoadSettings();
  void SetNeedToSave();
  void SyncMovePID();
  void SetMovePID(float p, float i, float d, float zero);

private:
  I_MMU_Hardware *_hal;

  // State
  flash_save_struct data_save;
  Motion_control_save_struct mc_save;
  MotorChannel motors[4];
  FeedToExtruderState _fte;

  LaneState lanes[4];
  int active_lane;


  LanePositionState filament_now_position[4];

  // Sensor Cache
  float speed_as5600[4];
  float MC_PULL_stu_raw[4];
  int MC_PULL_stu[4];
  int MC_ONLINE_key_stu[4];
  float pressure_filtered[4];
  float as5600_delta_mm[4];

  bool Assist_send_filament[4];
  bool is_backing_out;
  float last_total_distance[4];
  int32_t as5600_distance_save[4];

  int32_t unload_target_dist[4];
  float unload_start_meters[4];

  bool Bambubus_need_to_save;
  uint64_t save_timer;

  bool is_connected;
  uint64_t last_heartbeat_time;
  uint16_t device_type_addr;

  // Diagnostic State
  int diag_pwm[4];
  uint64_t diag_end_time[4];
  bool diag_active[4];

  // Motion timing state (replaces static locals in methods)
  uint64_t _motion_switch_time_end = 0;
  uint64_t _last_run_us = 0;
  uint64_t _last_led_update = 0;
  bool _led_toggle = false;

  // Constants
  const bool is_two = true; // AMS Lite logic
  const float PULL_voltage_up = 1.85f;
  const float PULL_voltage_down = 1.45f;
  const uint32_t use_flash_addr = 0x0800F000;
  const uint32_t Motion_control_save_flash_addr = 0x0800E000;

  // Internal Methods
  void motor_motion_switch();
  void MC_PULL_ONLINE_read();
  void AS5600_Update(float time_E);
  bool Prepare_For_filament_Pull_Back(float OUT_filament_meters);
  void UpdateLEDStatus(int channel);
  void RunMotorChannel(int channel, float time_E);

  // Helper
  uint64_t get_time64() { return _hal->GetTimeMS(); }
};
