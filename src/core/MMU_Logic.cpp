#include "MMU_Logic.h"
#include "../hal/Flash_saves.h"
#include "../hal/Hardware.h"

#define AS5600_PI 3.1415926535897932384626433832795

// --- MotorChannel Helper Implementation ---

float MotorChannel::CalculatePressureOutput(float current_pressure,
                                            float control_voltage, float time_E,
                                            pressure_control_enum control_type,
                                            float sign, float gain, float min_pwm) {
  float x = 0;
  switch (control_type) {
  case pressure_control_enum::all:
    x = sign *
        PID_pressure.Calculate(control_voltage - current_pressure, time_E);
    break;
  case pressure_control_enum::less_pressure:
    if (current_pressure < control_voltage)
      x = sign *
          PID_pressure.Calculate(control_voltage - current_pressure, time_E);
    break;
  case pressure_control_enum::over_pressure:
    if (current_pressure > control_voltage)
      x = sign *
          PID_pressure.Calculate(control_voltage - current_pressure, time_E);
    break;
  }
  if (x > 0) {
    x = min_pwm + x;
  } else if (x < 0) {
    x = -min_pwm + x;
  } else {
    x = 0;
  }
  return x;
}

// --- MMU_Logic Implementation ---

MMU_Logic::MMU_Logic(I_MMU_Hardware *hal) : _hal(hal) {
  // Defaults
  device_type_addr = BambuBus_AMS;
  Bambubus_need_to_save = false;
  save_timer = 0;

  // Init Arrays
  for (int i = 0; i < 4; i++) {
    motors[i].Init(i);
    filament_now_position[i] = filament_idle;
    speed_as5600[i] = 0;
    MC_PULL_stu_raw[i] = 0;
    MC_PULL_stu[i] = 0;
    MC_ONLINE_key_stu_raw[i] = 0;
    MC_ONLINE_key_stu[i] = 0;
    Assist_send_filament[i] = false;
    last_total_distance[i] = 0;
    as5600_distance_save[i] = 0;
    unload_target_dist[i] = -1;
    unload_start_meters[i] = 0;
    diag_active[i] = false;
    diag_pwm[i] = 0;
    diag_end_time[i] = 0;
    pressure_filtered[i] = 1.65f;
  }
  is_backing_out = false;
  is_connected = false;
  last_heartbeat_time = 0;

  // Initialize Data Save defaults before Load
  data_save.check = 0; // Invalid
}

void MMU_Logic::Init() {
  _hal->Init();
  LoadSettings();
  // AS5600 Init moved to HAL Init inside _hal->Init()

  // Setup Motor Directions based on Config
  for (int i = 0; i < 4; i++) {
    motors[i].dir = 1.0f; // Standardized: Plus = Forward, Minus = Backward
    motors[i].SetMotion(filament_motion_enum::pressure_ctrl_idle);
    motors[i].PID_pressure.Init(data_save.pressure_gain, 0, 0);

    last_total_distance[i] = data_save.filament[i].meters;
  }
  SyncMovePID();
}

void MMU_Logic::UpdateConnectivity(bool online) {
  is_connected = online;
  if (online)
    last_heartbeat_time = _hal->GetTimeMS();
}

void MMU_Logic::SaveSettings() {
  Flash_saves(&data_save, sizeof(data_save), use_flash_addr);
  Bambubus_need_to_save = false;
}

void MMU_Logic::SetNeedToSave() {
  if (!Bambubus_need_to_save) {
    Bambubus_need_to_save = true;
    save_timer = _hal->GetTimeMS();
  }
}

void MMU_Logic::SyncMovePID() {
  for (int i = 0; i < 4; i++) {
    motors[i].PID_speed.Init(data_save.move_p, data_save.move_i, data_save.move_d);
  }
}

void MMU_Logic::SetMovePID(float p, float i, float d, float zero) {
  if (p >= 0) data_save.move_p = p;
  if (i >= 0) data_save.move_i = i;
  if (d >= 0) data_save.move_d = d;
  if (zero >= 0) data_save.move_pwm_zero = zero;
  SyncMovePID();
  SetNeedToSave();
}

void MMU_Logic::LoadSettings() {
  flash_save_struct *ptr = (flash_save_struct *)(uintptr_t)(use_flash_addr);
  bool need_defaults = true;

  if (ptr->check == 0x40614061) {
    if (ptr->version == data_save.version) {
      __builtin_memcpy(&data_save, ptr, sizeof(data_save));
      need_defaults = false;
    } else if (ptr->version >= 5 && ptr->version <= 9) {
      // Migrate v5-v9 to v10
      size_t copy_size = (ptr->version >= 9) ? sizeof(data_save) - sizeof(float) : sizeof(data_save) - sizeof(float)*3;
      __builtin_memcpy(&data_save, ptr, copy_size);
      
      if (ptr->version < 9) {
        data_save.pressure_gain = 250.0f;
        data_save.pressure_tolerance = 0.03f;
      }
      data_save.pressure_min_pwm = 50.0f;
      data_save.version = 13;
      data_save.pressure_gain = 5000.0f;
      data_save.pressure_offset = -0.07f;
      data_save.boost_threshold = 0.02f;
      data_save.boost_pwm = 350.0f;
      data_save.boost_time_ms = 50;
      data_save.retract_deadzone = 1.2f;
      data_save.pressure_min_pwm = 180.0f;
      data_save.pressure_tolerance = 0.015f;
      data_save.move_p = 25.0f;
      data_save.move_i = 80.0f;
      data_save.move_d = 0.0f;
      data_save.move_pwm_zero = 200.0f;
      data_save.version = 14;
      SetNeedToSave();
      need_defaults = false;
    } else if (ptr->version == 5) {
      __builtin_memcpy(&data_save, ptr,
                       sizeof(data_save) - sizeof(float) * 6); // copy v5
      data_save.version = 9;
      for (int i = 0; i < 4; i++) {
        data_save.pressure_zero[i] = 1.65f;
      }
      data_save.pressure_tolerance = 0.03f;
      data_save.pressure_gain = 250.0f;
      SetNeedToSave();
      need_defaults = false;
    }
  }

  if (need_defaults) {
    // Default constants - set all 4 filaments to PLA defaults
    for (int i = 0; i < 4; i++) {
      data_save.filament[i].SetID(""); // Clear RFID
      data_save.filament[i].SetName("PLA");
      data_save.filament[i].temperature_min = 200;
      data_save.filament[i].temperature_max = 220;
      data_save.filament[i].meters = 0;
      data_save.filament[i].pressure = 0;
      data_save.filament[i].color_R = 0xFF;
      data_save.filament[i].color_G = 0xFF;
      data_save.filament[i].color_B = 0xFF;
      data_save.pressure_zero[i] = 1.65f;
    }
    data_save.pressure_tolerance = 0.015f;
    data_save.pressure_gain = 5000.0f;
    data_save.pressure_offset = -0.07f;
    data_save.boost_threshold = 0.02f;
    data_save.boost_pwm = 350.0f;
    data_save.boost_time_ms = 50;
    data_save.retract_deadzone = 1.2f;
    data_save.pressure_min_pwm = 180.0f;
    data_save.boot_mode = 1; // Default to Klipper
    data_save.move_p = 25.0f;
    data_save.move_i = 80.0f;
    data_save.move_d = 0.0f;
    data_save.move_pwm_zero = 200.0f;
    data_save.version = 14;
    data_save.check = 0x40614061;
    SetNeedToSave();
  }

  Motion_control_save_struct *mc_ptr =
      (Motion_control_save_struct *)(uintptr_t)(Motion_control_save_flash_addr);
  if (mc_ptr->check == 0x40614061) {
    __builtin_memcpy(&mc_save, mc_ptr, sizeof(mc_save));
  }
}

void MMU_Logic::AS5600_Update(float time_E) {
  // HAL provides polling.
  for (int i = 0; i < 4; i++) {
    // bool online = _hal->GetEncoderOnline(i); // HAL doesn't have this.
    // Assume always online or use logic?
    // `GetEncoderValue` returns 0 if error?
    int32_t now = _hal->GetEncoderValue(i);
    // Note: implementation needs to handle wrapping here or in HAL?
    // HAL returns raw 0-4096.
    // Logic handles wrapping.

    int32_t last = as5600_distance_save[i];
    int cir_E = 0;
    if (now > 3072 && last <= 1024)
      cir_E = -4096;
    else if (now <= 1024 && last > 3072)
      cir_E = 4096;

    float dist_E = (float)(now - last + cir_E) * AS5600_PI * 7.5 / 4096;
    as5600_distance_save[i] = now;

    float speedx = dist_E / (time_E > 0 ? time_E : 0.001f);
    speed_as5600[i] = speedx;
    as5600_delta_mm[i] = dist_E;

    // Only accumulate meters when filament is actively moving (not idling)
    if (motors[i].motion != filament_motion_enum::stop &&
        motors[i].motion != filament_motion_enum::pressure_ctrl_idle &&
        motors[i].motion != filament_motion_enum::pressure_ctrl_in_use) {
        data_save.filament[i].meters += dist_E / 1000.0f;
    }
  }
}

void MMU_Logic::UpdateLEDStatus(int channel) {
  MotorChannel &m = motors[channel];

  // Advanced LED logic for Auto-Feed mode
  if (m.SET_AUTO_FEED || m.motion == filament_motion_enum::pressure_ctrl_in_use) {
    // Standard rule: Nothing burns if no filament
    if (MC_ONLINE_key_stu[channel] == 0) {
      _hal->SetLED(channel, 0, 0, 0);
      return;
    }

    float zero = data_save.pressure_zero[channel];
    float actual_offset = m.use_overflow ? data_save.pressure_offset : 0.0f;
    float target_zero = zero - actual_offset; 
    float error = target_zero - MC_PULL_stu_raw[channel];
    float abs_error = __builtin_fabsf(error);
    float tol = data_save.pressure_tolerance;

    // Locked State: Show Purple
    if (m.pull_state_old) {
      _hal->SetLED(channel, 120, 0, 150);
      return;
    }

    // Region 3: Manual Zone (> 0.30V error from target)
    if (error < -0.30f) { // Too much pressure (slack)
      _hal->SetLED(channel, 255, 0, 0);
      return;
    }
    if (error > 0.30f) { // Way too little pressure (deep tension)
      _hal->SetLED(channel, 0, 0, 255);
      return;
    }

    // Region 1: Neutral Zone (< tolerance) - Static Orange
    if (abs_error < tol) {
      _hal->SetLED(channel, 255, 80, 0);
      return;
    }

    // Region 2: Work Zone (tolerance to 0.30V) - Orange Strobe
    // freq: 1Hz (at tol) to 15Hz (at 0.30V)
    float freq = 1.0f + (abs_error - tol) * (14.0f / (0.30f - tol));
    uint32_t period_ms = (uint32_t)(1000.0f / freq);
    bool blink_on = (_hal->GetTimeMS() / (period_ms / 2)) % 2 == 0;

    if (blink_on) {
      _hal->SetLED(channel, 255, 80, 0);
    } else {
      _hal->SetLED(channel, 0, 0, 0);
    }
    return;
  }

  // FTE Indication: Cyan Strobe (10Hz)
  if (m.motion == filament_motion_enum::feed_to_extruder) {
    bool blink_on = (_hal->GetTimeMS() / 50) % 2 == 0; // 10Hz
    if (blink_on) {
      _hal->SetLED(channel, 0, 255, 255);
    } else {
      _hal->SetLED(channel, 0, 0, 0);
    }
    return;
  }

  if (MC_PULL_stu[channel] == 1) {
    _hal->SetLED(channel, 255, 0, 0);
  } else if (MC_PULL_stu[channel] == -1) {
    _hal->SetLED(channel, 0, 0, 255);
  } else if (MC_PULL_stu[channel] == 0) {
    if (MC_ONLINE_key_stu[channel] == 1) {
      FilamentState &f = data_save.filament[channel];
      // If no color stored in flash (all zeros), default to white
      if (f.color_R == 0 && f.color_G == 0 && f.color_B == 0) {
        _hal->SetLED(channel, 255, 255, 255);
      } else {
        _hal->SetLED(channel, f.color_R, f.color_G, f.color_B);
      }
    } else {
      _hal->SetLED(channel, 0, 0, 0);
    }
  }
}

void MMU_Logic::RunMotorChannel(int CHx, float time_E) {
  MotorChannel &m = motors[CHx];

  // Auto-resume pressure control if enabled and idle
  if (m.SET_AUTO_FEED &&
      (m.motion == filament_motion_enum::stop ||
       m.motion == filament_motion_enum::pressure_ctrl_idle)) {
    m.SetMotion(filament_motion_enum::pressure_ctrl_in_use);
  }

  // Diagnostic Override
  if (diag_active[CHx]) {
    if (_hal->GetTimeMS() < diag_end_time[CHx]) {
      _hal->SetMotorPower(CHx, diag_pwm[CHx]);
      return;
    } else {
      diag_active[CHx] = false;
      _hal->SetMotorPower(CHx, 0);
    }
  }

  // Distance Accumulation
  float dist_step = __builtin_fabsf(as5600_delta_mm[CHx]);
  if (is_backing_out) {
    last_total_distance[CHx] += dist_step;
  }
  if ((m.motion == filament_motion_enum::velocity_control ||
       m.motion == filament_motion_enum::feed_to_extruder) &&
      m.target_distance > 0) {
    m.accumulated_distance += dist_step;
    if (m.motion == filament_motion_enum::velocity_control &&
        m.accumulated_distance >= m.target_distance) {
      m.SetMotion(filament_motion_enum::stop);
    }
  }

  float speed_set = 0;
  float now_speed = -speed_as5600[CHx]; // Hardware specific: Forward motion produces negative raw encoder counts
  float x = 0;

  // Logic extraction from ControlLogic "Run" loop part
  if (m.motion == filament_motion_enum::pressure_ctrl_idle) { // Idle
    // Slider always active: no filament sensor guard
    if (m.pressure_ctrl == pressure_control_enum::reverse) {
      x = -m.PID_pressure.Calculate(
              data_save.pressure_zero[CHx] - MC_PULL_stu_raw[CHx], time_E);

      if (x > 0) x += data_save.pressure_min_pwm;
      else if (x < 0) x -= data_save.pressure_min_pwm;
    } else if (MC_PULL_stu[CHx] != 0) {
      x = m.PID_pressure.Calculate(
              data_save.pressure_zero[CHx] - MC_PULL_stu_raw[CHx], time_E);

      if (x > 0) x += data_save.pressure_min_pwm;
      else if (x < 0) x -= data_save.pressure_min_pwm;
    } else {
      x = 0;
      m.PID_pressure.Clear();
    }
  } else if (MC_ONLINE_key_stu[CHx] != 0 ||
             m.motion == filament_motion_enum::velocity_control ||
             m.motion == filament_motion_enum::feed_to_extruder) {

    if (m.motion == filament_motion_enum::pressure_ctrl_in_use) {
      float zero = data_save.pressure_zero[CHx];
      float actual_offset = m.use_overflow ? data_save.pressure_offset : 0.0f;
      float target_zero = zero - actual_offset; 
      float tol = data_save.pressure_tolerance;

      // Auto-Feed logic: Maintain tension
      if (m.pull_state_old && MC_PULL_stu_raw[CHx] < target_zero - (tol * 0.5f)) {
        m.pull_state_old = false;
      }

      if (!m.pull_state_old) {
        float error = target_zero - MC_PULL_stu_raw[CHx];
        uint64_t now_ms = _hal->GetTimeMS();

        // Dynamic Boost Trigger
        if (error > data_save.boost_threshold && now_ms > m.boost_end_time) {
          m.boost_end_time = now_ms + data_save.boost_time_ms;
        }

        if (now_ms < m.boost_end_time) {
          // ExtraBoost Pulse: Full power to overcome 1500mm tube friction
          x = m.dir * data_save.boost_pwm;
        } else {
          // Normal PID Regulation
          if (error > tol) {
            x = m.PID_pressure.Calculate(error - tol, time_E);
          } else if (error < -tol) {
            x = m.PID_pressure.Calculate(error + tol, time_E);
          } else {
            m.PID_pressure.Clear();
          }

          // Min PWM Floor to overcome motor deadzone
          if (x > 1.0f)
            x += data_save.pressure_min_pwm;
          else if (x < -1.0f)
            x -= data_save.pressure_min_pwm;
        }
      }
    } else {
      if (m.motion == filament_motion_enum::stop) {
        m.PID_speed.Clear();
        _hal->SetMotorPower(CHx, 0);
        return;
      }
      if (m.motion == filament_motion_enum::send) {
        if (device_type_addr == BambuBus_AMS_lite) {
          if (MC_PULL_stu_raw[CHx] < data_save.pressure_zero[CHx] + 0.05f)
            speed_set = MOTOR_SPEED_AMS_LITE_SEND;
          else
            speed_set = 0;
        } else
          speed_set = MOTOR_SPEED_SEND;
      }
      if (m.motion == filament_motion_enum::slow_send)
        speed_set = MOTOR_SPEED_SLOW_SEND;
      if (m.motion == filament_motion_enum::pull)
        speed_set = -MOTOR_SPEED_PULL;
      if (m.motion == filament_motion_enum::velocity_control ||
          m.motion == filament_motion_enum::feed_to_extruder)
        speed_set = m.target_velocity;

      // Smooth Acceleration (Ramping)
      float accel_limit = 1000.0f * time_E; // 1000 mm/s^2
      if (m.current_velocity_set < speed_set) {
        m.current_velocity_set += accel_limit;
        if (m.current_velocity_set > speed_set)
          m.current_velocity_set = speed_set;
      } else if (m.current_velocity_set > speed_set) {
        m.current_velocity_set -= accel_limit;
        if (m.current_velocity_set < speed_set)
          m.current_velocity_set = speed_set;
      }

      float error = m.current_velocity_set - now_speed;
      x = m.PID_speed.Calculate(error, time_E);

      // FTE Triggers
      if (m.motion == filament_motion_enum::feed_to_extruder) {
        _fte.dist_moved = m.accumulated_distance;

        // Trigger 1: Pressure
        float pressure_delta = MC_PULL_stu_raw[CHx] - data_save.pressure_zero[CHx];
        if (pressure_delta > _fte.pressure_threshold) {
          m.SetMotion(filament_motion_enum::stop);
          _fte.done = true; _fte.active = false;
          _fte.reason = StopReason::pressure;
          _hal->SetMotorPower(CHx, 0); return;
        }

        // Trigger 2: Stall (configurable timeout)
        if (__builtin_fabsf(x) > 900 && __builtin_fabsf(now_speed) < 5.0f) {
          if (m.stall_timer == 0) m.stall_timer = _hal->GetTimeMS();
          if (_hal->GetTimeMS() - m.stall_timer > _fte.stall_ms) {
            m.SetMotion(filament_motion_enum::stop);
            _fte.done = true; _fte.active = false;
            _fte.reason = StopReason::stall;
            _hal->SetMotorPower(CHx, 0); return;
          }
        } else { m.stall_timer = 0; }

        // Trigger 3: Distance
        if (m.accumulated_distance >= _fte.max_mm) {
          m.SetMotion(filament_motion_enum::stop);
          _fte.done = true; _fte.active = false;
          _fte.reason = StopReason::distance;
          _hal->SetMotorPower(CHx, 0); return;
        }
      }

      // Normal Stall Detection (500ms)
      if (m.motion != filament_motion_enum::feed_to_extruder &&
          __builtin_fabsf(x) > 900 && __builtin_fabsf(now_speed) < 5.0f) {
        if (m.stall_timer == 0)
          m.stall_timer = _hal->GetTimeMS();
        if (_hal->GetTimeMS() - m.stall_timer > 500) {
          m.SetMotion(filament_motion_enum::stop);
          x = 0;
        }
      } else if (m.motion != filament_motion_enum::feed_to_extruder) {
        m.stall_timer = 0;
      }
    }
  } else {
    x = 0;
  }

  if (x > 1.0f)
    x += data_save.move_pwm_zero;
  else if (x < -1.0f)
    x -= data_save.move_pwm_zero;
  else
    x = 0;

  if (x > 1000)
    x = 1000;
  if (x < -1000)
    x = -1000;

  _hal->SetMotorPower(CHx, (int)x);
  UpdateLEDStatus(CHx);
}

bool MMU_Logic::Prepare_For_filament_Pull_Back(float OUT_filament_meters) {
  bool wait = false;
  for (int i = 0; i < 4; i++) {
    if (filament_now_position[i] == filament_pulling_back) {
      if (last_total_distance[i] < OUT_filament_meters) {
        motors[i].SetMotion(filament_motion_enum::pull);
        // LED Logic
        float npercent =
            (last_total_distance[i] / OUT_filament_meters) * 100.0f;
        int r = 255 - ((255 / 100) * (int)npercent);
        int g = 125 - ((125 / 100) * (int)npercent);
        int b = (255 / 100) * (int)npercent;
        if (r < 0)
          r = 0;
        if (g < 0)
          g = 0;
        if (b < 0)
          b = 0;
        _hal->SetLED(i, r, g, b);
      } else {
        is_backing_out = false;
        motors[i].SetMotion(filament_motion_enum::stop);
        filament_now_position[i] = filament_idle;
        data_save.filament[i].motion_set = AMS_filament_motion::idle;
        last_total_distance[i] = 0;
      }
      wait = true;
    }
  }
  return wait;
}

void MMU_Logic::motor_motion_switch() {
  int num = data_save.BambuBus_now_filament_num;
  // Logic mostly identical to before, updating member vars

  for (int i = 0; i < 4; i++) {
    if (motors[i].motion == filament_motion_enum::velocity_control)
      continue;
    if (motors[i].motion == filament_motion_enum::pressure_ctrl_in_use)
      continue;
    if (motors[i].motion == filament_motion_enum::feed_to_extruder)
      continue;

    if (i != num) {
      filament_now_position[i] = filament_idle;
      motors[i].SetMotion(filament_motion_enum::pressure_ctrl_idle);
    } else {
      if (filament_now_position[num] == filament_unloading) {
        bool done = false;
        if (unload_target_dist[num] == -1) {
          if (MC_ONLINE_key_stu[num] == 0)
            done = true;
        } else {
          if (last_total_distance[num] >= (float)unload_target_dist[num])
            done = true;
        }

        if (done) {
          motors[num].SetMotion(filament_motion_enum::stop);
          motors[num].PID_speed.Clear();
          motors[num].accumulated_distance = 0;
          motors[num].current_velocity_set = 0;
          motors[num].stall_timer = 0;
          filament_now_position[num] = filament_idle;
          is_backing_out = false;
          return;
        }
        return;
      }

      if (MC_ONLINE_key_stu[num] > 0) {
        AMS_filament_motion current_motion = data_save.filament[num].motion_set;

        if (filament_now_position[num] == filament_loading) {
          float pressure = MC_PULL_stu_raw[num];
          float zero = data_save.pressure_zero[num];

          bool dist_done = false;
          if (unload_target_dist[num] > 0) {
            if (last_total_distance[num] >= (float)unload_target_dist[num])
              dist_done = true;
          }

          if (pressure > zero + 0.20f) {
            filament_now_position[num] = filament_using;
            motors[num].pull_state_old = true;
            motors[num].SetMotion(filament_motion_enum::pressure_ctrl_in_use);
          } else if (dist_done) {
            filament_now_position[num] = filament_idle;
            motors[num].SetMotion(filament_motion_enum::pressure_ctrl_idle);
            is_backing_out = false;
          } else if (pressure > zero + 0.05f) {
            motors[num].SetMotion(filament_motion_enum::slow_send);
          } else {
            motors[num].SetMotion(filament_motion_enum::send);
          }
          return;
        }

        switch (current_motion) {
        case AMS_filament_motion::need_send_out:
          _hal->SetLED(num, 0, 255, 0);
          filament_now_position[num] = filament_sending_out;
          motors[num].SetMotion(filament_motion_enum::send);
          break;

        case AMS_filament_motion::need_pull_back:
          motors[num].pull_state_old = false;
          is_backing_out = true;
          filament_now_position[num] = filament_pulling_back;
          if (device_type_addr == BambuBus_AMS_lite) {
            motors[num].SetMotion(filament_motion_enum::pull);
          }
          break;

        case AMS_filament_motion::before_pull_back:
        case AMS_filament_motion::in_use: {
          uint64_t time_now = get_time64();

          if (filament_now_position[num] == filament_sending_out) {
            is_backing_out = false;
            motors[num].pull_state_old = true;
            filament_now_position[num] = filament_using;
            _motion_switch_time_end = time_now + 1500;
          } else if (filament_now_position[num] == filament_using) {
            last_total_distance[num] = 0; // Fix index i -> num
            if (time_now > _motion_switch_time_end) {
              _hal->SetLED(num, 255, 255, 255);
              motors[num].SetMotion(filament_motion_enum::pressure_ctrl_in_use);
            } else {
              _hal->SetLED(num, 128, 192, 128);
              motors[num].SetMotion(filament_motion_enum::slow_send);
            }
          }
          break;
        }
        case AMS_filament_motion::idle:
          filament_now_position[num] = filament_idle;
          motors[num].SetMotion(filament_motion_enum::pressure_ctrl_idle);
          break;
        }
      }
    }
  }

  /*
  // Auto-Load trigger disabled per user request
  if (filament_now_position[num] == filament_idle && MC_ONLINE_key_stu[num] > 0)
  { StartLoadFilament(num, 0);
  }
  */
}

void MMU_Logic::Run() {
  uint64_t now = _hal->GetTimeUS();
  if (_last_run_us == 0) {
    _last_run_us = now;
    return;
  }
  float time_E = (float)(now - _last_run_us) / 1000000.0f;
  _last_run_us = now;

  MC_PULL_ONLINE_read();
  AS5600_Update(time_E);

  if (Bambubus_need_to_save) {
    // Smart save timing: wait for serial silence to avoid blocking during
    // communication Option 1: Save after 500ms of serial idle (fast response
    // during dead time) Option 2: Force save after 5000ms absolute (safety
    // fallback)
    extern bool KlipperCLI_IsSerialIdle(uint32_t);   // Forward declaration
    bool serial_idle = KlipperCLI_IsSerialIdle(500); // 500ms of silence
    bool timeout_hit = (now - save_timer > 5000);    // 5s absolute max

    if (serial_idle || timeout_hit) {
      SaveSettings();
    }
  }

  // 200.0f = OUT_filament_meters constant
  bool pulling = Prepare_For_filament_Pull_Back(200.0f);

  if (!pulling) {
    motor_motion_switch();
  }

  for (int i = 0; i < 4; i++) {
    RunMotorChannel(i, time_E);
  }

  _hal->LED_Show();

  // System LED Debug Flash
  if (now - _last_led_update > 1000) {
    _led_toggle = !_led_toggle;
    // Heartbeat: White for Klipper/Refactored
    if (_led_toggle) {
      _hal->SetLED(4, 10, 10, 10); // White
    } else
      _hal->SetLED(4, 0, 0, 0);
    _last_led_update = now;
  }
  // LED_Show() called at end of Run() — pushes buffered pixels to strip
}

void MMU_Logic::MC_PULL_ONLINE_read() {
  for (int i = 0; i < 4; i++) {
    float raw_p = _hal->GetPressureReading(i);
    pressure_filtered[i] = (raw_p * 0.4f) + (pressure_filtered[i] * 0.6f);
    MC_PULL_stu_raw[i] = pressure_filtered[i];

    float raw_o = _hal->GetPresenceVoltage(i);
    MC_ONLINE_key_stu_raw[i] = raw_o;

    bool present = _hal->GetFilamentPresence(i);
    MC_ONLINE_key_stu[i] = present ? 1 : 0;

    // Sync Filament Status for Bambu Protocol
    if (MC_ONLINE_key_stu[i] != 0) {
      if (data_save.filament[i].status == AMS_filament_status::offline) {
        data_save.filament[i].status = AMS_filament_status::online;
      }
    } else {
      if (data_save.filament[i].status == AMS_filament_status::online) {
        data_save.filament[i].status = AMS_filament_status::offline;
      }
    }

    // Pressure State Calculation (-1, 0, 1) - Manual Mode Threshold (0.30V)
    float zero = data_save.pressure_zero[i];
    if (pressure_filtered[i] > zero + 0.30f)
      MC_PULL_stu[i] = 1;
    else if (pressure_filtered[i] < zero - 0.30f)
      MC_PULL_stu[i] = -1;
    else
      MC_PULL_stu[i] = 0;

    data_save.filament[i].pressure = (uint16_t)(raw_p * 1000.0f);
  }
}

void MMU_Logic::SetPressureTolerance(float tol) {
  data_save.pressure_tolerance = tol;
  SetNeedToSave();
}

void MMU_Logic::SetPressureGain(float gain) {
  data_save.pressure_gain = gain;
  for (int i = 0; i < 4; i++) {
    motors[i].PID_pressure.Init(gain, 0, 0);
  }
  SetNeedToSave();
}

void MMU_Logic::SetPressureMinPWM(float pwm) {
  data_save.pressure_min_pwm = pwm;
  SetNeedToSave();
}

void MMU_Logic::SetPressureOffset(float offset) {
  data_save.pressure_offset = offset;
  SetNeedToSave();
}

void MMU_Logic::SetBoostThreshold(float threshold) {
  data_save.boost_threshold = threshold;
  SetNeedToSave();
}

void MMU_Logic::SetBoostPWM(float pwm) {
  data_save.boost_pwm = pwm;
  SetNeedToSave();
}

void MMU_Logic::SetBoostTime(uint32_t ms) {
  data_save.boost_time_ms = ms;
  SetNeedToSave();
}

void MMU_Logic::SetRetractDeadzone(float deadzone) {
  data_save.retract_deadzone = deadzone;
  SetNeedToSave();
}

// User Actions
void MMU_Logic::SetFilamentInfoAction(int id, const FilamentInfo &info,
                                      float meters) {
  if (id < 0 || id >= 4)
    return;
  FilamentState &target = data_save.filament[id];

  bool changed = false;

  if (__builtin_memcmp(target.ID, info.ID, sizeof(target.ID)) != 0) {
    __builtin_memcpy(target.ID, info.ID, sizeof(target.ID));
    changed = true;
  }

  if (__builtin_memcmp(target.name, info.name, sizeof(target.name)) != 0) {
    __builtin_memcpy(target.name, info.name, sizeof(target.name));
    changed = true;
  }

  if (target.color_R != info.color_R || target.color_G != info.color_G ||
      target.color_B != info.color_B || target.color_A != info.color_A) {
    target.color_R = info.color_R;
    target.color_G = info.color_G;
    target.color_B = info.color_B;
    target.color_A = info.color_A;
    changed = true;
  }

  if (target.temperature_min != info.temperature_min ||
      target.temperature_max != info.temperature_max) {
    target.temperature_min = info.temperature_min;
    target.temperature_max = info.temperature_max;
    changed = true;
  }

  if (meters != target.meters) { // Only update if changed (ignoring previous
                                 // >=0 check to allow clamping logic)
    float valid_meters = meters;

    // User requested: <0 = 0
    if (valid_meters < 0.0f)
      valid_meters = 0.0f;

    // User requested: ceiling like 3000.0 (prevents integer overflow and
    // nonsensical values)
    if (valid_meters > 3000.0f)
      valid_meters = 3000.0f;

    if (target.meters != valid_meters) {
      target.meters = valid_meters;
      changed = true;
    }
  }

  target.ID[7] = 0;
  target.name[19] = 0;

  if (changed) {
    SetNeedToSave();
  }
}

void MMU_Logic::StartLoadFilament(int tray, int length_mm) {
  if (tray < 0 || tray >= 4)
    return;
  data_save.BambuBus_now_filament_num = tray;
  data_save.filament_use_flag = 0x02;
  filament_now_position[tray] = filament_loading;
  motors[tray].SetMotion(filament_motion_enum::send);
  unload_target_dist[tray] = length_mm;
  if (length_mm > 0) {
    last_total_distance[tray] = 0;
    is_backing_out = true;
  } else {
    is_backing_out = false;
  }
  for (int i = 0; i < 4; i++) {
    if (i != tray) {
      filament_now_position[i] = filament_idle;
      motors[i].SetMotion(filament_motion_enum::pressure_ctrl_idle);
    }
  }
}

void MMU_Logic::StartUnloadFilament(int tray, int length_mm) {
  if (tray < 0 || tray >= 4)
    return;
  data_save.BambuBus_now_filament_num = tray;
  data_save.filament_use_flag = 0x02;
  filament_now_position[tray] = filament_unloading;
  motors[tray].SetMotion(filament_motion_enum::pull);
  unload_target_dist[tray] = length_mm;
  unload_start_meters[tray] = last_total_distance[tray];
  is_backing_out = true;
  last_total_distance[tray] = 0;
  for (int i = 0; i < 4; i++) {
    if (i != tray) {
      filament_now_position[i] = filament_idle;
      motors[i].SetMotion(filament_motion_enum::pressure_ctrl_idle);
    }
  }
}

void MMU_Logic::MoveAxis(int axis, float dist_mm, float speed) {
  if (axis < 0 || axis >= 4)
    return;
  motors[axis].target_velocity =
      (dist_mm >= 0) ? __builtin_fabsf(speed) : -__builtin_fabsf(speed);
  motors[axis].target_distance = __builtin_fabsf(dist_mm);
  motors[axis].SetMotion(filament_motion_enum::velocity_control);
}

void MMU_Logic::StopAll() {
  for (int i = 0; i < 4; i++) {
    motors[i].SET_AUTO_FEED = false;
    motors[i].SetMotion(filament_motion_enum::stop);
    filament_now_position[i] = filament_idle;
  }
  _fte.active = false;
}

void MMU_Logic::SetCurrentFilamentIndex(int index) {
  if (index >= 0 && index < 4) {
    data_save.BambuBus_now_filament_num = index;
  }
}

void MMU_Logic::SetAutoFeed(int lane, bool enable, bool overflow) {
  if (lane < 0) {
    // Apply to all lanes or active? Usually all if -1
    for(int i=0; i<4; i++) {
        motors[i].SET_AUTO_FEED = enable;
        motors[i].use_overflow = overflow;
        if (enable) motors[i].SetMotion(filament_motion_enum::pressure_ctrl_in_use);
        else motors[i].SetMotion(filament_motion_enum::pressure_ctrl_idle);
    }
    return;
  }
  if (lane >= 4) return;
  
  motors[lane].SET_AUTO_FEED = enable;
  motors[lane].use_overflow = overflow;
  if (enable) {
    motors[lane].SetMotion(filament_motion_enum::pressure_ctrl_in_use);
  } else {
    motors[lane].SetMotion(filament_motion_enum::pressure_ctrl_idle);
  }
}

uint16_t MMU_Logic::GetSensorState() {
  uint16_t state = 0;
  for (int i = 0; i < 4; i++) {
    if (MC_ONLINE_key_stu[i] != 0)
      state |= (1 << i);
  }
  return state;
}

int MMU_Logic::GetLaneMotion(int lane) {
  if (lane < 0 || lane >= 4)
    return 0;
  return (int)motors[lane].motion;
}

FilamentState &MMU_Logic::GetFilament(int index) {
  if (index < 0 || index >= 4)
    return data_save.filament[0];
  return data_save.filament[index];
}

void MMU_Logic::DiagnosticMotorControl(int lane, int pwm,
                                       uint32_t duration_ms) {
  if (lane < 0 || lane >= 4)
    return;
  diag_pwm[lane] = pwm;
  diag_end_time[lane] = _hal->GetTimeMS() + duration_ms;
  diag_active[lane] = true;
}

int MMU_Logic::GetCurrentFilamentIndex() {
  return data_save.BambuBus_now_filament_num;
}

uint16_t MMU_Logic::GetDeviceType() { return device_type_addr; }

float MMU_Logic::GetPressureZero(int lane) {
  if (lane < 0 || lane >= 4)
    return 1.65f;
  return data_save.pressure_zero[lane];
}

CalibrateResult MMU_Logic::CalibratePressure(int lane) {
  CalibrateResult res;
  res.ok = true;
  res.error_msg = nullptr;
  res.value = 0.0f;

  if (lane < -1 || lane >= 4) {
    res.ok = false;
    res.error_msg = "Invalid lane";
    return res;
  }

  bool saved = false;

  if (lane == -1) {
    // Calibrate all empty lanes
    for (int i = 0; i < 4; i++) {
      if (_hal->GetFilamentPresence(i))
        continue; // Skip busy lanes

      float raw = _hal->GetPressureReading(i);
      data_save.pressure_zero[i] = raw;
      saved = true;
    }
  } else {
    // Calibrate specific lane
    if (_hal->GetFilamentPresence(lane)) {
      res.ok = false;
      res.error_msg = "Lane is busy with filament";
      return res;
    }

    float raw = _hal->GetPressureReading(lane);
    data_save.pressure_zero[lane] = raw;
    res.value = raw;
    saved = true;
  }

  if (saved) {
    SetNeedToSave();
  }

  return res;
}

void MMU_Logic::StartFeedToExtruder(int lane, float speed, float max_mm,
                                     float pressure_threshold, uint32_t stall_ms, int cmd_id) {
    if (lane < 0 || lane >= 4 || _fte.active) return;

    _fte.active             = true;
    _fte.done               = false;
    _fte.lane               = lane;
    _fte.cmd_id             = cmd_id;
    _fte.speed              = speed;
    _fte.max_mm             = max_mm;
    _fte.pressure_threshold = pressure_threshold;
    _fte.stall_ms           = stall_ms;
    _fte.start_ms           = _hal->GetTimeMS();
    _fte.reason             = StopReason::none;
    _fte.dist_moved         = 0.0f;

    // Start motion using the new mode
    motors[lane].accumulated_distance = 0.0f;
    motors[lane].current_velocity_set = 0.0f;
    motors[lane].stall_timer = 0;
    motors[lane].target_velocity = __builtin_fabsf(speed);
    motors[lane].target_distance = max_mm;
    motors[lane].SetMotion(filament_motion_enum::feed_to_extruder);
}

MMU_Logic::StopReason MMU_Logic::GetFTEResult(float &dist_moved_mm) {
    if (!_fte.done) return StopReason::none;
    dist_moved_mm = _fte.dist_moved;
    return _fte.reason;
}

void MMU_Logic::ClearFTEResult() {
    _fte.done = false;
    _fte.reason = StopReason::none;
}

