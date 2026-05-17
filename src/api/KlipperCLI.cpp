/*
* DEVELOPMENT STATE: TESTING
* This file implements the Klipper JSON protocol which is currently in a testing state.
*/
#include "KlipperCLI.h"
#include "../interfaces/I_MMU_Transport.h"
#include "../core/MMU_Logic.h"
#include "../core/UnitState.h"
#include "../libs/LiteJSON.h"
#include "../hal/Hardware.h"
#include <Arduino.h>
#include <string.h>
#include <ctype.h>
#include <limits.h>

namespace KlipperCLI {

    static MMU_Logic* _mmu = nullptr;
    static I_MMU_Transport* _transport = nullptr; // Primary
    static I_MMU_Transport* _aux_transport = nullptr; // Auxiliary

    static char rx_buffer[1024]; // Primary RX buffer
    static int rx_idx = 0;

    static char rx_buffer_aux[1024]; // Auxiliary RX buffer
    static int rx_idx_aux = 0;

    static bool last_was_cr = false;
    static JsonDocument doc;
    static char global_json_buf[2048]; // Shared buffer for all responses
    static uint64_t last_activity_time = 0; // Track last serial activity for smart save timing
    static uint64_t last_fte_check = 0;

    // Pointer to track which transport received the request currently being processed
    static I_MMU_Transport* _current_response_transport = nullptr;

    const char* get_sign(float val) { return (val < 0) ? "-" : ""; }

    // Response Helper
    void WaitTX(I_MMU_Transport* t) {
        if (!t) return;
        uint32_t timeout = 200; 
        uint64_t start = millis();
        while (t->IsBusy() && (millis() - start < timeout)) {
            Hardware::DelayUS(100); // Reduced from 1ms to improve throughput
        }
    }

    void WaitTX() {
        WaitTX(_current_response_transport ? _current_response_transport : _transport);
    }

    void SendResponse(JsonDocument& d) {
        I_MMU_Transport* t = _current_response_transport ? _current_response_transport : _transport;
        if (!t) return;
        WaitTX(t);
        size_t len = serializeJson(d, global_json_buf, sizeof(global_json_buf) - 2);
        global_json_buf[len++] = '\r';
        global_json_buf[len++] = '\n';
        t->Write((const uint8_t*)global_json_buf, len);
    }
    
    void SendError(int id, const char* code, const char* msg) {
        doc.clear();
        doc["id"] = id;
        doc["ok"] = false;
        doc["code"] = code;
        doc["msg"] = msg;
        SendResponse(doc);
    }
    
    void SendOk(int id, const char* code = nullptr, const char* msg = nullptr) {
        doc.clear();
        doc["id"] = id;
        doc["ok"] = true;
        if(code) doc["code"] = code;
        if(msg) doc["msg"] = msg;
        SendResponse(doc);
    }

    // --- Command Handlers ---
    
    void HandlePing(int id, JsonObject args) {
        doc.clear();
        doc["id"] = id;
        doc["cmd"] = "PING";
        doc["ok"] = true;
        doc["result"] = "ok";
        
        LiteObject& t = doc["telemetry"].makeObject();
        t["version"] = DEVICE_VERSION; 
        t["uptime"] = (int)millis();
        
        SendResponse(doc);
    }

    void HandleStatus(int id, JsonObject args) {
         if (!_mmu) return;
         WaitTX();
         int offset = 0;
         offset += snprintf(global_json_buf + offset, sizeof(global_json_buf) - offset, 
             "{\"id\":%d,\"cmd\":\"STATUS\",\"ok\":true,\"lanes\":[", id);
             
         uint16_t sensors = _mmu->GetSensorState();
         
         for(int i=0; i<4; i++) {
            if(i > 0) offset += snprintf(global_json_buf + offset, sizeof(global_json_buf) - offset, ",");
            
            int m_val = _mmu->GetLaneMotion(i);
            const char* m_str = "Idle";
            switch(m_val) {
                case 0: m_str = "Idle"; break;
                case 1: m_str = "Feed"; break;
                case 2: m_str = "Retract"; break;
                case 3: m_str = "SlowFeed"; break;
                case 5: m_str = "AutoFeed"; break; 
                case 7: m_str = "VelCtrl"; break;
                case 8: m_str = "FTE"; break;
                default: m_str = "Idle"; break;
            }
            
            LaneState &f = _mmu->GetLane(i);
            
             float meters_f = f.meters;
             if (!isfinite(meters_f)) meters_f = 0;
             if (meters_f > 2000000000.0f) meters_f = 2000000000.0f;
             if (meters_f < -2000000000.0f) meters_f = -2000000000.0f;

             int m_int = (int)meters_f;
             int m_dec = (int)((meters_f - m_int) * 100); 
             if(m_dec < 0) m_dec = -m_dec; 
            
             int p_int = f.pressure / 1000;
             int p_dec = f.pressure % 1000;
             
             float p_zero_f = _mmu->GetPressureZero(i);
             int p_zero_int = (int)p_zero_f;
             int p_zero_dec = (int)((p_zero_f - p_zero_int) * 1000);
             if (p_zero_dec < 0) p_zero_dec = -p_zero_dec;
            
             if (offset >= (int)sizeof(global_json_buf) - 128) {
                 // Danger zone: not enough space for a full lane record
                 break; 
             }
             
             // Precision Fix: Handle negative sign for meters between -1.0 and 0.0
             const char* sign = (meters_f < 0 && m_int == 0) ? "-" : "";

             int n = snprintf(global_json_buf + offset, sizeof(global_json_buf) - offset, 
                 "{\"id\":%d,\"present\":%s,\"motion\":\"%s\",\"autofeed\":%s,\"overflow\":%s,\"meters\":%s%d.%02d,\"pressure\":%d.%03d,\"pressure_zero\":%d.%03d}",
                 i, 
                 (sensors & (1<<i)) ? "true" : "false",
                 m_str,
                 _mmu->GetLaneAutoFeed(i) ? "true" : "false",
                 _mmu->GetLaneOverflow(i) ? "true" : "false",
                 sign, m_int, m_dec, p_int, p_dec, 
                 p_zero_int, p_zero_dec);
             
             if (n > 0) {
                 if (offset + n >= (int)sizeof(global_json_buf)) {
                     SendError(id, "BUFFER_OVERFLOW", "Status too large");
                     return;
                 }
                 offset += n;
             }
         }
         
         // Global Advanced Parameters
         float tol = _mmu->GetPressureTolerance();
         int tol_int = (int)__builtin_fabsf(tol);
         int tol_dec = (int)((__builtin_fabsf(tol) - tol_int) * 1000);

         float gain = _mmu->GetPressureGain();
         int g_int = (int)__builtin_fabsf(gain);
         int g_dec = (int)((__builtin_fabsf(gain) - g_int) * 10);

         float offset_val = _mmu->GetPressureOffset();
         int off_int = (int)__builtin_fabsf(offset_val);
         int off_dec = (int)((__builtin_fabsf(offset_val) - off_int) * 1000);

         float bth = _mmu->GetBoostThreshold();
         int bth_int = (int)__builtin_fabsf(bth);
         int bth_dec = (int)((__builtin_fabsf(bth) - bth_int) * 1000);

         float bpw = _mmu->GetBoostPWM();
         int bpw_int = (int)__builtin_fabsf(bpw);
         int bpw_dec = (int)((__builtin_fabsf(bpw) - bpw_int) * 10);

         float dz = _mmu->GetRetractDeadzone();
         int dz_int = (int)__builtin_fabsf(dz);
         int dz_dec = (int)((__builtin_fabsf(dz) - dz_int) * 100);

         float mpw = _mmu->GetPressureMinPWM();
         int mpw_int = (int)__builtin_fabsf(mpw);
         int mpw_dec = (int)((__builtin_fabsf(mpw) - mpw_int) * 10);

          flash_save_struct *f_ptr = (flash_save_struct *)(uintptr_t)(0x0800F000);
          float fz0 = f_ptr->pressure_zero[0];
          int f0_int = (int)fz0;
          int f0_dec = (int)((__builtin_fabsf(fz0) - __builtin_fabsf((float)f0_int)) * 1000.0f);
          if (f0_dec < 0) f0_dec = -f0_dec;

          offset += snprintf(global_json_buf + offset, sizeof(global_json_buf) - offset, 
              "],\"p_tol\":%s%d.%03d,\"p_gain\":%s%d.%01d,\"p_offset\":%s%d.%03d,\"p_boost_thr\":%s%d.%03d,\"p_boost_pwm\":%s%d.%01d,\"p_boost_time\":%u,\"p_deadzone\":%s%d.%02d,\"p_min_pwm\":%s%d.%01d,\"m_p\":%d.%d,\"m_i\":%d.%d,\"m_d\":%d.%d,\"m_zero\":%d,\"f_chk\":%u,\"f_ver\":%u,\"f_z0\":%d.%03d}\r\n",
              get_sign(tol), tol_int, tol_dec,
              get_sign(gain), g_int, g_dec,
              get_sign(offset_val), off_int, off_dec,
              get_sign(bth), bth_int, bth_dec,
              get_sign(bpw), bpw_int, bpw_dec,
              (unsigned int)_mmu->GetBoostTime(),
              get_sign(dz), dz_int, dz_dec,
              get_sign(mpw), mpw_int, mpw_dec,
              (int)_mmu->GetMoveP(), (int)(_mmu->GetMoveP() * 10) % 10,
              (int)_mmu->GetMoveI(), (int)(_mmu->GetMoveI() * 10) % 10,
              (int)_mmu->GetMoveD(), (int)(_mmu->GetMoveD() * 10) % 10,
              (int)_mmu->GetMovePwmZero(),
              (unsigned int)f_ptr->check,
              (unsigned int)f_ptr->version,
              f0_int, f0_dec);
         I_MMU_Transport* t = _current_response_transport ? _current_response_transport : _transport;
         if (t) t->Write((const uint8_t*)global_json_buf, offset);
    }
    
    void HandleGetSensors(int id, JsonObject args) {
         if (!_mmu) return;
         doc.clear();
         doc["id"] = id;
         doc["cmd"] = "GET_SENSORS";
         doc["ok"] = true;
         
         uint16_t state = _mmu->GetSensorState();
         LiteArray& lanes = doc["lane"].makeArray();
         for(int i=0; i<4; i++) {
             lanes.add((state & (1<<i)) ? 1 : 0);
         }
         SendResponse(doc);
    }
    
    void HandleMove(int id, JsonObject args) {
        if (!_mmu) return;
        if(!args["axis"].isString() || !args["dist_mm"].isFloat() || !args["speed"].isFloat()) {
             SendError(id, "BAD_ARGS", "Missing axis, dist, or speed"); 
             return;
        }
        
        char axis[16]; strncpy(axis, args["axis"], 15); axis[15]=0;
        float dist = args["dist_mm"];
        float speed = args["speed"];
        
        int motor_idx = -1;
        if(strcmp(axis, "FEED") == 0) {
            motor_idx = _mmu->GetActiveLaneIndex();
        } else if(strcmp(axis, "SELECTOR") == 0) {
        } else if (isdigit(axis[0])) {
             motor_idx = atoi(axis);
        }
        
        if (motor_idx >= 0 && motor_idx < 4) {
             _mmu->MoveAxis(motor_idx, dist, speed);
             SendOk(id, "MOVING", "Motion started");
        } else {
             SendError(id, "BAD_AXIS", "Invalid or unknown axis");
        }
    }

    void HandleStop(int id, JsonObject args) {
        if (!_mmu) return;
        _mmu->StopAll();
        SendOk(id, "STOPPED", "All motion stopped");
    }

    void HandleSelectLane(int id, JsonObject args) {
        if (!_mmu) return;
        if(!args["lane"].isInt()) {
            SendError(id, "BAD_ARGS", "Missing lane");
            return;
        }
        int lane = args["lane"];
        if(lane < 0 || lane >= 4) {
            SendError(id, "BAD_LANE", "Lane must be 0-3");
            return;
        }
        
        _mmu->SetActiveLaneIndex(lane);
        SendOk(id);
    }

    void HandleSetAutoFeed(int id, LiteObject& args) {
        if (!_mmu) return;
        int lane = args["lane"] | -1;
        
        // Incremental logic: if key is missing, keep current state
        bool enable;
        if (args.containsKey("enable")) {
            enable = args["enable"];
        } else if (args.containsKey("ENABLE")) {
            enable = args["ENABLE"];
        } else {
            enable = (lane >= 0 && lane < 4) ? _mmu->GetLaneAutoFeed(lane) : false;
        }

        bool overflow;
        if (args.containsKey("overflow")) {
            overflow = args["overflow"];
        } else if (args.containsKey("OVERFLOW")) {
            overflow = args["OVERFLOW"];
        } else {
            overflow = (lane >= 0 && lane < 4) ? _mmu->GetLaneOverflow(lane) : false;
        }

        _mmu->SetAutoFeed(lane, enable, overflow);
        SendOk(id);
    }

    void HandleTestMotor(int id, JsonObject args) {
        if (!_mmu) return;
        if(!args["lane"].isInt() || !args["pwm"].isInt()) {
             SendError(id, "BAD_ARGS", "Missing lane or pwm");
             return;
        }
        int lane = args["lane"];
        int pwm = args["pwm"];
        uint32_t duration = args["duration"].isInt() ? (uint32_t)args["duration"].asInt() : 1000;
        
        if(lane < 0 || lane >= 4) { SendError(id, "BAD_LANE", "0-3 only"); return; }
        
        _mmu->DiagnosticMotorControl(lane, pwm, duration);
        SendOk(id, "STARTED", "Diagnostic started");
    }


    void HandleCalibrate(int id, JsonObject args) {
         if (!_mmu) return;
         int lane = -1;
         if(args["lane"].isInt()) {
             lane = args["lane"];
         }
         
         CalibrateResult res = _mmu->CalibratePressure(lane);
         
         if (!res.ok) {
             SendError(id, "BUSY", res.error_msg);
             return;
         }
         
         WaitTX();
         int offset = 0;
         offset += snprintf(global_json_buf + offset, sizeof(global_json_buf) - offset, 
             "{\"id\":%d,\"cmd\":\"CALIBRATE\",\"ok\":true", id);
             
         if (lane == -1) {
             offset += snprintf(global_json_buf + offset, sizeof(global_json_buf) - offset, ",\"results\":{");
             uint16_t sensors = _mmu->GetSensorState();
             for (int i = 0; i < 4; i++) {
                 if (i > 0) offset += snprintf(global_json_buf + offset, sizeof(global_json_buf) - offset, ",");
                 
                 if (sensors & (1 << i)) {
                     // Busy
                     offset += snprintf(global_json_buf + offset, sizeof(global_json_buf) - offset, "\"%d\":{\"status\":\"busy\"}", i);
                 } else {
                     float p = _mmu->GetPressureZero(i);
                     int p_int = (int)p;
                     int p_dec = (int)((p - p_int) * 1000);
                     if(p_dec < 0) p_dec = -p_dec;
                     offset += snprintf(global_json_buf + offset, sizeof(global_json_buf) - offset, "\"%d\":{\"status\":\"calibrated\",\"value\":%d.%03d}", i, p_int, p_dec);
                 }
             }
             offset += snprintf(global_json_buf + offset, sizeof(global_json_buf) - offset, "}");
         } else {
             float p = res.value;
             int p_int = (int)p;
             int p_dec = (int)((p - p_int) * 1000);
             if(p_dec < 0) p_dec = -p_dec;
             offset += snprintf(global_json_buf + offset, sizeof(global_json_buf) - offset, ",\"lane\":%d,\"value\":%d.%03d", lane, p_int, p_dec);
         }
         
         offset += snprintf(global_json_buf + offset, sizeof(global_json_buf) - offset, "}\r\n");
         if (_transport) _transport->Write((const uint8_t*)global_json_buf, offset);
    }
    
    void HandleSetTolerance(int id, JsonObject args) {
         if (!_mmu) return;
         if(!args["value"].isFloat() && !args["value"].isInt()) {
             SendError(id, "BAD_ARGS", "Missing value float"); 
             return;
         }
         float val = args["value"];
         _mmu->SetPressureTolerance(val);
         _mmu->SetNeedToSave();
         SendOk(id);
    }

    void HandleSetGain(int id, LiteObject& args) {
        float gain = args["value"] | 250.0f;
        _mmu->SetPressureGain(gain);
        SendOk(id);
    }

    void HandleSetBoost(int id, LiteObject& args) {
        float pwm = args["value"] | 50.0f;
        _mmu->SetPressureMinPWM(pwm);
        SendOk(id);
    }

    void HandleSetPressureAdvanced(int id, LiteObject& args) {
        if (!_mmu) return;
        
        // Helper to check both cases
        auto get_float = [&](const char* lower, const char* upper) -> float {
            if (args[lower].isFloat()) return args[lower];
            if (args[upper].isFloat()) return args[upper];
            return -1e10f; // Sentinel
        };

        float gain = get_float("gain", "GAIN");
        if (gain > -1e9f) _mmu->SetPressureGain(gain);

        float offset = get_float("offset", "OFFSET");
        if (offset > -1e9f) _mmu->SetPressureOffset(offset);

        float bth = get_float("boost_thr", "BOOST_THR");
        if (bth > -1e9f) _mmu->SetBoostThreshold(bth);

        float bpw = get_float("boost_pwm", "BOOST_PWM");
        if (bpw > -1e9f) _mmu->SetBoostPWM(bpw);

        float dz = get_float("deadzone", "DEADZONE");
        if (dz > -1e9f) _mmu->SetRetractDeadzone(dz);

        float mpw = get_float("min_pwm", "MIN_PWM");
        if (mpw > -1e9f) _mmu->SetPressureMinPWM(mpw);

        float tol = get_float("tol", "TOL");
        if (tol > -1e9f) _mmu->SetPressureTolerance(tol);

        if (args["boost_time"].isInt()) _mmu->SetBoostTime((uint32_t)args["boost_time"]);
        else if (args["BOOST_TIME"].isInt()) _mmu->SetBoostTime((uint32_t)args["BOOST_TIME"]);

        SendOk(id);
    }

    void HandleSetMovePID(int id, LiteObject& args) {
        if (!_mmu) return;
        
        auto get_float = [&](const char* lower, const char* upper) -> float {
            if (args[lower].isFloat()) return args[lower];
            if (args[upper].isFloat()) return args[upper];
            if (args[lower].isInt()) return (float)args[lower].asInt();
            if (args[upper].isInt()) return (float)args[upper].asInt();
            return -1e10f;
        };

        float p = get_float("p", "P");
        float i = get_float("i", "I");
        float d = get_float("d", "D");
        float zero = get_float("zero", "ZERO");

        _mmu->SetMovePID(p, i, d, zero);
        SendOk(id);
    }

    void HandleSaveParams(int id, LiteObject& args) {
        if (!_mmu) return;
        _mmu->SaveSettings();
        SendOk(id, "SAVED", "Parameters saved to flash");
    }

    void HandleLoadParams(int id, LiteObject& args) {
        if (!_mmu) return;
        _mmu->LoadSettings();
        SendOk(id, "LOADED", "Parameters loaded from flash");
    }

    void HandleFeedToExtruder(int id, LiteObject& args) {
        if (!_mmu) return;
        int lane = args["lane"] | (args["LANE"] | -1);
        float spd = args["speed"].isFloat() ? args["speed"] : (args["SPEED"].isFloat() ? args["SPEED"] : 30.0f);
        float max = args["max_mm"].isFloat() ? args["max_mm"] : (args["MAX_MM"].isFloat() ? args["MAX_MM"] : 250.0f);
        float pthr = args["pressure_thr"].isFloat() ? args["pressure_thr"] : (args["PRESSURE_THR"].isFloat() ? args["PRESSURE_THR"] : 0.20f);
        uint32_t stall = args["stall_ms"].isInt() ? (uint32_t)args["stall_ms"] : (args["STALL"].isInt() ? (uint32_t)args["STALL"] : 300);

        if (lane < 0 || lane >= 4) {
            SendError(id, "BAD_LANE", "Lane 0-3 required");
            return;
        }

        _mmu->StartFeedToExtruder(lane, spd, max, pthr, stall, id);
        SendOk(id, "MOVING", "FTE started");
    }

    void ProcessPacket(char* json_str) {
        // Guard against null or empty input
        if (!json_str || json_str[0] == '\0') {
            return; // Silently ignore empty packets
        }
        
        // RS485 noise recovery: find first '{' in the received data.
        // RS485 transceiver startup noise corrupts the first few bytes
        // of each frame, so we skip any garbage before the JSON object.
        char* json_start = strchr(json_str, '{');
        if (!json_start) {
            return; // No JSON start found — discard (pure noise)
        }
        json_str = json_start; // Parse from '{' onwards
        
        if (_mmu) _mmu->UpdateConnectivity(true);
        
        doc.clear();
        DeserializationResult error = deserializeJson(doc, json_str);

        if (error) {
            // Debug: Echo back what was received (truncated to 100 chars)
            WaitTX();
            static char err_buf[256];
            char truncated[101];
            strncpy(truncated, json_str, 100);
            truncated[100] = '\0';
            snprintf(err_buf, sizeof(err_buf), 
                "{\"ok\":false,\"msg\":\"JSON Parse Error\",\"received\":\"%s\",\"error\":\"%s\"}\n",
                truncated, error.c_str());
            I_MMU_Transport* t = _current_response_transport ? _current_response_transport : _transport;
            if (t) t->Write((const uint8_t*)err_buf, strlen(err_buf));
            return;
        }

        int id = doc["id"] | 0;
        const char* cmd = doc["cmd"];
        
        // Get args from nested object, or use root if not present
        LiteObject* argsPtr = doc["args"].getObject();
        LiteObject& args = argsPtr ? *argsPtr : doc.root();

        if (!cmd) return;

        if (strcasecmp(cmd, "PING") == 0) HandlePing(id, args);
        else if (strcasecmp(cmd, "STATUS") == 0) HandleStatus(id, args);
        else if (strcasecmp(cmd, "GET_SENSORS") == 0) HandleGetSensors(id, args);
        else if (strcasecmp(cmd, "MOVE") == 0) HandleMove(id, args);
        else if (strcasecmp(cmd, "STOP") == 0) HandleStop(id, args);
        else if (strcasecmp(cmd, "SELECT_LANE") == 0) HandleSelectLane(id, args);
        else if (strcasecmp(cmd, "SET_AUTO_FEED") == 0) HandleSetAutoFeed(id, args);
        else if (strcasecmp(cmd, "SET_PRESSURE_ADVANCED") == 0 || strcasecmp(cmd, "SET_PA") == 0) HandleSetPressureAdvanced(id, args);
        else if (strcasecmp(cmd, "SET_MOVE_PID") == 0) HandleSetMovePID(id, args);
        else if (strcasecmp(cmd, "CALIBRATE") == 0) HandleCalibrate(id, args);
        else if (strcasecmp(cmd, "TEST_MOTOR") == 0) HandleTestMotor(id, args);
        else if (strcasecmp(cmd, "FEED_TO_EXTRUDER") == 0) HandleFeedToExtruder(id, args);
        else if (strcasecmp(cmd, "SAVE_PARAMS") == 0) HandleSaveParams(id, args);
        else if (strcasecmp(cmd, "LOAD_PARAMS") == 0) HandleLoadParams(id, args);
        else {
            SendError(id, "UNKNOWN_COMMAND", cmd);
        }
    }

    void Init(MMU_Logic* mmu, I_MMU_Transport* main_transport, I_MMU_Transport* aux_transport) {
        _mmu = mmu;
        _transport = main_transport;
        _aux_transport = aux_transport;
        const char* startup = "{\"event\":\"STARTUP\",\"msg\":\"KlipperCLI Ready\"}\r\n";
        if (_transport) _transport->Write((const uint8_t*)startup, strlen(startup));
        if (_aux_transport) _aux_transport->Write((const uint8_t*)startup, strlen(startup));
    }
    
    void Run() {
        // Process Primary Channel
        if (_transport) {
            int bytes_to_read = 64; 
            while (_transport->Available() > 0 && bytes_to_read-- > 0) {
                int b = _transport->Read();
                if (b < 0) break;
                
                if (b != '\r' && b != '\n' && (b < 32 || b > 126)) {
                    continue; 
                }
                
                if (b == '\n' || b == '\r') {
                    if (rx_idx > 0) {
                        rx_buffer[rx_idx] = '\0';
                        last_activity_time = millis();
                        _current_response_transport = _transport;
                        ProcessPacket(rx_buffer);
                    }
                    rx_idx = 0;
                } else {
                    if (rx_idx < (int)sizeof(rx_buffer) - 1) {
                        rx_buffer[rx_idx++] = (char)b;
                    } else {
                        rx_idx = 0; 
                    }
                }
            }
        }

        // Process Auxiliary Channel
        if (_aux_transport) {
            int bytes_to_read = 64; 
            while (_aux_transport->Available() > 0 && bytes_to_read-- > 0) {
                int b = _aux_transport->Read();
                if (b < 0) break;
                
                if (b != '\r' && b != '\n' && (b < 32 || b > 126)) {
                    continue; 
                }
                
                if (b == '\n' || b == '\r') {
                    if (rx_idx_aux > 0) {
                        rx_buffer_aux[rx_idx_aux] = '\0';
                        last_activity_time = millis();
                        _current_response_transport = _aux_transport;
                        ProcessPacket(rx_buffer_aux);
                    }
                    rx_idx_aux = 0;
                } else {
                    if (rx_idx_aux < (int)sizeof(rx_buffer_aux) - 1) {
                        rx_buffer_aux[rx_idx_aux++] = (char)b;
                    } else {
                        rx_idx_aux = 0; 
                    }
                }
            }
        }
        

        // FTE Notification
        if (_mmu && millis() - last_fte_check > 50) {
            last_fte_check = millis();
            float dist_mm = 0.0f;
            auto reason = _mmu->GetFTEResult(dist_mm);
            if (reason != MMU_Logic::StopReason::none) {
                const char* stop_str =
                    (reason == MMU_Logic::StopReason::pressure) ? "pressure" :
                    (reason == MMU_Logic::StopReason::stall)    ? "stall"    :
                    (reason == MMU_Logic::StopReason::distance) ? "distance" : "unknown";

                int d_int = (int)dist_mm;
                int d_dec = (int)((dist_mm - d_int) * 100);
                if (d_dec < 0) d_dec = -d_dec;

                int len = snprintf(global_json_buf, sizeof(global_json_buf),
                    "{\"id\":%d,\"ok\":true,\"cmd\":\"FEED_TO_EXTRUDER\","
                    "\"stopped_by\":\"%s\",\"dist_moved_mm\":%d.%02d}\r\n",
                    _mmu->GetFTECmdId(), stop_str, d_int, d_dec);
                if (_transport) _transport->Write((const uint8_t*)global_json_buf, len);
                if (_aux_transport) _aux_transport->Write((const uint8_t*)global_json_buf, len);

                _mmu->ClearFTEResult();
            }
        }
    }
    
    bool IsConnected() {
        bool main_conn = _transport && _transport->IsConnected();
        bool aux_conn = _aux_transport && _aux_transport->IsConnected();
        return main_conn || aux_conn;
    }
    
    // Returns true if no serial activity for the specified duration (ms)
    bool IsSerialIdle(uint32_t idle_ms) {
        return (millis() - last_activity_time) > idle_ms;
    }
}

// Global wrapper for cross-module access (used by MMU_Logic for smart save timing)
bool KlipperCLI_IsSerialIdle(uint32_t idle_ms) {
    return KlipperCLI::IsSerialIdle(idle_ms);
}
