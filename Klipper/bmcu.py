# BMCU Klipper "extras" module (raw-serial JSON transport)
#
# Drop this file into: ~/klipper/klippy/extras/bmcu.py
#
# Minimal printer.cfg:
#
#   [bmcu]
#   serial: /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0
#   baud: 115200
#
# Optional tuning keys:
#   timeout, poll_interval, read_interval, debug,
#   line_ending, tx_rx_mode,
#   connect_flush, connect_flush_delay, connect_drain_s,
#   connect_set_dtr, connect_dtr_low, connect_dtr_settle,
#   connect_set_rts, connect_rts_low,
#   default_speed, max_move_mm, max_speed,
#   default_lane_feed_mm, default_lane_retract_mm,
#   supported_cmds
#
# Firmware surface (per KlipperCLI.cpp with LiteJSON):
#   PING, STATUS, GET_SENSORS, MOVE, STOP, SELECT_LANE,
#   SET_AUTO_FEED
#
# LiteJSON Firmware Limits:
#   - MAX_KEYS = 8 (max key-value pairs per JSON object)
#   - MAX_STRING_LEN = 32 (strings truncated beyond this)
#   - MAX_ARRAY_SIZE = 4 (max elements per array)
#   - MAX_NESTING = 3 (max nesting depth)

import logging
import json
import time

import serial  # pyserial


class BMCU:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')

        # -----------------------------
        # Config
        # -----------------------------
        self.serial_port = config.get('serial')
        self.baud = config.getint('baud', 115200)  # Reduced from 250000
        self.timeout = config.getfloat('timeout', 0.1)

        # Gentler defaults
        self.poll_interval = config.getfloat('poll_interval', 10.0)
        self.read_interval = config.getfloat('read_interval', 0.05)

        # Quiet by default; turn on in cfg when needed
        self.debug = config.getboolean('debug', False)

        # Line ending for JSON packets
        self.line_ending = self._normalize_line_ending(config.get('line_ending', 'LF'))

        # TX/RX mode: "full" or "halfduplex"
        self.tx_rx_mode = (config.get("tx_rx_mode", "halfduplex") or "halfduplex").lower()

        # Connect hygiene
        self.connect_flush = config.getboolean('connect_flush', True)
        self.connect_flush_delay = config.getfloat('connect_flush_delay', 0.2)
        self.connect_drain_s = config.getfloat('connect_drain_s', 0.2)

        # DTR/RTS behavior (baked to your working defaults)
        self.connect_set_dtr = config.getboolean('connect_set_dtr', False)
        self.connect_dtr_low = config.getboolean('connect_dtr_low', True)
        self.connect_dtr_settle = config.getfloat('connect_dtr_settle', 0.10)

        self.connect_set_rts = config.getboolean('connect_set_rts', True)
        self.connect_rts_low = config.getboolean('connect_rts_low', True)

        # Motion limits
        self.default_speed = config.getfloat('default_speed', 20.0)
        self.max_move_mm = config.getfloat('max_move_mm', 1200.0)
        self.max_speed = config.getfloat('max_speed', 200.0)

        self.default_lane_feed_mm = config.getfloat('default_lane_feed_mm', 60.0)
        self.default_lane_retract_mm = config.getfloat('default_lane_retract_mm', 60.0)

        # Optional allowlist of commands
        supported = config.get('supported_cmds', '').strip()
        if supported:
            self.supported_cmds = {c.strip() for c in supported.split(',') if c.strip()}
        else:
            self.supported_cmds = set()

        # Pressure Advanced config
        self.p_gain = config.getfloat('p_gain', None)
        self.p_offset = config.getfloat('p_offset', None)
        self.p_boost_thr = config.getfloat('p_boost_thr', None)
        self.p_boost_pwm = config.getfloat('p_boost_pwm', None)
        self.p_boost_time = config.getint('p_boost_time', None)
        self.p_deadzone = config.getfloat('p_deadzone', None)
        self.p_min_pwm = config.getfloat('p_min_pwm', None)
        self.p_tol = config.getfloat('p_tol', None)

        # Move PID config
        self.m_p = config.getfloat('m_p', None)
        self.m_i = config.getfloat('m_i', None)
        self.m_d = config.getfloat('m_d', None)
        self.m_zero = config.getint('m_zero', None)

        # -----------------------------
        # State
        # -----------------------------
        self.ser = None
        self.is_connected = False
        self._buf = ""
        self._last_connect_attempt = 0.0
        self._connect_backoff = 1.0

        self.lanes = None
        self.last_rx = None
        self.last_rx_by_id = {}

        # Have we already kicked a STATUS after STARTUP?
        self._did_startup_status = False
        self._config_sent = False

        self.fte_stopped_by = ""
        self.fte_dist_mm = 0.0

        # Events
        self.printer.register_event_handler("klippy:shutdown", self._handle_shutdown)
        self.printer.register_event_handler("klippy:connect", self._handle_connect)

        # Expose BMCU lane data to Moonraker and Klipper clients via
        # status callbacks.  We attempt to register a callback with
        # the Moonraker webhooks API, but also implement a
        # get_status() method for Klipper's native status polling.
        # Some installs may not have a webhooks component or the
        # register_status_callback method; in those cases get_status()
        # will still expose lane data.
        try:
            self.webhooks = self.printer.lookup_object('webhooks')
            if hasattr(self.webhooks, 'register_status_callback'):
                # Wrap get_status() so that the callback returns a
                # dictionary keyed by the module name.  Moonraker will
                # merge this into the printer status under 'bmcu'.
                def _bmcu_status_callback(eventtime):
                    return {'bmcu': self.get_status(eventtime)}
                self.webhooks.register_status_callback(_bmcu_status_callback)
        except Exception:
            # Gracefully ignore if webhooks or its status API is unavailable.
            pass
        # G-code commands
        self._register_gcode()

        # Timers
        self._read_timer = None
        self._poll_timer = None

        if self.debug:
            logging.info("BMCU: initialized (deferred connect) serial=%s baud=%d",
                         self.serial_port, self.baud)

    # -----------------------------
    # G-code registration
    # -----------------------------
    def _register_gcode(self):
        gc = self.gcode
        gc.register_command("BMCU_CAPS", self.cmd_BMCU_CAPS)
        gc.register_command("BMCU_PING", self.cmd_BMCU_PING)
        gc.register_command("BMCU_STATUS", self.cmd_BMCU_STATUS)
        gc.register_command("BMCU_GET_SENSORS", self.cmd_BMCU_GET_SENSORS)
        gc.register_command("BMCU_STOP", self.cmd_BMCU_STOP)
        gc.register_command("BMCU_SELECT_LANE", self.cmd_BMCU_SELECT_LANE)
        gc.register_command("BMCU_SET_AUTO_FEED", self.cmd_BMCU_SET_AUTO_FEED)
        gc.register_command("BMCU_MOVE", self.cmd_BMCU_MOVE)
        gc.register_command("BMCU_FEED", self.cmd_BMCU_FEED)
        gc.register_command("BMCU_CALL", self.cmd_BMCU_CALL)
        gc.register_command("BMCU_LANE_FEED", self.cmd_BMCU_LANE_FEED)
        gc.register_command("BMCU_LANE_RETRACT", self.cmd_BMCU_LANE_RETRACT)
        
        # Extended Commands
        gc.register_command("BMCU_FEED_TO_EXTRUDER", self.cmd_BMCU_FEED_TO_EXTRUDER)
        gc.register_command("BMCU_CALIBRATE", self.cmd_BMCU_CALIBRATE)
        gc.register_command("BMCU_SET_PA", self.cmd_BMCU_SET_PA)
        gc.register_command("BMCU_SET_MOVE_PID", self.cmd_BMCU_SET_MOVE_PID)
        gc.register_command("BMCU_TEST_MOTOR", self.cmd_BMCU_TEST_MOTOR)
        gc.register_command("BMCU_SAVE_PARAMS", self.cmd_BMCU_SAVE_PARAMS)
        gc.register_command("BMCU_LOAD_PARAMS", self.cmd_BMCU_LOAD_PARAMS)

    # -----------------------------
    # Timers
    # -----------------------------
    def _handle_poll(self, eventtime):
        if not self.is_connected:
            self._maybe_connect(eventtime)
            return eventtime + max(self.poll_interval, 0.5)

        # Lightweight health check: PING only
        self._send_pkt("PING", {}, note="poll")

        if self.tx_rx_mode == 'halfduplex':
            self._pump_rx(0.05)

        return eventtime + self.poll_interval

    def _handle_read(self, eventtime):
        if not self.is_connected or self.ser is None:
            return eventtime + max(self.read_interval, 0.1)
        try:
            data = self.ser.read(4096)
            if data:
                if self.debug:
                    logging.info("BMCU RX chunk: len=%d hex=%s...",
                                 len(data), data[:16].hex())
                self._buf += data.decode('utf-8', errors='ignore')

                while '\n' in self._buf or '\r' in self._buf:
                    idx_n = self._buf.find('\n')
                    idx_r = self._buf.find('\r')
                    idxs = [i for i in (idx_n, idx_r) if i != -1]
                    if not idxs:
                        break
                    idx = min(idxs)
                    line = self._buf[:idx]
                    self._buf = self._buf[idx+1:]
                    line = line.strip()
                    if line:
                        self._process_line(line)

                if len(self._buf) > 4096:
                    logging.error("BMCU: Buffer overflow (>4k), clearing")
                    self._buf = ""
        except Exception as e:
            logging.error("BMCU: read error: %s", e)
            self._disconnect()
        return eventtime + self.read_interval

    # -----------------------------
    # Connect / disconnect
    # -----------------------------
    def _maybe_connect(self, eventtime):
        if (eventtime - self._last_connect_attempt) < self._connect_backoff:
            return
        self._last_connect_attempt = eventtime
        try:
            self._connect()
            self._connect_backoff = 1.0
        except Exception as e:
            if self.debug:
                logging.warning("BMCU: connect failed: %s", e)
            self._connect_backoff = min(self._connect_backoff * 2.0, 30.0)

    def _normalize_line_ending(self, val):
        if val is None:
            return "\n"
        v = str(val).strip().upper()
        if v in ("LF", "\\N", "N", "NEWLINE"):
            return "\n"
        if v in ("CRLF", "\\R\\N", "RN", "WINDOWS"):
            return "\r\n"
        if v in ("CR", "\\R"):
            return "\r"
        raw = str(val)
        if "\r\n" in raw:
            return "\r\n"
        if "\n" in raw:
            return "\n"
        if "\r" in raw:
            return "\r"
        return "\n"

    def _connect(self):
        self.ser = serial.Serial(self.serial_port, self.baud, timeout=self.timeout)
        try:
            if self.connect_set_dtr:
                self.ser.dtr = (False if self.connect_dtr_low else True)
        except Exception:
            pass
        try:
            if self.connect_set_rts:
                self.ser.rts = (False if self.connect_rts_low else True)
        except Exception:
            pass
        try:
            time.sleep(max(0.0, float(self.connect_dtr_settle)))
        except Exception:
            pass

        if self.connect_flush:
            try:
                self.ser.reset_input_buffer()
            except Exception:
                pass
            try:
                self.ser.reset_output_buffer()
            except Exception:
                pass
            try:
                time.sleep(max(0.0, float(self.connect_flush_delay)))
            except Exception:
                pass
            try:
                old_to = self.ser.timeout
                self.ser.timeout = 0.0
                t_end = time.time() + max(0.0, float(self.connect_drain_s))
                while time.time() < t_end:
                    data = self.ser.read(4096)
                    if not data:
                        break
                self.ser.timeout = old_to
            except Exception:
                pass

        try:
            self.reactor.pause(self.reactor.monotonic() + 0.12)
        except Exception:
            pass
        self.is_connected = True
        self._did_startup_status = False
        if self._read_timer is None:
            self._read_timer = self.reactor.register_timer(self._handle_read, self.reactor.NOW + self.read_interval)
        if self.debug:
            logging.info("BMCU: connected on %s @ %d", self.serial_port, self.baud)

    def _disconnect(self):
        self.is_connected = False
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None

    def _handle_connect(self):
        if self._poll_timer is None:
            self._poll_timer = self.reactor.register_timer(self._handle_poll, self.reactor.NOW + 1.0)

    def _handle_shutdown(self):
        try:
            if self.is_connected and self.ser:
                self.ser.write(b'{"cmd":"STOP"}\r\n')
                self.ser.flush()
        except Exception:
            pass
        self._disconnect()

    # -----------------------------
    # Protocol helpers
    # -----------------------------
    def _pump_rx(self, budget_s: float = 0.05) -> None:
        if not self.is_connected or self.ser is None:
            return
        try:
            old_to = self.ser.timeout
            self.ser.timeout = 0.0
        except Exception:
            old_to = None
        t_end = time.time() + max(0.0, float(budget_s))
        try:
            while time.time() < t_end:
                data = self.ser.read(4096)
                if not data:
                    break
                self._buf += data.decode('utf-8', errors='ignore')
                while '\n' in self._buf or '\r' in self._buf:
                    idx_n = self._buf.find('\n')
                    idx_r = self._buf.find('\r')
                    idxs = [i for i in (idx_n, idx_r) if i != -1]
                    if not idxs:
                        break
                    idx = min(idxs)
                    line = self._buf[:idx]
                    self._buf = self._buf[idx+1:]
                    line = line.strip()
                    if line:
                        self._process_line(line)
        except Exception:
            pass
        finally:
            try:
                if old_to is not None:
                    self.ser.timeout = old_to
            except Exception:
                pass

    def _next_id(self) -> int:
        # Match your UI behavior (0..9999)
        return int(time.time() * 1000) % 10000

    def _cmd_allowed(self, cmd):
        return (not self.supported_cmds) or (cmd in self.supported_cmds)

    def _send_pkt(self, cmd, args, note=None):
        if not self.is_connected or self.ser is None:
            return False, None
        if not self._cmd_allowed(cmd):
            return False, None
        pkt_id = self._next_id()
        pkt = {"id": pkt_id, "cmd": cmd, "args": args}
        try:
            msg = json.dumps(pkt) + self.line_ending
            if self.debug:
                logging.info("BMCU TX(%s): %s", note or "pkt", msg.strip())
            self.ser.write(msg.encode('utf-8'))
            return True, pkt_id
        except Exception as e:
            logging.error("BMCU: send error: %s", e)
            self._disconnect()
            return False, None

    def _process_line(self, line):
        # Junk-tolerant: handle 'UN{...}' and similar noise
        if not isinstance(line, str):
            line = str(line)
        raw_line = line

        # Strip leading junk before first { or [
        idx_obj = line.find("{")
        idx_arr = line.find("[")
        idxs = [i for i in (idx_obj, idx_arr) if i != -1]
        if idxs:
            start = min(idxs)
            if start > 0:
                line = line[start:]

        # Trim trailing junk after last } or ]
        last_obj = line.rfind("}")
        last_arr = line.rfind("]")
        last_idxs = [i for i in (last_obj, last_arr) if i != -1]
        if last_idxs:
            end = max(last_idxs) + 1
            if end < len(line):
                line = line[:end]

        if self.debug:
            logging.info("BMCU RX: %s", line[:300])

        try:
            pkt = json.loads(line)
            self.last_rx = pkt

            # Special: firmware startup event
            if isinstance(pkt, dict) and pkt.get("event") == "STARTUP":
                if self.debug:
                    logging.info("BMCU: got STARTUP event")
                self._config_sent = False
                if not self._did_startup_status:
                    self._did_startup_status = True
                    self._send_pkt("STATUS", {}, note="startup_status")
                return

            if not self._config_sent and isinstance(pkt, dict) and "cmd" in pkt:
                self._config_sent = True
                
                args_pa = {}
                if self.p_gain is not None: args_pa["gain"] = self.p_gain
                if self.p_offset is not None: args_pa["offset"] = self.p_offset
                if self.p_boost_thr is not None: args_pa["boost_thr"] = self.p_boost_thr
                if self.p_boost_pwm is not None: args_pa["boost_pwm"] = self.p_boost_pwm
                if self.p_boost_time is not None: args_pa["boost_time"] = self.p_boost_time
                if self.p_deadzone is not None: args_pa["deadzone"] = self.p_deadzone
                if self.p_min_pwm is not None: args_pa["min_pwm"] = self.p_min_pwm
                if self.p_tol is not None: args_pa["tol"] = self.p_tol
                if args_pa:
                    if self.debug: logging.info("BMCU: pushing startup SET_PA")
                    self._send_pkt("SET_PA", args_pa, note="startup_pa")
                
                args_pid = {}
                if self.m_p is not None: args_pid["p"] = self.m_p
                if self.m_i is not None: args_pid["i"] = self.m_i
                if self.m_d is not None: args_pid["d"] = self.m_d
                if self.m_zero is not None: args_pid["zero"] = self.m_zero
                if args_pid:
                    if self.debug: logging.info("BMCU: pushing startup SET_MOVE_PID")
                    self._send_pkt("SET_MOVE_PID", args_pid, note="startup_pid")

            if isinstance(pkt, dict) and pkt.get("cmd") == "FEED_TO_EXTRUDER" and "stopped_by" in pkt:
                self.fte_stopped_by = str(pkt.get("stopped_by", "unknown"))
                self.fte_dist_mm = float(pkt.get("dist_moved_mm", 0.0))

            if isinstance(pkt, dict) and "id" in pkt:
                try:
                    rx_id = int(pkt["id"])
                    self.last_rx_by_id[rx_id] = pkt
                    if len(self.last_rx_by_id) > 50:
                        for k in list(self.last_rx_by_id.keys())[:10]:
                            self.last_rx_by_id.pop(k, None)
                except Exception:
                    pass

            if isinstance(pkt, dict) and "lanes" in pkt:
                self.lanes = pkt.get("lanes")

        except json.JSONDecodeError:
            if self.debug:
                logging.warning("BMCU: non-json line: %r", raw_line[:240])
        except Exception as e:
            logging.error("BMCU: parse error: %s", e)

    def _clamp(self, v, lo, hi):
        return max(lo, min(hi, v))

    # -----------------------------
    # Wait for reply helper
    # -----------------------------
    def _wait_for_reply(self, gcmd, pkt_id, wait_s):
        if not wait_s or wait_s <= 0:
            return
        wait_s = max(0.0, min(wait_s, 5.0))
        end = self.reactor.monotonic() + wait_s
        while self.reactor.monotonic() < end:
            self.reactor.pause(self.reactor.monotonic() + 0.05)
            if pkt_id is not None and pkt_id in self.last_rx_by_id:
                gcmd.respond_info("BMCU: reply: " + json.dumps(self.last_rx_by_id[pkt_id])[:1200])
                return
        gcmd.respond_info(f"BMCU: no reply within {wait_s:.2f}s (id={pkt_id})")

    # -----------------------------
    # Status callback / reporting
    # -----------------------------
    def get_status(self, eventtime=None):
        """
        Provide current lane state for status queries.

        This method returns a dictionary of fields that will be
        merged into printer.bmcu by Klipper's status machinery.
        It should contain only module‑specific values; Klipper will
        handle prefixing with the module name.  If the BMCU has not
        yet reported any lanes (STATUS frame not received), return an
        empty list.

        The eventtime parameter is ignored but preserved for API
        compatibility with Moonraker and Klipper.
        """
        lanes = self.lanes if isinstance(self.lanes, list) else []
        return {
            'lanes': lanes,
            'fte_stopped_by': self.fte_stopped_by,
            'fte_dist_mm': self.fte_dist_mm
        }

    # Preserve the old _get_status for backward compatibility; delegate to get_status.
    def _get_status(self, eventtime=None):
        """
        Backward‑compatibility wrapper that returns a dict keyed by
        'bmcu'.  This is retained for older versions of Moonraker
        which might still invoke the old callback signature.
        """
        return {'bmcu': self.get_status(eventtime)}

    # -----------------------------
    # GCODE COMMANDS
    # -----------------------------
    def cmd_BMCU_CAPS(self, gcmd):
        if not self.supported_cmds:
            gcmd.respond_info("BMCU_CAPS: all commands allowed (no allowlist).")
        else:
            gcmd.respond_info("Allowed commands: " + ", ".join(sorted(self.supported_cmds)))

    def cmd_BMCU_PING(self, gcmd):
        wait_s = gcmd.get_float("WAIT", 0.0)
        ok, pkt_id = self._send_pkt("PING", {}, note="ping")
        gcmd.respond_info(f"Pinging... id={pkt_id}")
        if ok:
            self._wait_for_reply(gcmd, pkt_id, wait_s)

    def cmd_BMCU_STATUS(self, gcmd):
        if self.lanes is None:
            gcmd.respond_info("No cached STATUS yet.")
            return
        out = ["Cached STATUS:"]
        for i, lane in enumerate(self.lanes):
            out.append(f"  Lane {i}: {lane}")
        gcmd.respond_info("\n".join(out))

    def cmd_BMCU_GET_SENSORS(self, gcmd):
        wait_s = gcmd.get_float("WAIT", 0.5)
        ok, pkt_id = self._send_pkt("GET_SENSORS", {}, note="sensors")
        gcmd.respond_info(f"GET_SENSORS sent id={pkt_id}")
        if ok:
            self._wait_for_reply(gcmd, pkt_id, wait_s)

    def cmd_BMCU_STOP(self, gcmd):
        ok, pkt_id = self._send_pkt("STOP", {}, note="stop")
        gcmd.respond_info(f"STOP sent id={pkt_id}")

    def cmd_BMCU_SELECT_LANE(self, gcmd):
        lane = gcmd.get_int("LANE", 0)
        wait_s = gcmd.get_float("WAIT", 0.3)
        ok, pkt_id = self._send_pkt("SELECT_LANE", {"lane": lane}, note="select")
        gcmd.respond_info(f"SELECT_LANE lane={lane} id={pkt_id}")
        if ok:
            self._wait_for_reply(gcmd, pkt_id, wait_s)

    def cmd_BMCU_SET_AUTO_FEED(self, gcmd):
        lane = gcmd.get_int("LANE", 0)
        enable = bool(gcmd.get_int("ENABLE", 1))
        overflow_raw = gcmd.get_int("OVERFLOW", None)
        
        args = {"lane": lane, "enable": enable}
        if overflow_raw is not None:
            args["overflow"] = bool(overflow_raw)
            
        ok, pkt_id = self._send_pkt("SET_AUTO_FEED", args, note="auto_feed")
        gcmd.respond_info(f"AUTO_FEED lane={lane} enable={enable} overflow={overflow_raw}")

    def cmd_BMCU_MOVE(self, gcmd):
        axis = gcmd.get("AXIS", "0")
        dist = gcmd.get_float("DIST", 0.0)
        speed = gcmd.get_float("SPEED", self.default_speed)
        wait_s = gcmd.get_float("WAIT", 0.0)
        dist = self._clamp(dist, -self.max_move_mm, self.max_move_mm)
        speed = self._clamp(speed, -self.max_speed, self.max_speed)
        ok, pkt_id = self._send_pkt(
            "MOVE",
            {"axis": str(axis), "dist_mm": float(dist), "speed": float(speed)},
            note="move"
        )
        gcmd.respond_info(f"MOVE axis={axis} dist={dist} speed={speed}")
        if ok:
            self._wait_for_reply(gcmd, pkt_id, wait_s)

    def cmd_BMCU_FEED(self, gcmd):
        mm = gcmd.get_float("MM", self.default_lane_feed_mm)
        speed = gcmd.get_float("SPEED", self.default_speed)
        ok, pkt_id = self._send_pkt(
            "MOVE",
            {"axis": "FEED", "dist_mm": float(mm), "speed": float(speed)}, note="feed")
        gcmd.respond_info(f"FEED {mm}mm @ {speed}")



    def cmd_BMCU_CALL(self, gcmd):
        cmd = gcmd.get("CMD")
        args_json = gcmd.get("ARGS", "{}")
        wait_s = gcmd.get_float("WAIT", 0.0)
        try:
            args = json.loads(args_json)
        except Exception:
            gcmd.respond_info("Invalid ARGS JSON")
            return
        ok, pkt_id = self._send_pkt(cmd, args, note="call")
        gcmd.respond_info(f"CALL {cmd} id={pkt_id}")
        if ok:
            self._wait_for_reply(gcmd, pkt_id, wait_s)

    def cmd_BMCU_LANE_FEED(self, gcmd):
        lane = gcmd.get_int("LANE", 0)
        mm = gcmd.get_float("MM", self.default_lane_feed_mm)
        speed = gcmd.get_float("SPEED", self.default_speed)
        ok, _ = self._send_pkt(
            "MOVE",
            {"axis": str(lane), "dist_mm": float(mm), "speed": float(speed)}, note="lane_feed")
        gcmd.respond_info(f"Lane {lane} FEED {mm}mm")

    def cmd_BMCU_LANE_RETRACT(self, gcmd):
        lane = gcmd.get_int("LANE", 0)
        mm = gcmd.get_float("MM", self.default_lane_retract_mm)
        speed = gcmd.get_float("SPEED", self.default_speed)
        # Retract uses negative distance with positive speed (per firmware convention)
        ok, _ = self._send_pkt(
            "MOVE",
            {"axis": str(lane), "dist_mm": float(-mm), "speed": float(abs(speed))}, note="lane_retract")
        gcmd.respond_info(f"Lane {lane} RETRACT {mm}mm")

    def cmd_BMCU_FEED_TO_EXTRUDER(self, gcmd):
        lane = gcmd.get_int("LANE", -1)
        speed = gcmd.get_float("SPEED", 30.0)
        max_mm = gcmd.get_float("MAX_MM", 250.0)
        pressure_thr = gcmd.get_float("PRESSURE_THR", 0.20)
        stall_ms = gcmd.get_int("STALL", 300)
        
        calc_timeout = (max_mm / max(speed, 1.0)) + 10.0
        wait_s = gcmd.get_float("WAIT", calc_timeout)
        
        self.fte_stopped_by = ""
        self.fte_dist_mm = 0.0
        
        args = {"lane": lane, "speed": speed, "max_mm": max_mm, "pressure_thr": pressure_thr, "stall_ms": stall_ms}
        ok, pkt_id = self._send_pkt("FEED_TO_EXTRUDER", args, note="fte")
        gcmd.respond_info(f"FEED_TO_EXTRUDER lane={lane} id={pkt_id} (Timeout {wait_s:.1f}s)")
        
        if ok and wait_s > 0:
            end = self.reactor.monotonic() + wait_s
            while self.reactor.monotonic() < end:
                self.reactor.pause(self.reactor.monotonic() + 0.05)
                if self.fte_stopped_by != "":
                    gcmd.respond_info(f"FTE Completed. Stopped by: {self.fte_stopped_by}. Distance: {self.fte_dist_mm}mm")
                    return
            gcmd.respond_info("FTE Timeout! Klipper didn't receive completion event.")

    def cmd_BMCU_CALIBRATE(self, gcmd):
        lane_raw = gcmd.get_int("LANE", None)
        wait_s = gcmd.get_float("WAIT", 2.0)
        
        args = {}
        if lane_raw is not None:
            args["lane"] = lane_raw
            
        ok, pkt_id = self._send_pkt("CALIBRATE", args, note="calibrate")
        gcmd.respond_info(f"CALIBRATE lane={lane_raw if lane_raw is not None else 'ALL'} id={pkt_id}")
        if ok:
            self._wait_for_reply(gcmd, pkt_id, wait_s)

    def cmd_BMCU_SET_PA(self, gcmd):
        args = {}
        for key in ["GAIN", "OFFSET", "BOOST_THR", "BOOST_PWM", "DEADZONE", "MIN_PWM", "TOL"]:
            val = gcmd.get_float(key, None)
            if val is not None: args[key.lower()] = val
            
        bt = gcmd.get_int("BOOST_TIME", None)
        if bt is not None: args["boost_time"] = bt
        
        if not args:
            gcmd.respond_info("SET_PA requires at least one parameter")
            return
            
        ok, pkt_id = self._send_pkt("SET_PA", args, note="set_pa")
        gcmd.respond_info(f"SET_PA args={args}")

    def cmd_BMCU_SET_MOVE_PID(self, gcmd):
        args = {}
        for key in ["P", "I", "D"]:
            val = gcmd.get_float(key, None)
            if val is not None: args[key.lower()] = val
            
        z = gcmd.get_int("ZERO", None)
        if z is not None: args["zero"] = z
        
        if not args:
            gcmd.respond_info("SET_MOVE_PID requires at least one parameter")
            return
            
        ok, pkt_id = self._send_pkt("SET_MOVE_PID", args, note="set_move_pid")
        gcmd.respond_info(f"SET_MOVE_PID args={args}")

    def cmd_BMCU_TEST_MOTOR(self, gcmd):
        lane = gcmd.get_int("LANE", 0)
        pwm = gcmd.get_int("PWM", 150)
        duration = gcmd.get_int("DURATION", 1000)
        
        args = {"lane": lane, "pwm": pwm, "duration": duration}
        ok, pkt_id = self._send_pkt("TEST_MOTOR", args, note="test_motor")
        gcmd.respond_info(f"TEST_MOTOR lane={lane} pwm={pwm} duration={duration}")

    def cmd_BMCU_SAVE_PARAMS(self, gcmd):
        wait_s = gcmd.get_float("WAIT", 0.5)
        ok, pkt_id = self._send_pkt("SAVE_PARAMS", {}, note="save_params")
        gcmd.respond_info(f"SAVE_PARAMS sent id={pkt_id}")
        if ok:
            self._wait_for_reply(gcmd, pkt_id, wait_s)

    def cmd_BMCU_LOAD_PARAMS(self, gcmd):
        wait_s = gcmd.get_float("WAIT", 0.5)
        ok, pkt_id = self._send_pkt("LOAD_PARAMS", {}, note="load_params")
        gcmd.respond_info(f"LOAD_PARAMS sent id={pkt_id}")
        if ok:
            self._wait_for_reply(gcmd, pkt_id, wait_s)


def load_config(config):
    return BMCU(config)
