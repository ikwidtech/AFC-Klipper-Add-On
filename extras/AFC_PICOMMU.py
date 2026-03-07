# Armored Turtle Automated Filament Control
#
# Config section:
#   [AFC_PICOMMU <name>]
#
# Required:
#   drive_stepper: <AFC_stepper name>   # feeder/drive motor (same role as HTLF)
#   selector_servo: <servo name>        # Klipper [servo ...] name (e.g. TINY_SERVO)
#
# Optional:
#   selector_home_mode: angle|lane|none (default: angle)
#   selector_home_angle: <deg>          (default: 90)
#   selector_home_lane: <lane name or index> (default: 1)
#     - accepts lane keys like "lane1" OR numeric lane index like 1
#   selector_dwell_ms: <ms>             (default: 250)
#   selector_detach_after: True|False   (default: True)
#   selector_angle_offset: <deg>        (default: 0)
#   cam_angle: <deg>                    (optional fallback if lane has no selector_angle)
#   MAX_ANGLE_MOVEMENT: <deg>           (default: 180)
#   auto_home_on_ready: True|False      (default: True)
#
# Per-lane options (in [AFC_lane <lane>]) supported by AFC_lane.py patch:
#   selector_angle: <deg>               # preferred for servo selectors
#   selector_servo: <servo name>        # optional per-lane servo override
#
#
import traceback
from configfile import error
from extras.AFC_utils import ERROR_STR
from extras.AFC_BoxTurtle import afcBoxTurtle


try:
    from extras.AFC_utils import ERROR_STR
except Exception:
    raise error("Error when trying to import AFC_utils.ERROR_STR\n{trace}".format(
        trace=traceback.format_exc()))

try:
    from extras.AFC_BoxTurtle import afcBoxTurtle
except Exception:
    raise error(ERROR_STR.format(import_lib="AFC_BoxTurtle", trace=traceback.format_exc()))


class AFC_PICOMMU(afcBoxTurtle):
    def __init__(self, config):
        super().__init__(config)

        # Keep AFC naming conventions
        self.type = config.get('type', 'PICOMMU')

        # Drive stepper (filament motion core) - SAME AS HTLF
        self.drive_stepper = config.get("drive_stepper")
        self.drive_stepper_obj = None

        # Servo selector (replaces selector_stepper)
        self.selector_servo = config.get("selector_servo")
        self.selector_servo_obj = None

        # Home strategy (servo-based, no pins)
        self.selector_home_mode = config.get('selector_home_mode', 'angle').lower()
        self.selector_home_angle = config.getfloat('selector_home_angle', 90.0)
        # Accept lane key ("lane1") or index ("1")
        self.selector_home_lane = config.get('selector_home_lane', '1')

        # Servo timing/behavior
        self.selector_dwell_ms = config.getint('selector_dwell_ms', 250)
        self.selector_detach_after = config.getboolean('selector_detach_after', True)
        self.selector_angle_offset = config.getfloat('selector_angle_offset', 0.0)

        # Cam math fallback (optional; matches HTLF style)
        self.cam_angle = config.getint('cam_angle', 0)  # 0 means disabled unless used
        self.MAX_ANGLE_MOVEMENT = config.getint('MAX_ANGLE_MOVEMENT', 180)

        # Whether to auto-home on klippy:ready (default True)
        self.auto_home_on_ready = config.getboolean('auto_home_on_ready', True)

        self.current_selected_lane = None

        # Defer homing until Klipper is ready
        self.printer.register_event_handler("klippy:ready", self._handle_ready)

    # ------------------------------------------------------------
    # Klippy connect / ready
    # ------------------------------------------------------------
    def handle_connect(self):
        # Validate drive stepper exists (same behavior/error string style as HTLF)
        try:
            self.drive_stepper_obj = self.printer.lookup_object(
                'AFC_stepper {}'.format(self.drive_stepper)
            )
        except Exception:
            error_string = (
                'Error: No config found for drive_stepper: {drive_stepper} in [AFC_PICOMMU {unit}]. '
                'Please make sure [AFC_stepper {drive_stepper}] section exists in your config'
            ).format(drive_stepper=self.drive_stepper, unit=self.name)
            raise error(error_string)

        # Validate servo exists (helps catch typos like selector_servo name mismatch)
        try:
            self.selector_servo_obj = self.printer.lookup_object(
                'servo {}'.format(self.selector_servo)
            )
        except Exception:
            raise error(
                "Error: No config found for selector_servo: {servo} in [AFC_PICOMMU {unit}]. "
                "Please make sure [servo {servo}] exists in your config".format(
                    servo=self.selector_servo, unit=self.name
                )
            )

        # Register HOME_UNIT (keeps AFC UX consistent)
        self.gcode.register_mux_command('HOME_UNIT', "UNIT", self.name, self.cmd_HOME_UNIT)

        # Run standard AFC unit connect checks (hub/extruder/buffer, lane registration, etc.)
        super().handle_connect()

        # Friendly status strings
        self.logo = '<span class=success--text>PICOMMU Ready\n</span>'
        self.logo_error = self.logo
        

        # IMPORTANT: do NOT call return_to_home() here; Klipper may not be ready for SET_SERVO.

    def _handle_ready(self):
        # Klipper is ready for gcode now; safe to SET_SERVO.
        if self.auto_home_on_ready:
            try:
                self.return_to_home()
            except Exception:
                self.logger.exception("AFC_PICOMMU: auto home on ready failed")
        self.prep_state = True
        self.afc.prep_done = True
        self.ready = True
        
    # ------------------------------------------------------------
    # Servo helpers
    # ------------------------------------------------------------
    def _set_selector_angle(self, angle):
        angle = float(angle) + float(self.selector_angle_offset)

        # Clamp safety
        if angle < 0.0:
            angle = 0.0
        if angle > 180.0:
            angle = 180.0

        self.gcode.run_script_from_command(
            "SET_SERVO SERVO={} ANGLE={:.3f}".format(self.selector_servo, angle)
        )

        if self.selector_dwell_ms and self.selector_dwell_ms > 0:
            self.gcode.run_script_from_command("G4 P{}".format(int(self.selector_dwell_ms)))

        if self.selector_detach_after:
            # Detach to reduce jitter/heating
            self.gcode.run_script_from_command(
                "SET_SERVO SERVO={} WIDTH=0".format(self.selector_servo)
            )

    def _cam_fallback_angle(self, lane_index):
        if not self.cam_angle:
            raise error(
                "AFC_PICOMMU: lane has no selector_angle and cam_angle is 0/undefined. "
                "Set selector_angle per lane, or set cam_angle + MAX_ANGLE_MOVEMENT."
            )
        return self.MAX_ANGLE_MOVEMENT - ((lane_index - 1) * self.cam_angle)

    def _resolve_home_lane(self):
        """Return a lane object given selector_home_lane which may be a key or an index."""
        key = self.selector_home_lane

        # Most AFC versions store lanes as a dict
        if isinstance(self.lanes, dict):
            # Direct key match (e.g. "lane1")
            if isinstance(key, str) and key in self.lanes:
                return self.lanes[key]

            # Numeric match by lane.index (e.g. "1" or 1)
            if isinstance(key, int) or (isinstance(key, str) and key.isdigit()):
                idx = int(key)
                for ln in self.lanes.values():
                    if getattr(ln, "index", None) == idx:
                        return ln

            avail = ", ".join(sorted([str(k) for k in self.lanes.keys()]))
            raise error(
                "AFC_PICOMMU: selector_home_lane '{}' not found. Available lanes: {}".format(key, avail)
            )

        # Some forks may store lanes as list-like
        if isinstance(key, int) or (isinstance(key, str) and key.isdigit()):
            idx = int(key)
            for ln in self.lanes:
                if getattr(ln, "index", None) == idx:
                    return ln
            raise error("AFC_PICOMMU: selector_home_lane {} not found by index".format(idx))

        raise error("AFC_PICOMMU: selector_home_lane must be a lane name or numeric index")

    # ------------------------------------------------------------
    # AFC commands
    # ------------------------------------------------------------
    def cmd_HOME_UNIT(self, gcmd):
        self.return_to_home()

    def return_to_home(self):
        mode = self.selector_home_mode

        if mode == 'none':
            return True

        if mode == 'angle':
            self._set_selector_angle(self.selector_home_angle)
            self.current_selected_lane = None
            return True

        if mode == 'lane':
            home_lane = self._resolve_home_lane()
            self.select_lane(home_lane)
            return True

        raise error("AFC_PICOMMU: selector_home_mode must be one of: angle, lane, none")

    def select_lane(self, lane):
        """
        Keep all normal AFC state machine/motion logic untouched.
        Only change how the selector is actuated.
        """

        # Choose angle:
        angle = None
        if hasattr(lane, 'selector_angle') and lane.selector_angle is not None:
            angle = lane.selector_angle

        # Optional per-lane servo override:
        servo_name = self.selector_servo
        if hasattr(lane, 'selector_servo') and lane.selector_servo:
            servo_name = lane.selector_servo

        # Use per-lane override if present
        if servo_name != self.selector_servo:
            old = self.selector_servo
            self.selector_servo = servo_name
            try:
                if angle is None:
                    angle = self._cam_fallback_angle(lane.index)
                self._set_selector_angle(angle)
            finally:
                self.selector_servo = old
        else:
            if angle is None:
                angle = self._cam_fallback_angle(lane.index)
            self._set_selector_angle(angle)

        self.current_selected_lane = lane
        self.logger.debug(
            "PICOMMU: {} selected lane {} (angle={})".format(self.name, lane.index, angle)
        )
        return True


def load_config_prefix(config):
    return AFC_PICOMMU(config)
