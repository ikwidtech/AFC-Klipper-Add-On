# Armored Turtle Automated Filament Changer
#
# Copyright (C) 2024-2026 Armored Turtle
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from __future__ import annotations

import math
import traceback

from contextlib import contextmanager
from configfile import error
from datetime import datetime
from enum import Enum

from typing import TYPE_CHECKING, Optional
if TYPE_CHECKING:
    from configfile import ConfigWrapper
    from extras.AFC import afc
    from extras.AFC_stepper import AFCExtruderStepper
    from extras.AFC_unit import afcUnit

try: from extras.AFC_utils import ERROR_STR, add_filament_switch
except: raise error("Error when trying to import AFC_utils.ERROR_STR, add_filament_switch\n{trace}".format(trace=traceback.format_exc()))

try: from extras import AFC_assist
except: raise error(ERROR_STR.format(import_lib="AFC_assist", trace=traceback.format_exc()))

try: from extras.AFC_stats import AFCStats_var
except: raise error(ERROR_STR.format(import_lib="AFC_stats", trace=traceback.format_exc()))

if TYPE_CHECKING:
    from extras.AFC_hub import afc_hub

EXCLUDE_TYPES = ["HTLF", "ViViD", "PICOMMU"]
# Class for holding different states so its clear what all valid states are

class AssistActive(Enum):
    YES = 1
    NO = 2
    DYNAMIC = 3
class SpeedMode(Enum):
    NONE = None
    LONG = 1
    SHORT = 2
    HUB = 3
    NIGHT = 4
    CALIBRATION = 5
    DIST_HUB = 6

class MoveDirection(float):
    POS = 1.
    NEG = -1.

class AFCLaneState:
    NONE             = "None"
    ERROR            = "Error"
    LOADED           = "Loaded"
    TOOLED           = "Tooled"
    TOOL_LOADED      = "Tool Loaded"
    TOOL_LOADING     = "Tool Loading"
    TOOL_UNLOADING   = "Tool Unloading"
    HUB_LOADING      = "HUB Loading"
    EJECTING         = "Ejecting"
    CALIBRATING      = "Calibrating"

class AFCHomingPoints(str):
    NONE        = None
    HUB         = "hub"
    LOAD        = "load"
    TOOL        = "tool"
    TOOL_START  = "tool_start"
    BUFFER      = "buffer"
    BUFFER_TRAIL= "buffer_trailing"

class AFCLane:
    UPDATE_WEIGHT_DELAY = 10.0
    def __init__(self, config):
        self._config            = config
        self.printer            = config.get_printer()
        self.afc: afc           = self.printer.load_object(config, 'AFC')
        self.gcode              = self.printer.load_object(config, 'gcode')
        self.reactor            = self.printer.get_reactor()
        self.mutex              = self.reactor.mutex()
        self.extruder_stepper   = None
        self.logger             = self.afc.logger
        self.printer.register_event_handler("klippy:ready", self._handle_ready)
        self.printer.register_event_handler("afc:moonraker_connect", self.handle_moonraker_connect)
        self.cb_update_weight   = self.reactor.register_timer( self.update_weight_callback )

        self.unit_obj: afcUnit  = None
        self.hub_obj: afc_hub   = None
        self.buffer_obj         = None
        self.extruder_obj       = None

        #stored status variables
        self.fullname: str      = config.get_name()
        self.name               = self.fullname.split()[-1]

        # TODO: Put these variables into a common class or something so they are easier to clear out
        # when lanes are unloaded
        self.tool_loaded        = False
        self.loaded_to_hub      = False
        self.spool_id           = None
        self.color              = None
        self.weight             = 0
        self._material          = None
        self.extruder_temp      = None
        self.bed_temp           = None
        self.td1_data           = {}
        self.runout_lane        = None
        self.status             = AFCLaneState.NONE
        # END TODO

        self.multi_hubs_found   = False
        self.drive_stepper: AFCExtruderStepper = None
        unit: str               = config.get('unit')                                    # Unit name(AFC_BoxTurtle/NightOwl/etc) that belongs to this stepper.
        # Overrides buffers set at the unit level
        self.hub: str           = config.get('hub',None)                                # Hub name(AFC_hub) that belongs to this stepper, overrides hub that is set in unit(AFC_BoxTurtle/NightOwl/etc) section.
        # Overrides buffers set at the unit and extruder level
        self.buffer_name: str   = config.get("buffer", None)                            # Buffer name(AFC_buffer) that belongs to this stepper, overrides buffer that is set in extruder(AFC_extruder) or unit(AFC_BoxTurtle/NightOwl/etc) sections.
        self.unit               = unit.split(':')[0]
        try:
            self.index              = int(unit.split(':')[1])
        except:
            self.index              = 0
            pass

        self.extruder_name      = config.get('extruder', None)                          # Extruder name(AFC_extruder) that belongs to this stepper, overrides extruder that is set in unit(AFC_BoxTurtle/NightOwl/etc) section.
        self.remember_spool :bool = config.getboolean('remember_spool', None)             # remember_spool that is set in AFC_Stepper section, overrides remember_spool that is set in unit(AFC_BoxTurtle/NightOwl/etc) section.
        self.map                = config.get('cmd', None)                               # Keeping this in so it does not break others config that may have used this, use map instead
        # Saving to self._map so that if a user has it defined it will be reset back to this when
        # the calling RESET_AFC_MAPPING macro.
        self._map = self.map    = config.get('map', self.map)
        self.led_index          = config.get('led_index', None)                         # LED index of lane in chain of lane LEDs
        self.led_fault          = config.get('led_fault',None)                          # LED color to set when faults occur in lane        (R,G,B,W) 0 = off, 1 = full brightness. Setting value here overrides values set in unit(AFC_BoxTurtle/NightOwl/etc) section
        self.led_ready          = config.get('led_ready',None)                          # LED color to set when lane is ready               (R,G,B,W) 0 = off, 1 = full brightness. Setting value here overrides values set in unit(AFC_BoxTurtle/NightOwl/etc) section
        self.led_not_ready      = config.get('led_not_ready',None)                      # LED color to set when lane not ready              (R,G,B,W) 0 = off, 1 = full brightness. Setting value here overrides values set in unit(AFC_BoxTurtle/NightOwl/etc) section
        self.led_loading        = config.get('led_loading',None)                        # LED color to set when lane is loading             (R,G,B,W) 0 = off, 1 = full brightness. Setting value here overrides values set in unit(AFC_BoxTurtle/NightOwl/etc) section
        self.led_prep_loaded    = config.get('led_loading',None)                        # LED color to set when lane is loaded              (R,G,B,W) 0 = off, 1 = full brightness. Setting value here overrides values set in unit(AFC_BoxTurtle/NightOwl/etc) section
        self.led_unloading      = config.get('led_unloading',None)                      # LED color to set when lane is unloading           (R,G,B,W) 0 = off, 1 = full brightness. Setting value here overrides values set in unit(AFC_BoxTurtle/NightOwl/etc) section
        self.led_tool_loaded    = config.get('led_tool_loaded',None)                    # LED color to set when lane is loaded into tool    (R,G,B,W) 0 = off, 1 = full brightness. Setting value here overrides values set in unit(AFC_BoxTurtle/NightOwl/etc) section
        self.led_spool_index    = config.get('led_spool_index', None)                   # LED index to illuminate under spool
        self.led_spool_illum    = config.get('led_spool_illuminate', None)              # LED color to illuminate under spool

        self.long_moves_speed: float   = config.getfloat("long_moves_speed", None)             # Speed in mm/s to move filament when doing long moves. Setting value here overrides values set in unit(AFC_BoxTurtle/NightOwl/etc) section
        self.long_moves_accel: float   = config.getfloat("long_moves_accel", None)             # Acceleration in mm/s squared when doing long moves. Setting value here overrides values set in unit(AFC_BoxTurtle/NightOwl/etc) section
        self.short_moves_speed: float  = config.getfloat("short_moves_speed", None)            # Speed in mm/s to move filament when doing short moves. Setting value here overrides values set in unit(AFC_BoxTurtle/NightOwl/etc) section
        self.short_moves_accel: float  = config.getfloat("short_moves_accel", None)            # Acceleration in mm/s squared when doing short moves. Setting value here overrides values set in unit(AFC_BoxTurtle/NightOwl/etc) section
        self.short_move_dis: float     = config.getfloat("short_move_dis", None)               # Move distance in mm for failsafe moves. Setting value here overrides values set in unit(AFC_BoxTurtle/NightOwl/etc) section
        self.max_move_dis: float       = config.getfloat("max_move_dis", None)                 # Maximum distance to move filament. AFC breaks filament moves over this number into multiple moves. Useful to lower this number if running into timer too close errors when doing long filament moves. Setting value here overrides values set in unit(AFC_BoxTurtle/NightOwl/etc) section
        self.n20_break_delay_time: float = config.getfloat("n20_break_delay_time", None)        # Time to wait between breaking n20 motors(nSleep/FWD/RWD all 1) and then releasing the break to allow coasting. Setting value here overrides values set in unit(AFC_BoxTurtle/NightOwl/etc) section
        self.homing_overshoot: float   = config.getfloat("homing_overshoot", None)             # Amount to add to homing distance so that distance is long enough to actually hit endstop
        self.homing_delta: float       = config.getfloat("homing_delta", None)                 # Delta for which to warn if homing move delta is not within this amount from command move distance.
        self.load_then_home_var: bool  = config.getboolean("load_then_home", None)
        self.load_undershoot: float    = config.getfloat("load_undershoot", None)
        self.extruder_clear_dis: float = config.getfloat("extruder_clear_dis", None)

        self.rev_long_moves_speed_factor: float = config.getfloat("rev_long_moves_speed_factor", None)     # scalar speed factor when reversing filamentalist

        self.dist_hub: float        = config.getfloat('dist_hub', 60)                       # Bowden distance between Box Turtle extruder and hub
        self.park_dist: float       = config.getfloat('park_dist', 10)                      # Currently unused

        # --- PICOMMU/servo selector overrides (optional; used by AFC_PICOMMU / servo selectors) ---
        self.selector_angle: Optional[float] = config.getfloat("selector_angle", None)
        self.selector_servo: Optional[str]   = config.get("selector_servo", None)

        # Per-lane drive direction multiplier (useful for 2-lane single-drive MMUs where filament approaches the drive from opposite sides)
        #  1 = normal, -1 = reversed
        self.drive_dir: int = config.getint("drive_dir", 1)
        if self.drive_dir not in (-1, 1):
            raise error(f"drive_dir must be 1 or -1 for lane {self.name}")

        self.load_to_hub: bool      = config.getboolean("load_to_hub", self.afc.load_to_hub) # Fast loads filament to hub when inserted, set to False to disable. Setting here overrides global setting in AFC.cfg
        self.enable_sensors_in_gui: bool = config.getboolean("enable_sensors_in_gui",    self.afc.enable_sensors_in_gui) # Set to True to show prep and load sensors switches as filament sensors in mainsail/fluidd gui, overrides value set in AFC.cfg
        self.debounce_delay: float  = config.getfloat("debounce_delay",             self.afc.debounce_delay)
        self.enable_runout: bool    = config.getboolean("enable_hub_runout",        self.afc.enable_hub_runout)
        self.sensor_to_show: str    = config.get("sensor_to_show", None)                # Set to prep to only show prep sensor, set to load to only show load sensor. Do not add if you want both prep and load sensors to show in web gui

        self.assisted_unload: bool = config.getboolean("assisted_unload", None) # If True, the unload retract is assisted to prevent loose windings, especially on full spools. This can prevent loops from slipping off the spool. Setting value here overrides values set in unit(AFC_BoxTurtle/NightOwl/etc) section
        self.td1_when_loaded: bool = config.getboolean("capture_td1_when_loaded", None)
        self.td1_device_id: str    = config.get("td1_device_id", None)
        self.calibrated_lane: bool = config.getboolean("calibrated_lane", False)

        self.post_prep_macro: str  = config.get("post_prep_macro", None)  # Macro to call after loading filament during prep callback


        self.printer.register_event_handler("AFC_unit_{}:connect".format(self.unit),self.handle_unit_connect)

        self.config_dist_hub = self.dist_hub
        self.only_lane = False

        # lane triggers
        buttons = self.printer.load_object(config, "buttons")
        self.prep = config.get('prep', None)                                    # MCU pin for prep trigger
        self.prep_state = False
        if self.prep is not None:
            buttons.register_buttons([self.prep], self.prep_callback)

        self.load = config.get('load', None)                                    # MCU pin load trigger
        self._load_state = False
        if self.load is not None:
            buttons.register_buttons([self.load], self.load_callback)
        else: self._load_state = True

        self.selector: str = config.get("selector", None)

        self.espooler = AFC_assist.Espooler(self.name, config)
        self.lane_load_count = None

        self.filament_diameter: float  = config.getfloat("filament_diameter", 1.75)    # Diameter of filament being used
        self.filament_density: float   = config.getfloat("filament_density", 1.24)     # Density of filament being used
        self.inner_diameter: float     = config.getfloat("spool_inner_diameter", 75, minval=1)   # Inner diameter in mm
        self.outer_diameter: float     = config.getfloat("spool_outer_diameter", 200, minval=100)  # Outer diameter in mm
        self.empty_spool_weight: float = config.getfloat("empty_spool_weight", 190, minval=1)    # Empty spool weight in g
        self.max_motor_rpm: float      = config.getfloat("assist_max_motor_rpm", 465)  # Max motor RPM
        self.rwd_speed_multi: float    = config.getfloat("rwd_speed_multiplier", 0.5)  # Multiplier to apply to rpm
        self.fwd_speed_multi: float    = config.getfloat("fwd_speed_multiplier", 0.5)  # Multiplier to apply to rpm
        self.diameter_range     = self.outer_diameter - self.inner_diameter     # Range for effective diameter
        self.past_extruder_position = -1
        self.save_counter       = -1

        query_endstops = self.printer.load_object( config, "query_endstops")
        ppins          = self.printer.lookup_object('pins')
        # Defaulting to false so that extruder motors to not move until PREP has been called
        self._afc_prep_done = False

        self.prep_endstop_name = None
        self.prep_endstop = None
        if self.prep is not None:
            show_sensor = True
            if not self.enable_sensors_in_gui or (self.sensor_to_show is not None and 'prep' not in self.sensor_to_show):
                show_sensor = False
            self.fila_prep, self.prep_debounce_button = add_filament_switch(f"{self.name}_prep", self.prep, self.printer,
                                                                            show_sensor, enable_runout=self.enable_runout,
                                                                            debounce_delay=self.debounce_delay )
            self.prep_debounce_button.button_action = self.handle_prep_runout
            self.prep_debounce_button.debounce_delay = 0 # Delay will be set once klipper is ready
            ppins.allow_multi_use_pin(self.prep.strip("!^"))
            ppins.parse_pin(self.prep, True, True)
            self.prep_endstop = ppins.setup_pin('endstop', self.prep)
            self.prep_endstop_name = f"{self.name}_prep"
            try:
                query_endstops.register_endstop(self.prep_endstop,
                                                self.prep_endstop_name)
            except Exception as e:
                err_msg = f"Error trying to register prep endstop for {self.name}.\n Error:{e}"
                raise error(err_msg)

        self.load_endstop_name = None
        self.load_endstop = None
        if self.load is not None:
            show_sensor = True
            if not self.enable_sensors_in_gui or (self.sensor_to_show is not None and 'load' not in self.sensor_to_show):
                show_sensor = False
            self.fila_load, self.load_debounce_button = add_filament_switch(f"{self.name}_load", self.load, self.printer,
                                                                            show_sensor, enable_runout=self.enable_runout,
                                                                            debounce_delay=self.debounce_delay )
            self.load_debounce_button.button_action = self.handle_load_runout
            self.load_debounce_button.debounce_delay = 0 # Delay will be set once klipper is ready
            ppins.allow_multi_use_pin(self.load.strip("!^"))
            ppins.parse_pin(self.load, True, True)
            self.load_endstop = ppins.setup_pin('endstop', self.load)
            self.load_endstop_name = f"{self.name}_load"
            try:
                query_endstops.register_endstop(self.load_endstop,
                                                self.load_endstop_name)
            except Exception as e:
                err_msg = f"Error trying to register load endstop for {self.name}.\n Error:{e}"
                raise error(err_msg)

        self.selector_endstop_name = None
        self.selector_endstop = None
        self._selector_state: Optional[bool] = None
        self.selector_cal_dis: Optional[float] = None
        if self.selector is not None:
            self.selector_cal_dis = config.getfloat("selector_cal_distance", 0.0)
            show_sensor = True
            if not self.enable_sensors_in_gui or (self.sensor_to_show is not None and 'selector' not in self.sensor_to_show):
                show_sensor = False
            self.fila_selector = add_filament_switch(f"{self.name}_selector", self.selector,
                                                     self.printer, show_sensor)

            ppins.allow_multi_use_pin(self.selector.strip("!^"))
            ppins.parse_pin(self.selector, True, True)
            self.selector_endstop = ppins.setup_pin('endstop', self.selector)
            self.selector_endstop_name = f"{self.name}_selector"
            try:
                query_endstops.register_endstop(self.selector_endstop,
                                                self.selector_endstop_name)
            except Exception as e:
                err_msg = f"Error trying to register selector endstop for {self.name}.\n Error:{e}"
                raise error(err_msg)

            buttons.register_buttons([self.selector], self.selector_callback)

        self.connect_done = False
        self.prep_active = False
        self.last_prep_time = 0

        self.show_macros = self.afc.show_macros
        self.function = self.printer.load_object(config, 'AFC_functions')
        self.function.register_mux_command(self.show_macros, 'SET_LANE_LOADED', 'LANE', self.name,
                                           self.cmd_SET_LANE_LOADED, self.cmd_SET_LANE_LOADED_help,
                                           self.cmd_SET_LANE_LOAD_options )
        self.function.register_mux_command(self.show_macros, 'AFC_RECOVER_LANE', 'LANE', self.name,
                                           self.cmd_AFC_RECOVER_LANE, self.cmd_AFC_RECOVER_LANE_help,
                                           self.cmd_AFC_RECOVER_LANE_options )
        self._get_steppers(config)

        if getattr(self.unit_obj, "selector_stepper_obj", None):
            # Register macro for units that have selectors
            self.function.register_mux_command(self.afc.show_macros, "AFC_SELECT_LANE",
                                               "LANE", self.name,
                                               self.unit_obj.cmd_AFC_SELECT_LANE,
                                               description=self.unit_obj.cmd_AFC_SELECT_LANE_help,
                                               options=self.unit_obj.cmd_AFC_SELECT_LANE_options)

    def __str__(self):
        return self.name

    @property
    def material(self):
        """
        Returns lanes filament material type
        """
        return self._material

    @property
    def load_es(self) -> str:
        """
        Returns endstop name to use for homing.

        :return str: Returns 'load' if lane has stepper, returns unique lane load name if lane is
                     part of a selector based system.
        """
        if self.only_lane:
            return self.load_endstop_name
        else:
            return AFCHomingPoints.LOAD

    @material.setter
    def material(self, value):
        """
        Sets filament material type and sets filament density based off material type.
        To use custom density, set density after setting material
        """
        self._material = value
        if not value:
            self.filament_density = 1.24 # Setting to a default value
            return

        for density in self.afc.common_density_values:
            v = density.split(":")
            if v[0] in value:
                self.filament_density = float(v[1])
                break

    def _handle_ready(self):
        """
        Handles klippy:ready callback and verifies that steppers have units defined in their config
        """
        if self.unit_obj is None:
            raise error("Unit {unit} is not defined in your configuration file. Please defined unit ex. [AFC_BoxTurtle {unit}]".format(unit=self.unit))

        if self.led_index is not None:
            # Verify that LED config is found
            error_string, led = self.afc.function.verify_led_object(self.led_index)
            if led is None:
                raise error(error_string)
        self.espooler.handle_ready()

        # Setting debounce delay after ready so that callback does not get triggered when initially loading
        if hasattr(self, "prep_debounce_button"):
            self.prep_debounce_button.debounce_delay = self.debounce_delay
        if hasattr(self, "load_debounce_button"):
            self.load_debounce_button.debounce_delay = self.debounce_delay

    def handle_moonraker_connect(self):
        """
        Function that should be called at the beginning of PREP so that moonraker has
        enough time to start before AFC tries to connect. This fixes a race condition that can
        happen between klipper and moonraker when first starting up.
        """

        if (self.unit_obj.type not in EXCLUDE_TYPES
            or (self.unit_obj.type in EXCLUDE_TYPES and "AFC_lane" in self.fullname)):
            values = self.afc.moonraker.get_afc_stats()
            self.lane_load_count = AFCStats_var(self.name, "load_count", values, self.afc.moonraker)
            self.espooler.handle_moonraker_connect()

            # Update boolean and check to make sure a TD-1 device is detected
            self.td1_when_loaded = self.td1_when_loaded and self.afc.td1_defined

    def handle_unit_connect(self, unit_obj):
        """
        Callback from <unit_name>:connect to verify units/hub/buffer/extruder object. Errors out if user specified names and they do not exist in their configuration
        """
        # Saving reference to unit
        self.unit_obj: afcUnit = unit_obj
        self.buffer_obj = self.unit_obj.buffer_obj
        add_to_other_obj = False

        # Inherit remember_spool from unit unless explicitly set in lane config
        if self.remember_spool is None:
            self.remember_spool = bool(self.unit_obj.remember_spool)

        # Register all lanes if their type is not HTLF or only register lanes that are HTLF and have AFC_lane
        # in the name so that HTLF stepper names do not get added since they are not a lane for this unit type
        if (self.unit_obj.type not in EXCLUDE_TYPES
            or (self.unit_obj.type in EXCLUDE_TYPES and "AFC_lane" in self.fullname)):
            add_to_other_obj = True
            # Registering lane name in unit
            self.unit_obj.lanes[self.name] = self
            self.afc.lanes[self.name] = self


        self.hub_obj = self.unit_obj.hub_obj

        if self.hub != 'direct':
            if self.hub is not None:
                try:
                    self.hub_obj = self.printer.lookup_object("AFC_hub {}".format(self.hub))
                except:
                    error_string = 'Error: No config found for hub: {hub} in [AFC_stepper {stepper}]. Please make sure [AFC_hub {hub}] section exists in your config'.format(
                    hub=self.hub, stepper=self.name )
                    raise error(error_string)
            # Removing for now as this is probably not the best idea to default to a random hub
            # elif self.hub_obj is None:
            #     # Check to make sure at least 1 hub exists in config, if not error out with message
            #     if len(self.afc.hubs) == 0:
            #         error_string = "Error: AFC_hub not found in configuration please make sure there is a [AFC_hub <hub_name>] defined in your configuration"
            #         raise error(error_string)
            #     # Setting hub to first hub in AFC hubs dictionary
            #     if len(self.afc.hubs) > 0:
            #         self.hub_obj = next(iter(self.afc.hubs.values()))
            #     # Set flag to warn during prep that multiple hubs were found
            #         if len(self.afc.hubs) > 1:
            #             self.multi_hubs_found = True

            # Assigning hub name just in case stepper is using hub defined in units config
            if self.hub_obj:
                self.hub = self.hub_obj.name
                if add_to_other_obj:
                    self.hub_obj.lanes[self.name] = self
        else:
            self.hub_obj = lambda: None
            self.hub_obj.state = False
            self.hub_obj.move_dis = 75
            self.hub_obj.hub_clear_move_dis = 65

        self.extruder_obj = self.unit_obj.extruder_obj
        if self.extruder_name is not None:
            try:
                self.extruder_obj = self.printer.lookup_object('AFC_extruder {}'.format(self.extruder_name))
            except:
                error_string = 'Error: No config found for extruder: {extruder} in [AFC_stepper {stepper}]. Please make sure [AFC_extruder {extruder}] section exists in your config'.format(
                    extruder=self.extruder_name, stepper=self.name )
                raise error(error_string)
        elif self.extruder_obj is None:
            error_string = "Error: Extruder has not been configured for stepper {name}, please add extruder variable to either [AFC_stepper {name}] or [AFC_{unit_type} {unit_name}] in your config file".format(
                        name=self.name, unit_type=self.unit_obj.type.replace("_", ""), unit_name=self.unit_obj.name)
            raise error(error_string)

        # Assigning extruder name just in case stepper is using extruder defined in units config
        self.extruder_name = self.extruder_obj.name
        if add_to_other_obj:
            self.extruder_obj.lanes[self.name] = self

        # Use buffer defined in stepper and override buffers that maybe set at the UNIT or extruder levels
        self.buffer_obj = self.unit_obj.buffer_obj
        if self.buffer_name is not None:
            try:
                self.buffer_obj = self.printer.lookup_object("AFC_buffer {}".format(self.buffer_name))
            except:
                error_string = 'Error: No config found for buffer: {buffer} in [AFC_stepper {stepper}]. Please make sure [AFC_buffer {buffer}] section exists in your config'.format(
                    buffer=self.buffer_name, stepper=self.name )
                raise error(error_string)

        # Checking if buffer was defined in extruder if not defined in unit/stepper
        elif self.buffer_obj is None and self.extruder_obj.tool_start == "buffer":
            if self.extruder_obj.buffer_name is not None:
                self.buffer_obj = self.printer.lookup_object("AFC_buffer {}".format(self.extruder_obj.buffer_name))
            else:
                error_string = 'Error: Buffer was defined as tool_start in [AFC_extruder {extruder}] config, but buffer variable has not been configured. Please add buffer variable to either [AFC_extruder {extruder}], [AFC_stepper {name}] or [AFC_{unit_type} {unit_name}] section in your config file'.format(
                    extruder=self.extruder_obj.name, name=self.name, unit_type=self.unit_obj.type.replace("_", ""), unit_name=self.unit_obj.name )
                raise error(error_string)

        # Valid to not have a buffer defined, check to make sure object exists before adding lane to buffer
        if self.buffer_obj is not None and add_to_other_obj:
            self.buffer_obj.lanes[self.name] = self
            # Assigning buffer name just in case stepper is using buffer defined in units/extruder config
            self.buffer_name = self.buffer_obj.name

        if self.led_fault           is None: self.led_fault         = self.unit_obj.led_fault
        if self.led_ready           is None: self.led_ready         = self.unit_obj.led_ready
        if self.led_not_ready       is None: self.led_not_ready     = self.unit_obj.led_not_ready
        if self.led_loading         is None: self.led_loading       = self.unit_obj.led_loading
        if self.led_prep_loaded     is None: self.led_prep_loaded   = self.unit_obj.led_prep_loaded
        if self.led_unloading       is None: self.led_unloading     = self.unit_obj.led_unloading
        if self.led_tool_loaded     is None: self.led_tool_loaded   = self.unit_obj.led_tool_loaded
        if self.led_spool_illum     is None: self.led_spool_illum   = self.unit_obj.led_spool_illum

        if self.rev_long_moves_speed_factor is None: self.rev_long_moves_speed_factor  = self.unit_obj.rev_long_moves_speed_factor
        if self.long_moves_speed            is None: self.long_moves_speed  = self.unit_obj.long_moves_speed
        if self.long_moves_accel            is None: self.long_moves_accel  = self.unit_obj.long_moves_accel
        if self.short_moves_speed           is None: self.short_moves_speed = self.unit_obj.short_moves_speed
        if self.short_moves_accel           is None: self.short_moves_accel = self.unit_obj.short_moves_accel
        if self.short_move_dis              is None: self.short_move_dis    = self.unit_obj.short_move_dis
        if self.max_move_dis                is None: self.max_move_dis      = self.unit_obj.max_move_dis
        if self.homing_overshoot            is None: self.homing_overshoot  = self.unit_obj.homing_overshoot
        if self.homing_delta                is None: self.homing_delta      = self.unit_obj.homing_delta
        if self.load_then_home_var          is None: self.load_then_home_var= self.unit_obj.load_then_home_var
        if self.load_undershoot             is None: self.load_undershoot   = self.unit_obj.load_undershoot
        if self.td1_when_loaded             is None: self.td1_when_loaded   = self.unit_obj.td1_when_loaded
        if self.td1_device_id               is None: self.td1_device_id     = self.unit_obj.td1_device_id
        if self.extruder_clear_dis          is None: self.extruder_clear_dis= self.unit_obj.extruder_clear_dis
        if self.post_prep_macro             is None: self.post_prep_macro   = self.unit_obj.post_prep_macro

        if self.rev_long_moves_speed_factor < 0.5: self.rev_long_moves_speed_factor = 0.5
        if self.rev_long_moves_speed_factor > 1.2: self.rev_long_moves_speed_factor = 1.2

        self.espooler.handle_connect(self)

        # Set hub loading speed depending on distance between extruder and hub
        self.dist_hub_move_speed = self.long_moves_speed if self.dist_hub >= 200 else self.short_moves_speed
        self.dist_hub_move_accel = self.long_moves_accel if self.dist_hub >= 200 else self.short_moves_accel

        # Register macros
        # TODO: add check so that HTLF stepper lanes do not get registered here
        self.afc.gcode.register_mux_command('SET_LONG_MOVE_SPEED',    "LANE", self.name, self.cmd_SET_LONG_MOVE_SPEED, desc=self.cmd_SET_LONG_MOVE_SPEED_help)
        self.afc.gcode.register_mux_command('SET_SPEED_MULTIPLIER',   "LANE", self.name, self.cmd_SET_SPEED_MULTIPLIER, desc=self.cmd_SET_SPEED_MULTIPLIER_help)
        self.afc.gcode.register_mux_command('SAVE_SPEED_MULTIPLIER',  "LANE", self.name, self.cmd_SAVE_SPEED_MULTIPLIER, desc=self.cmd_SAVE_SPEED_MULTIPLIER_help)
        self.afc.gcode.register_mux_command('SET_HUB_DIST',           "LANE", self.name, self.cmd_SET_HUB_DIST, desc=self.cmd_SET_HUB_DIST_help)
        self.afc.gcode.register_mux_command('SAVE_HUB_DIST',          "LANE", self.name, self.cmd_SAVE_HUB_DIST, desc=self.cmd_SAVE_HUB_DIST_help)
        self.afc.gcode.register_mux_command('AFC_SET_REMEMBER_SPOOL', "LANE", self.name, self.cmd_AFC_SET_REMEMBER_SPOOL, desc=self.cmd_AFC_SET_REMEMBER_SPOOL_help)

        if self.assisted_unload is None: self.assisted_unload = self.unit_obj.assisted_unload

        # Send out event so that macros and be registered properly with valid lane names
        self.printer.send_event("afc_stepper:register_macros", self)

        self.connect_done = True

    def _get_steppers(self, config: ConfigWrapper):
        """
        Helper function to get steppers for lane and setup for proper homing
        """
        if config.get("step_pin", None):
            return
        try:
            self.only_lane = True
            unit_cfg = next(config.getsection(s) for s in config.fileconfig.sections() if self.unit in s and "AFC" in s)
            self.logger.info(f"{unit_cfg.get_name()} drive stepper {self.name}")
            self.unit_obj: afcUnit = self.printer.load_object(config, unit_cfg.get_name())

            if getattr(self.unit_obj, "drive_stepper_obj", None):
                self.drive_stepper = self.unit_obj.drive_stepper_obj
                if self.load_endstop:
                    self.load_endstop.add_stepper(self.drive_stepper.extruder_stepper.stepper)
                    self.drive_stepper._endstops[self.load_endstop_name] = (self.load_endstop, self.load_endstop_name)

                if self.prep_endstop:
                    self.prep_endstop.add_stepper(self.drive_stepper.extruder_stepper.stepper)
                    self.drive_stepper._endstops[self.prep_endstop_name] = (self.prep_endstop, self.prep_endstop_name)

        except Exception as e:
            self.logger.info(f"Couldn't find unit for {self.name} {e}")
            return

        if (self.unit_obj.type in EXCLUDE_TYPES
            and "AFC_lane" in self.fullname):
            self.drive_stepper      = self.unit_obj.drive_stepper_obj
            self.extruder_stepper   = self.drive_stepper.extruder_stepper
            if (self.selector
                and getattr(self.unit_obj, "selector_stepper_obj", None)):
                selector_stepper = self.unit_obj.selector_stepper_obj
                self.selector_endstop.add_stepper(selector_stepper.extruder_stepper.stepper)
                selector_stepper._endstops[self.selector_endstop_name] = (self.selector_endstop, self.selector_endstop_name)

    def get_color(self):
        """
        Helper function for returning current color

        :return str: If TD-1 device is present, returns scanned color. If its not present, returns
                     manually entered or color from spoolman
        """
        color = self.color
        if "color" in self.td1_data:
            color = f"#{self.td1_data['color']}"
        return color

    @contextmanager
    def assist_move(self, speed, rewind, assist_active=True):
        """
        Starts an assist move and returns a context manager that turns off the assist move when it exist.
        :param speed:         The speed of the move
        :param rewind:        True for a rewind, False for a forward assist
        :param assist_active: Whether to assist
        :return:              the Context manager
        """
        if assist_active:
            if rewind:
                # Calculate Rewind Speed
                value = self.calculate_pwm_value(speed, True) * -1
            else:
                # Calculate Forward Assist Speed
                value = self.calculate_pwm_value(speed)

            # Clamp value to a maximum of 1
            if value > 1:
                value = 1

            self.espooler.assist(value)
        try:
            yield
        finally:
            if assist_active:
                self.espooler.assist(0)

    def move_auto_speed(self, distance):
        """
        Helper function for determining speed and accel from passed in distance

        :param distance: Distance to move filament
        """
        dist_hub_move_speed, dist_hub_move_accel, assist_active = self.get_speed_accel(mode=SpeedMode.NONE,
                                                                                       distance=distance)
        self.move(distance, dist_hub_move_speed, dist_hub_move_accel, assist_active)

    def get_speed_accel(self, mode: SpeedMode, distance=None):
        """
        Helper function to allow selecting the right speed and acceleration of movements.

        Always returns a 3-tuple: (speed, accel, assist_active)

        This fixes a DEV issue where the "else" branch returned only (speed, accel),
        while move_auto_speed() expects (speed, accel, assist_active).
        """
        # Distance-driven auto selection (used by move_auto_speed)
        if distance is not None and mode is SpeedMode.NONE:
            if abs(distance) > 200:
                return self.long_moves_speed, self.long_moves_accel, True
            else:
                # FIX: use short_moves_accel (DEV previously used long_moves_accel here)
                return self.short_moves_speed, self.short_moves_accel, False

        # Mode-driven selection (most calls land here)
        assist_active = False
        if self.afc._get_quiet_mode() == True:
            return self.afc.quiet_moves_speed, self.short_moves_accel, assist_active
        elif mode == SpeedMode.LONG:
            return self.long_moves_speed, self.long_moves_accel, assist_active
        elif (mode == SpeedMode.SHORT
              or mode == SpeedMode.CALIBRATION
              or mode == SpeedMode.HUB):
            return self.short_moves_speed, self.short_moves_accel, assist_active
        elif (mode == SpeedMode.DIST_HUB):
            return self.dist_hub_move_speed, self.dist_hub_move_accel, assist_active
        else:
            return self.short_moves_speed, self.short_moves_accel, assist_active

    def get_active_assist(self, distance, assist_active:AssistActive) -> bool:
        """
        Helper method to return boolean based off `AssistActive` enum and distance

        :param distance: Distance to move stepper, used to determine assist if using Dynamic assist mode
        :param assist_active: `AssistActive` enum to determine if assist will be active or not
        :return bool: Returns True if `AssistActive.YES` or `AssistActive.DYNAMIC` and distance > 200
        """
        assist = False
        if assist_active == AssistActive.YES:
            assist = True
        elif assist_active == AssistActive.DYNAMIC:
            assist = abs(distance) > 200
        return assist

    def move(self, distance, speed, accel, assist_active=False):
        """
        Move the specified lane a given distance with specified speed and acceleration.
        This function calculates the movement parameters and commands the stepper motor
        to move the lane accordingly.
        Parameters:
        distance (float): The distance to move.
        speed (float): The speed of the movement.
        accel (float): The acceleration of the movement.
        """
        self.unit_obj.select_lane( self )

        # Apply per-lane direction flip for shared-drive MMUs (PICOMMU)
        signed_distance = float(distance) * float(self.drive_dir)

        with self.assist_move( speed, signed_distance < 0, assist_active):
            if self.drive_stepper is not None:
                self.drive_stepper.move(signed_distance, speed, accel, assist_active)

    def move_to(self, distance: float, speed_mode: SpeedMode,
                endstop:AFCHomingPoints=AFCHomingPoints.NONE,
                assist_active=AssistActive.NO, use_homing=True) -> tuple[bool, float|int, bool]:
        warn = False
        if (self.drive_stepper
            or hasattr(self, "extruder_stepper")):
            if use_homing:
                if self.drive_stepper:
                    home_to = self.drive_stepper.home_to
                else:
                    home_to = self.home_to
                # Add extra distance to homing move to guarantee that endstop is hit
                new_distance = distance + self.homing_overshoot
                if distance < 0:
                    new_distance = distance - self.homing_overshoot
                self.unit_obj.select_lane(self)
                homed, mov_dis = home_to(endstop, new_distance, speed_mode,
                        distance > 0, assist_active=self.get_active_assist(distance, assist_active))
                if (abs(distance) - mov_dis) > self.homing_delta:
                    warn = True

                return homed, mov_dis, warn
            else:
                self.move_advanced(distance, speed_mode, assist_active )
                return True, 0, warn

    def move_advanced(self, distance, speed_mode: SpeedMode, assist_active: AssistActive = AssistActive.NO):
        """
        Wrapper for move function and is used to compute several arguments
        to move the lane accordingly.
        """
        speed, accel, _ = self.get_speed_accel(speed_mode)

        assist = self.get_active_assist(distance, assist_active)

        self.move(distance, speed, accel, assist)

    def set_afc_prep_done(self):
        self._afc_prep_done = True

    def _perform_infinite_runout(self):
        self.status = AFCLaneState.NONE
        self.afc.function.afc_led(self.afc.led_not_ready, self.led_index)
        self.logger.info("Infinite Spool triggered for {}".format(self.name))
        empty_lane = self.afc.lanes.get(self.afc.current)
        change_lane = self.afc.lanes.get(self.runout_lane)
        # Pause printer with manual command
        self.afc.error.pause_resume.send_pause_command()
        # Saving position after printer is paused
        self.afc.save_pos()

        # Verifying lanes are valid before continuing
        if not change_lane:
            self.afc.error.AFC_error(f"Error when looking up runout lane:{self.runout_lane} for lane:{self.name}")
            return
        if not empty_lane:
            self.afc.error.AFC_error(f"Error when looking up current lane:{self.afc.current}")
            return

        # Position will be restored after lane is unloaded so that nozzle does not sit
        # on print while lane is unloading
        if not self.afc.TOOL_UNLOAD(empty_lane):
            return

        # Eject spool before loading next lane
        self.gcode.run_script_from_command('LANE_UNLOAD LANE={}'.format(empty_lane.name))

        self.afc.TOOL_LOAD(change_lane)
        # Change Mapping
        self.gcode.run_script_from_command('SET_MAP LANE={} MAP={}'.format(change_lane.name, empty_lane.map))

        # Only continue if a error did not happen
        if not self.afc.error_state:
            # Resume pos
            self.afc.restore_pos()
            # Resume with manual issued command
            self.afc.error.pause_resume.send_resume_command()
            # Set LED to not ready
            self.afc.function.afc_led(self.led_not_ready, self.led_index)

    def _perform_pause_runout(self):
        # Unload if user has set AFC to unload on runout
        if self.unit_obj.unload_on_runout:
            # Pause printer
            self.afc.error.pause_resume.send_pause_command()
            self.afc.save_pos()
            self.afc.TOOL_UNLOAD(self)
            if not self.afc.error_state:
                self.afc.LANE_UNLOAD(self)
        # Pause print
        self.status = AFCLaneState.NONE
        msg = "Runout triggered for lane {} and runout lane is not setup to switch to another lane".format(self.name)
        msg += "\nPlease manually load next spool into toolhead and then hit resume to continue"
        self.afc.function.afc_led(self.afc.led_not_ready, self.led_index)
        self.afc.error.AFC_error(msg)

    def _prep_capture_td1(self):
        if self.td1_when_loaded:
            if not self.hub_obj.state and self.afc.function.get_current_lane_obj() is None:
                self.get_td1_data()
            else:
                self.logger.info(f"Cannot get TD-1 data for {self.name}, either toolhead is loaded or hub shows filament in path")

    @property
    def load_state(self) -> bool:
        if self.unit_obj.type == "ViViD":
            return self.loaded_to_hub
        else:
            return bool(self._load_state)

    @property
    def raw_load_state(self) -> bool:
        return bool(self._load_state)

    def selector_callback(self, eventtime: float, state):
        self._selector_state = state

    def load_callback(self, eventtime, state):
        self._load_state = state
        if self.printer.state_message == 'Printer is ready' and self.unit_obj.type in ("HTLF", "PICOMMU"):
            self.prep_state = state

    def handle_load_runout(self, eventtime=None, load_state=None, is_filament_present=None):
        """
        Callback function for load switch runout/loading for HTLF/PICOMMU.

        Supports both callback styles:
          - older: button_action(eventtime, state)
          - newer: button_action(is_filament_present=state)
        """
        if load_state is None and is_filament_present is not None:
            load_state = is_filament_present
        if eventtime is None:
            eventtime = 0.0

        # Call filament sensor callback so that state is registered
        try:
            self.load_debounce_button._old_note_filament_present(is_filament_present=load_state)
        except:
            self.load_debounce_button._old_note_filament_present(eventtime, load_state)

        if (self.printer.state_message == 'Printer is ready' and
            self.unit_obj.type in ("HTLF", "PICOMMU") and
            self._afc_prep_done):
            if load_state:
                self.status = AFCLaneState.LOADED
                self.unit_obj.lane_loaded(self)
                self.afc.spool._set_values(self)
                # Check if user wants to get TD-1 data when loading
                if not self.tool_loaded:
                    self._prep_capture_td1()
                self._post_prep_user_macro()
            else:
                # Don't run if user disabled sensor in gui
                if not self.fila_load.runout_helper.sensor_enabled and self.afc.function.is_printing():
                    self.logger.warning("Load runout has been detected, but pause and runout detection has been disabled")
                else:
                    try:
                        do_runout = self.unit_obj.check_runout(self)
                    except TypeError:
                        do_runout = self.unit_obj.check_runout()

                    if do_runout:
                        # Checking to make sure runout_lane is set
                        if self.runout_lane is not None:
                            self._perform_infinite_runout()
                        else:
                            self._perform_pause_runout()
                    elif self.status != "calibrating":
                        self.tool_loaded = False
                        self.afc.function.afc_led(self.led_not_ready, self.led_index)
                        self.status = AFCLaneState.NONE
                        self.loaded_to_hub = False
                        self.td1_data = {}
                        self.afc.spool.clear_values(self)
                        self.afc.function.afc_led(self.afc.led_not_ready, self.led_index)

        self.afc.save_vars()

    def prep_callback(self, eventtime, state):
        self.prep_state = state

        delta_time = eventtime - self.last_prep_time
        self.last_prep_time = eventtime

        if self.prep_active:
            return

        if self.hub =='direct' and not self.afc.function.is_homed():
            self.afc.error.AFC_error("Please home printer before directly loading to toolhead", False)
            return False

        self.prep_active = True

        # Checking to make sure printer is ready and making sure PREP has been called before trying to load anything
        for i in range(1):
            with self.mutex:
                if self.printer.state_message == 'Printer is ready' and self._afc_prep_done and self.status != AFCLaneState.TOOL_UNLOADING:
                    # Only try to load when load state trigger is false
                    if self.prep_state and not self.raw_load_state:
                        # Checking to make sure last time prep switch was activated was less than 1 second
                        if delta_time < 1.0:
                            break

                        # Check to see if the printer is printing or moving
                        if self.afc.function.is_printing(check_movement=True):
                            self.afc.error.AFC_error(f"Cannot load {self.name} spool while printer is actively moving or homing", False)
                            self.prep_active = False
                            return

                        # Calling common load function
                        self.unit_obj.prep_load(self)

                        self.status = AFCLaneState.NONE

                        # Verify that load state is still true as this would still trigger if prep sensor was triggered and then filament was removed
                        if self.hub == 'direct' and self.prep_state:
                            self.afc.afcDeltaTime.set_start_time()
                            self.afc.TOOL_LOAD(self)
                            self.material = self.afc.default_material_type
                            break

                        self.unit_obj.prep_post_load(self)

                        self.do_enable(False)
                        if (self.load_state
                            and self.prep_state):
                            self.status = AFCLaneState.LOADED
                            self.unit_obj.lane_loaded(self)
                            self.afc.spool._set_values(self)
                            self._post_prep_user_macro()
                            self._prep_capture_td1()

                    elif (self.prep_state == True
                          and self.raw_load_state == True
                          and not self.afc.function.is_printing()):
                        message = 'Cannot load {} load sensor is triggered.'.format(self.name)
                        message += '\n    Make sure filament is not stuck in load sensor or check to make sure load sensor is not stuck triggered.'
                        if self.unit_obj.type == "ViViD":
                            message += f'\n    If filament is not stuck in sensor run AFC_RECOVER_LANE LANE={self.name}'
                            message += " to reset internal AFC state."
                        message += '\n    Once cleared try loading again'
                        self.afc.error.AFC_error(message, pause=False)
        self.prep_active = False
        self.afc.save_vars()

    def handle_prep_runout(self, eventtime, prep_state):
        """
        Callback function for prep switch runout.
        """
        # Call filament sensor callback so that state is registered
        try:
            self.prep_debounce_button._old_note_filament_present(is_filament_present=prep_state)
        except:
            self.prep_debounce_button._old_note_filament_present(eventtime, prep_state)

        if (self.printer.state_message == 'Printer is ready'
            and True == self._afc_prep_done
            and self.status != AFCLaneState.TOOL_UNLOADING):
            if (prep_state == False
                and self.name == self.afc.current
                and self.afc.function.is_printing()
                and self.raw_load_state
                and self.status != AFCLaneState.EJECTING):
                # Don't run if user disabled sensor in gui
                if not self.fila_prep.runout_helper.sensor_enabled:
                    self.logger.warning("Prep runout has been detected, but pause and runout detection has been disabled")
                # Checking to make sure runout_lane is set
                elif self.runout_lane is not None:
                    self._perform_infinite_runout()
                else:
                    self._perform_pause_runout()
            elif not prep_state:
                # Filament is unloaded
                self.tool_loaded = False
                self.status = AFCLaneState.NONE
                self.loaded_to_hub = False
                self.td1_data = {}
                if not self.remember_spool:
                    self.afc.spool.clear_values(self)
                self.unit_obj.lane_unloaded(self)

        self.afc.save_vars()

    def _post_prep_user_macro(self):
        """
        Function to call macro once filament has successfully been loaded during prep callback
        """
        if self.afc.function.check_macro_present(self.post_prep_macro):
            cmd = f"{self.post_prep_macro} LANE={self.name}"
            self.gcode.run_script_from_command(cmd)

    def do_enable(self, enable):
        if self.drive_stepper is not None:
            self.drive_stepper.do_enable(enable)

    def sync_print_time(self):
        return

    def sync_to_extruder(self, update_current=True):
        if self.drive_stepper is not None:
            self.drive_stepper.sync_to_extruder(update_current, extruder_name=self.extruder_name)

    def unsync_to_extruder(self, update_current=True):
        if self.drive_stepper is not None:
            self.drive_stepper.unsync_to_extruder(update_current)

    def _set_current(self, current):
        return

    def set_load_current(self):
        if self.drive_stepper is not None:
            self.drive_stepper.set_load_current()

    def set_print_current(self):
        if self.drive_stepper is not None:
            self.drive_stepper.set_print_current()

    def update_rotation_distance(self, multiplier):
        if self.drive_stepper is not None:
            self.drive_stepper.update_rotation_distance( multiplier )

    def calculate_effective_diameter(self, weight_g, spool_width_mm=60):
        density_g_mm3 = self.filament_density / 1000.0
        filament_volume_mm3 = weight_g / density_g_mm3
        package_corrected_volume_mm3 = filament_volume_mm3 / 0.785
        filament_area_mm2 = package_corrected_volume_mm3 / spool_width_mm
        spool_outer_diameter_mm2 = (4 * filament_area_mm2 / 3.14159) + self.inner_diameter ** 2
        spool_outer_diameter_mm = spool_outer_diameter_mm2 ** 0.5
        return spool_outer_diameter_mm

    def calculate_rpm(self, feed_rate):
        weight = self.weight + self.empty_spool_weight
        effective_diameter = self.calculate_effective_diameter(weight)
        rpm = (feed_rate * 60) / (math.pi * effective_diameter)
        return min(rpm, self.max_motor_rpm)

    def calculate_pwm_value(self, feed_rate, rewind=False):
        rpm = self.calculate_rpm(feed_rate)
        if not rewind:
            pwm_value = rpm / (self.max_motor_rpm / (1 + 9 * self.fwd_speed_multi))
        else:
            pwm_value = rpm / (self.max_motor_rpm / (15 + 15 * self.rwd_speed_multi))
        return max(0.0, min(pwm_value, 1.0))

    def enable_weight_timer(self):
        self.past_extruder_position = self.afc.function.get_extruder_pos( None, self.past_extruder_position )
        self.reactor.update_timer( self.cb_update_weight, self.reactor.monotonic() + self.UPDATE_WEIGHT_DELAY)

    def disable_weight_timer(self):
        self.update_weight_callback( None )
        self.reactor.update_timer( self.cb_update_weight, self.reactor.NEVER)
        self.past_extruder_position = -1
        self.save_counter = -1
        self.afc.save_vars()

    def update_weight_callback(self, eventtime):
        extruder_pos = self.afc.function.get_extruder_pos( eventtime, self.past_extruder_position )
        delta_length = extruder_pos - self.past_extruder_position

        if -1 == self.past_extruder_position:
            self.past_extruder_position = extruder_pos

        self.save_counter += 1
        if extruder_pos > self.past_extruder_position:
            self.update_remaining_weight(delta_length)
            self.past_extruder_position = extruder_pos

            # Save vars every 2 minutes
            if self.save_counter > 120/self.UPDATE_WEIGHT_DELAY:
                self.afc.save_vars()
                self.save_counter = 0

        return self.reactor.monotonic() + self.UPDATE_WEIGHT_DELAY

    def update_remaining_weight(self, distance_moved):
        filament_volume_mm3 = math.pi * (self.filament_diameter / 2) ** 2 * distance_moved
        filament_weight_change = filament_volume_mm3 * self.filament_density / 1000
        self.weight -= filament_weight_change
        if self.weight < 0:
            self.weight = 0

    def set_loaded(self):
        self.tool_loaded = True
        self.afc.current = self.extruder_obj.lane_loaded = self.name
        self.afc.current_loading = None
        self.status = AFCLaneState.TOOLED
        self.afc.spool.set_active_spool(self.spool_id)
        self.unit_obj.lane_tool_loaded(self)

    def set_unloaded(self):
        self.tool_loaded = False
        self.extruder_obj.lane_loaded = ""
        self.status = AFCLaneState.NONE
        self.afc.current = None
        self.afc.current_loading = None
        self.afc.spool.set_active_spool(None)
        self.unit_obj.lane_tool_unloaded(self)

    def enable_buffer(self, disable_fault: bool=False):
        if self.buffer_obj is not None:
            if disable_fault: self.buffer_obj.disable_fault_sensitivity()
            self.buffer_obj.enable_buffer()
        self.espooler.enable_timer()
        self.enable_weight_timer()

    def enable_fault_detection(self):
        if self.buffer_obj is not None:
            self.buffer_obj.restore_fault_sensitivity()
            self.buffer_obj.enable_buffer()

    def disable_buffer(self):
        if self.buffer_obj is not None:
            self.buffer_obj.disable_buffer()
        self.espooler.disable_timer()
        self.disable_weight_timer()

    def buffer_status(self):
        if self.buffer_obj is not None:
            return self.buffer_obj.buffer_status()
        else:
            return None

    def get_toolhead_pre_sensor_state(self):
        if self.extruder_obj.tool_start == "buffer":
            return self.buffer_obj.advance_state
        else:
            return self.extruder_obj.tool_start_state

    def get_toolhead_endstop(self) -> AFCHomingPoints:
        if self.extruder_obj.tool_start == "buffer":
            return AFCHomingPoints.BUFFER
        else:
            return AFCHomingPoints.TOOL

    def get_trailing(self):
        if self.buffer_obj is not None:
            return self.buffer_obj.trailing_state
        else: return None

    def _is_normal_printing_state(self):
        if self.afc.in_toolchange:
            return False
        return self.status in (AFCLaneState.TOOLED, AFCLaneState.LOADED)

    def handle_toolhead_runout(self, sensor=None):
        if not (self._is_normal_printing_state() and self.afc.function.is_printing()):
            return

        prep_ok = self.prep_state
        load_ok = self.raw_load_state
        hub_ok = self.hub_obj.state if self.hub_obj is not None else True

        if prep_ok and load_ok and hub_ok:
            msg = (
                f"Toolhead runout detected by {sensor} sensor, but upstream sensors still detect filament.\n"
                "Possible filament break or jam at the toolhead. Please clear the jam and reload filament manually, then resume the print."
            )
            self.afc.error.pause_resume.send_pause_command()
            self.afc.save_pos()
            self.afc.error.AFC_error(msg)

    def handle_hub_runout(self, sensor=None):
        if not (self._is_normal_printing_state() and self.afc.function.is_printing()):
            return***
