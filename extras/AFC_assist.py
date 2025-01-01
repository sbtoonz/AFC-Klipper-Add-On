# Armored Turtle Automated Filament Changer
# Refactored AFC_Assist.py

import math

# Global debug flag
DEBUG = True

# Global constants
PIN_MIN_TIME = 0.100
RESEND_HOST_TIME = 0.300 + PIN_MIN_TIME
MAX_SCHEDULE_TIME = 5.0

class AFCassistMotor:
    def __init__(self, config, motor_type):
        # Initialize all attributes upfront to avoid AttributeError
        self.printer = None
        self.gcode = None
        self.mcu_pin = None
        self.is_pwm = False
        self.scale = 1.0
        self.last_print_time = 0.0
        self.last_value = 0.0
        self.shutdown_value = 0.0
        self.reactor = None
        self.resend_timer = None
        self.resend_interval = 0.0

        try:
            self.printer = config.get_printer()
            self.gcode = self.printer.lookup_object('gcode')
            ppins = self.printer.lookup_object('pins')
            self.is_pwm = config.getboolean('pwm', False)

            # Setup pins
            pin_name = f'afc_motor_{motor_type}'
            if self.is_pwm:
                self.mcu_pin = ppins.setup_pin('pwm', config.get(pin_name))
                cycle_time = config.getfloat('cycle_time', 0.100, above=0., maxval=MAX_SCHEDULE_TIME)
                hardware_pwm = config.getboolean('hardware_pwm', False)
                self.mcu_pin.setup_cycle_time(cycle_time, hardware_pwm)
                self.scale = config.getfloat('scale', 1., above=0.)
            else:
                self.mcu_pin = ppins.setup_pin('digital_out', config.get(pin_name))

            self.reactor = self.printer.get_reactor()
            self.resend_interval = self._calculate_resend_interval(config)

            # Configure start and shutdown values
            self._initialize_values(config)

            if DEBUG:
                self.gcode.respond_raw(f"AFCassistMotor initialized for {motor_type}")
        except Exception as e:
            if DEBUG:
                self.gcode.respond_raw(f"Error initializing AFCassistMotor for {motor_type}: {e}")

    def _calculate_resend_interval(self, config):
        try:
            max_mcu_duration = config.getfloat('maximum_mcu_duration', 0., minval=0.500, maxval=MAX_SCHEDULE_TIME)
            self.mcu_pin.setup_max_duration(max_mcu_duration)
            if max_mcu_duration:
                config.deprecate('maximum_mcu_duration')
                return max_mcu_duration - RESEND_HOST_TIME
            return 0.0
        except Exception as e:
            if DEBUG:
                self.gcode.respond_raw(f"Error calculating resend interval: {e}")
            return 0.0

    def _initialize_values(self, config):
        try:
            static_value = config.getfloat('static_value', None, minval=0., maxval=self.scale)
            if static_value is not None:
                config.deprecate('static_value')
                self.last_value = self.shutdown_value = static_value / self.scale
            else:
                self.last_value = config.getfloat('value', 0., minval=0., maxval=self.scale) / self.scale
                self.shutdown_value = config.getfloat('shutdown_value', 0., minval=0., maxval=self.scale) / self.scale

            self.mcu_pin.setup_start_value(self.last_value, self.shutdown_value)
        except Exception as e:
            if DEBUG:
                self.gcode.respond_raw(f"Error initializing pin values: {e}")

    def get_status(self, eventtime):
        return {'value': self.last_value}

    def _set_pin(self, print_time, value, is_resend=False):
        try:
            if value == self.last_value and not is_resend:
                return

            # Prevent rapid pin updates
            if print_time < self.last_print_time + PIN_MIN_TIME:
                if DEBUG:
                    self.gcode.respond_raw("Pin update skipped to prevent rate overrun.")
                return

            print_time = max(print_time, self.last_print_time + PIN_MIN_TIME)

            # Gradual ramping to prevent sudden stalls
            ramp_rate = 0.1  # Increment for ramping the duty cycle
            if value > self.last_value:
                value = min(value, self.last_value + ramp_rate)

            # Check for motor stall
            if self._detect_motor_stall():
                if DEBUG:
                    self.gcode.respond_raw("Motor stall detected. Stopping pin operations.")
                return

            if self.is_pwm:
                self.mcu_pin.set_pwm(print_time, value)
            else:
                self.mcu_pin.set_digital(print_time, value)

            self.last_value = value
            self.last_print_time = print_time

            if self.resend_interval and self.resend_timer is None:
                if self.last_print_time + 1.0 > self.reactor.monotonic():
                    self.reactor.pause(1.0)
                self.resend_timer = self.reactor.register_timer(self._resend_current_val, self.reactor.NOW)

            if DEBUG:
                self.gcode.respond_raw(f"Pin set: value={value}, print_time={print_time}, last_value={self.last_value}")

        except Exception as e:
            if DEBUG:
                self.gcode.respond_raw(f"Error setting pin: {e}")

    def _detect_motor_stall(self):
        """
        Detects motor stall based on simulated load conditions and duty cycle changes.
        Returns True if a stall is detected.
        """
        try:
            # Simulated load condition
            stall_threshold = 0.9  # Fraction of scale considered as high load
            max_load_duration = 1.5  # Duration (in seconds) for sustained high load

            current_runtime = self.reactor.monotonic() - self.last_print_time

            if self.last_value >= self.scale * stall_threshold and current_runtime >= max_load_duration:
                if DEBUG:
                    self.gcode.respond_raw(
                        f"Motor stall detected: High load ({self.last_value/self.scale*100:.1f}%) sustained for {current_runtime:.2f}s."
                    )
                return True

            return False
        except Exception as e:
            if DEBUG:
                self.gcode.respond_raw(f"Error detecting motor stall: {e}")
            return False

    def _resend_current_val(self, eventtime):
        try:
            if self.last_value == self.shutdown_value:
                self.reactor.unregister_timer(self.resend_timer)
                self.resend_timer = None
                return self.reactor.NEVER

            systime = self.reactor.monotonic()
            print_time = self.mcu_pin.get_mcu().estimated_print_time(systime)
            time_diff = (self.last_print_time + self.resend_interval) - print_time

            if time_diff > 0.0:
                return systime + time_diff

            self._set_pin(print_time + PIN_MIN_TIME, self.last_value, True)
            return systime + self.resend_interval
        except Exception as e:
            if DEBUG:
                self.gcode.respond_raw(f"Error in resend_current_val: {e}")
            return self.reactor.NEVER
