# Support for "neopixel" leds
#
# Copyright (C) 2019-2022  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import logging
import time
from . import led

BACKGROUND_PRIORITY_CLOCK = 0x7fffffff00000000
BIT_MAX_TIME = .000004
RESET_MIN_TIME = .000050
MAX_MCU_SIZE = 500  # Sanity check on LED chain length
RETRY_LIMIT = 3
RETRY_DELAY = 0.1  # 100ms delay between retries


class AFCled:
    def __init__(self, config):
        self.printer = printer = config.get_printer()
        self.mutex = printer.get_reactor().mutex()
        # Configure neopixel
        ppins = printer.lookup_object('pins')
        pin_params = ppins.lookup_pin(config.get('pin'))
        self.mcu = pin_params['chip']
        self.oid = self.mcu.create_oid()
        self.pin = pin_params['pin']
        self.mcu.register_config_callback(self.build_config)
        self.neopixel_update_cmd = self.neopixel_send_cmd = None
        # Build color map
        chain_count = config.getint('chain_count', 1, minval=1)
        color_order = config.getlist("color_order", ["GRB"])
        if len(color_order) == 1:
            color_order = [color_order[0]] * chain_count
        if len(color_order) != chain_count:
            raise config.error("color_order does not match chain_count")
        color_indexes = []
        for lidx, co in enumerate(color_order):
            if sorted(co) not in (sorted("RGB"), sorted("RGBW")):
                raise config.error("Invalid color_order '%s'" % (co,))
            color_indexes.extend([(lidx, "RGBW".index(c)) for c in co])
        self.color_map = list(enumerate(color_indexes))
        if len(self.color_map) > MAX_MCU_SIZE:
            raise config.error("neopixel chain too long")
        # Initialize color data
        self.led_helper = led.LEDHelper(config, self.update_leds, chain_count)
        self.color_data = bytearray(len(self.color_map))
        self.update_color_data(self.led_helper.get_status()['color_data'])
        self.old_color_data = bytearray([d ^ 1 for d in self.color_data])
        # Register callbacks
        printer.register_event_handler("klippy:connect", self.send_data)

    def build_config(self):
        bmt = self.mcu.seconds_to_clock(BIT_MAX_TIME)
        rmt = self.mcu.seconds_to_clock(RESET_MIN_TIME)
        self.mcu.add_config_cmd("config_neopixel oid=%d pin=%s data_size=%d"
                                " bit_max_ticks=%d reset_min_ticks=%d"
                                % (self.oid, self.pin, len(self.color_data),
                                   bmt, rmt))
        cmd_queue = self.mcu.alloc_command_queue()
        self.neopixel_update_cmd = self.mcu.lookup_command(
            "neopixel_update oid=%c pos=%hu data=%*s", cq=cmd_queue)
        self.neopixel_send_cmd = self.mcu.lookup_query_command(
            "neopixel_send oid=%c", "neopixel_result oid=%c success=%c",
            oid=self.oid, cq=cmd_queue)

    def update_color_data(self, led_state):
        color_data = self.color_data
        for cdidx, (lidx, cidx) in self.color_map:
            color_data[cdidx] = int(led_state[lidx][cidx] * 255. + .5)

    def send_data(self, print_time=None):
        old_data, new_data = self.old_color_data, self.color_data
        if new_data == old_data:
            return
        # Find the position of all changed bytes in this framebuffer
        diffs = [[i, 1] for i, (n, o) in enumerate(zip(new_data, old_data))
                 if n != o]
        # Batch together changes that are close to each other
        for i in range(len(diffs)-2, -1, -1):
            pos, count = diffs[i]
            nextpos, nextcount = diffs[i+1]
            if pos + 5 >= nextpos and nextcount < 16:
                diffs[i][1] = nextcount + (nextpos - pos)
                del diffs[i+1]
        # Transmit changes
        ucmd = self.neopixel_update_cmd.send
        for pos, count in diffs:
            for attempt in range(RETRY_LIMIT):
                try:
                    ucmd([self.oid, pos, new_data[pos:pos+count]],
                         reqclock=BACKGROUND_PRIORITY_CLOCK)
                    break
                except Exception as e:
                    logging.error(f"Retry {attempt + 1}/{RETRY_LIMIT} for updating data failed: {e}")
                    time.sleep(RETRY_DELAY)
            else:
                logging.error(f"Failed to update neopixel data after {RETRY_LIMIT} attempts.")
                return

        # Instruct mcu to update the LEDs
        minclock = 0
        if print_time is not None:
            minclock = self.mcu.print_time_to_clock(print_time)
        scmd = self.neopixel_send_cmd.send
        for attempt in range(RETRY_LIMIT):
            try:
                params = scmd([self.oid], minclock=minclock,
                              reqclock=BACKGROUND_PRIORITY_CLOCK)
                if params['success']:
                    return
            except Exception as e:
                logging.error(f"Retry {attempt + 1}/{RETRY_LIMIT} for sending data failed: {e}")
                time.sleep(RETRY_DELAY)
        logging.error("Neopixel update did not succeed after retries.")

    def update_leds(self, led_state, print_time):
        def reactor_bgfunc(eventtime):
            with self.mutex:
                self.update_color_data(led_state)
                self.send_data(print_time)
        self.printer.get_reactor().register_callback(reactor_bgfunc)

    def get_status(self, eventtime=None):
        return self.led_helper.get_status(eventtime)

    def led_change(self, index, status):
        colors = list(map(float, status.split(',')))
        transmit = 1

        def lookahead_bgfunc(print_time):
            if hasattr(self.led_helper, "_set_color"):
                set_color_fn = self.led_helper._set_color
                check_transmit_fn = self.led_helper._check_transmit
            else:
                set_color_fn = self.led_helper.set_color
                check_transmit_fn = self.led_helper.check_transmit
            set_color_fn(index, colors)
            if transmit:
                check_transmit_fn(print_time)

        toolhead = self.printer.lookup_object('toolhead')
        toolhead.register_lookahead_callback(lookahead_bgfunc)


def load_config_prefix(config):
    return AFCled(config)
