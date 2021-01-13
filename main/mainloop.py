#! /usr/bin/env python
#
# MicroPython MainLoop for USMART Sensor Application.
#
# This file is part of micropython-usmart-sensor-mainloop
# https://github.com/bensherlock/micropython-usmart-sensor-mainloop
#
# Standard Interface for MainLoop
# - def run_mainloop() : never returns
#
# MIT License
#
# Copyright (c) 2020 Benjamin Sherlock <benjamin.sherlock@ncl.ac.uk>
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#

"""MicroPython MainLoop for USMART Sensor Application."""
import time
import machine
import micropython
import pyb

micropython.alloc_emergency_exception_buf(100)
# https://docs.micropython.org/en/latest/reference/isr_rules.html#the-emergency-exception-buffer

"""
from pybd_expansion.main.max3221e import MAX3221E
from pybd_expansion.main.powermodule import PowerModule
#from uac_modem.main.unm3networksimple import Nm3NetworkSimple
#import jotter
"""

import sensor_payload.main.sensor_payload as sensor_payload
from uac_modem.main.unm3driver import Nm3
from uac_localisation.main.LocalizationProcess import LocalizationProcess
from uac_localisation.main.misc.datetime import datetime, timedelta


USE_RTC_FOR_TIMING = False

# Flag to denote NM3 callback
_nm3_callback_flag = False

if USE_RTC_FOR_TIMING:
    # Initialize RTC module for time keeping
    rtc = machine.RTC()
    # rtc.init()  # reinitialise - there were bugs in firmware. This wipes the datetime.
    print("DateTime: ", rtc.datetime())

    # Global variables to record RTC timestamp
    _nm3_callback_useconds = None
    _nm3_callback_seconds = None
    _nm3_callback_minutes = None
    _nm3_callback_hours = None
    _nm3_callback_date = None
    _nm3_callback_month = None
    _nm3_callback_year = None
else:
    # Global variables to record SysTick timestamp
    time.read_ticks()
    print("Second:", time.ticks_seconds(), "Millis:", time.ticks_millis(), "Micro:", time.ticks_micros())

    # Global variables to record SysTimer timestamp
    _nm3_callback_secs = None
    _nm3_callback_millis = None
    _nm3_callback_micros = None


# NM3 Callback function
def nm3_callback(line):
    # NB: You cannot do anything that allocates memory in this interrupt handler.
    if USE_RTC_FOR_TIMING:
        global _nm3_callback_flag
        global _nm3_callback_useconds
        global _nm3_callback_seconds
        global _nm3_callback_minutes
        global _nm3_callback_hours
        global _nm3_callback_date
        global _nm3_callback_month
        global _nm3_callback_year
    else:
        global _nm3_callback_secs
        global _nm3_callback_millis
        global _nm3_callback_micros

    if USE_RTC_FOR_TIMING:
        # Get current time from RTC
        rtc.silent_read()
        _nm3_callback_useconds = rtc.useconds()
        _nm3_callback_seconds = rtc.seconds()
        _nm3_callback_minutes = rtc.minutes()
        _nm3_callback_hours = rtc.hours()
        _nm3_callback_date = rtc.date()
        _nm3_callback_month = rtc.month()
        _nm3_callback_year = rtc.year()
    else:
        # Get Timestamp from SysTick
        time.read_ticks()
        _nm3_callback_micros = time.ticks_micros()
        _nm3_callback_millis = time.ticks_millis()
        _nm3_callback_secs = time.ticks_seconds()

    # Indicate the main-loop for nm3_callback
    _nm3_callback_flag = True


# Standard Interface for MainLoop
# - def run_mainloop() : never returns
def run_mainloop():
    """Standard Interface for MainLoop. Never returns."""
    print("Entered in run_mainloop.")

    global _nm3_callback_flag

    if USE_RTC_FOR_TIMING:
        global _nm3_callback_useconds
        global _nm3_callback_seconds
        global _nm3_callback_minutes
        global _nm3_callback_hours
        global _nm3_callback_date
        global _nm3_callback_month
        global _nm3_callback_year
    else:
        global _nm3_callback_secs
        global _nm3_callback_millis
        global _nm3_callback_micros

    # Enable power supply to RS232 driver
    ldo_3V3 = machine.Pin('EN_3V3', mode=machine.Pin.OUT, value=1)
    # Switch OFF the External LDO-2 3.3V to Sensor Payload
    ldo2 = pyb.Pin('Y5', mode=pyb.Pin.OPEN_DRAIN, pull=None, value=0)

    # Set callback for nm3 pin change - line goes high on frame synchronisation
    # make sure it is clear first
    nm3_extint = pyb.ExtInt(pyb.Pin.board.Y3, pyb.ExtInt.IRQ_RISING, pyb.Pin.PULL_DOWN, None)
    nm3_extint = pyb.ExtInt(pyb.Pin.board.Y3, pyb.ExtInt.IRQ_RISING, pyb.Pin.PULL_DOWN, nm3_callback)

    # Serial Port/UART is opened with a 100ms timeout for reading - non-blocking.
    uart = machine.UART(1, 9600, bits=8, parity=None, stop=1, timeout=100)
    nm3_modem = Nm3(input_stream=uart, output_stream=uart)

    # Create an instance of External Sensor Payload
    extern_sensor = sensor_payload.get_external_sensor_payload_instance()

    # Create an instance of Localisation Process
    locator = LocalizationProcess()
    # Just for testing purpose, set the depth of the sensor node
    locator.sensor_depth = 100.0

    # Pass external sensor handle to locator
    locator.extern_sensor = extern_sensor

    while True:
        try:
            #print("Entered into forever loop.:")

            if _nm3_callback_flag:
                _nm3_callback_flag = False  # Clear flag
                # Packet incoming - although it may not be for us - try process for 2 seconds?
                start_millis = pyb.millis()

                while pyb.elapsed_millis(start_millis) < 2000:  # This will need to be changed
                    nm3_modem.poll_receiver()
                    nm3_modem.process_incoming_buffer()

                    if nm3_modem.has_received_packet():
                        message_packet = nm3_modem.get_received_packet()
                        # Copy the HW triggered timestamps over
                        if USE_RTC_FOR_TIMING:
                            message_packet.timestamp = (_nm3_callback_year, _nm3_callback_month,
                                                        _nm3_callback_date, _nm3_callback_hours,
                                                        _nm3_callback_minutes, _nm3_callback_seconds,
                                                        _nm3_callback_useconds)
                        else:
                            message_packet.timestamp = (_nm3_callback_secs, _nm3_callback_millis, _nm3_callback_micros)

                        # debug msg
                        # timetuple = message_packet.timestamp
                        # if USE_RTC_FOR_TIMING:
                        #     dt = datetime(timetuple[0], timetuple[1], timetuple[2], timetuple[3], timetuple[4], timetuple[5], timetuple[6])
                        #     timestamp = dt.timestamp()  # convert datetime to seconds.
                        # else:
                        #     timestamp = timetuple[0] + (timetuple[1] / 1E3) + (timetuple[2] / 1E6)

                        # print("\n")
                        # print("Received a packet at: ", timestamp)
                        # print("Received packet payload: ", message_packet.packet_payload)
                        # Convert packet_payload into string bytes
                        msg_bytes = b""
                        for item in message_packet.packet_payload:
                            msg_bytes += bytes([item])

                        # print("msg_bytes: ", msg_bytes)

                        # Pass the message packet on to the relevant submodule

                        # Check message payload header (first 2 bytes) to identify
                        # the purpose of message packet
                        msg_header = msg_bytes[0:2]
                        # print("msg_header: ", msg_header)
                        if msg_header[0:1] == b'U':
                            if msg_header[1:2] == b'L':
                                # Message for Localization Process
                                locator.handle_incoming_data_packet(message_packet)
                            elif msg_header[1:2] == b'N':
                                # Message for Networking Process
                                pass
                            elif msg_header[1:2] == b'M':
                                # Message for Miscellaneous Process
                                pass
                            else:
                                # Not recognized message
                                pass
                        else:
                            # This message packet is not for USMART neglect it
                            print("Not recognized packet header!")

            # =============== Prepare to sleep ==================== #
            # Power down 3V3 regulator
            # pyb.Pin.board.EN_3V3.off()
            # Sleep
            # machine.lightsleep()
            #print("Going to sleep...")

            time.sleep(1)
        except Exception as the_exception:
            import sys
            sys.print_exception(the_exception)
            pass
            # Log to file

# Run mainloop indefinitely
run_mainloop()

