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

import pyb
import machine
import utime

from pybd_expansion.main.max3221e import MAX3221E
from pybd_expansion.main.powermodule import PowerModule
import sensor_payload.main.sensor_payload as sensor_payload

from uac_modem.main.unm3driver import MessagePacket, Nm3
from uac_modem.main.unm3networksimple import Nm3NetworkSimple

import jotter

import micropython
micropython.alloc_emergency_exception_buf(100)
# https://docs.micropython.org/en/latest/reference/isr_rules.html#the-emergency-exception-buffer


_rtc_callback_flag = False
def rtc_callback(unknown):
    # NB: You cannot do anything that allocates memory in this interrupt handler.
    global _rtc_callback_flag
    # RTC Callback function - Toggle LED
    pyb.LED(2).toggle()
    _rtc_callback_flag = True

_nm3_callback_flag = False
_nm3_callback_seconds = None
_nm3_callback_millis = None
_nm3_callback_micros = None
def nm3_callback(line):
    # NB: You cannot do anything that allocates memory in this interrupt handler.
    global _nm3_callback_flag
    global _nm3_callback_seconds
    global _nm3_callback_millis
    global _nm3_callback_micros
    # NM3 Callback function
    _nm3_callback_micros = pyb.micros()
    _nm3_callback_millis = pyb.millis()
    _nm3_callback_seconds = utime.time()
    _nm3_callback_flag = True




# Standard Interface for MainLoop
# - def run_mainloop() : never returns
def run_mainloop():
    """Standard Interface for MainLoop. Never returns."""

    global _rtc_callback_flag
    global _nm3_callback_flag
    global _nm3_callback_seconds
    global _nm3_callback_millis
    global _nm3_callback_micros

    # Set RTC to wakeup at a set interval
    rtc = pyb.RTC()
    rtc.init()  # reinitialise - there were bugs in firmware. This wipes the datetime.
    # A default wakeup to start with. To be overridden by network manager/sleep manager
    rtc.wakeup(1 * 60 * 1000, rtc_callback)  # in milliseconds

    # Enable the NM3 power supply on the powermodule
    powermodule = PowerModule()
    powermodule.enable_nm3()

    # Enable power supply to 232 driver
    pyb.Pin.board.EN_3V3.on()
    pyb.Pin('Y5', pyb.Pin.OUT, value=0)  # enable Y5 Pin as output
    max3221e = MAX3221E(pyb.Pin.board.Y5)
    max3221e.tx_force_off()  # Disable Tx Driver

    # Set callback for nm3 pin change - line goes high on frame synchronisation
    # make sure it is clear first
    nm3_extint = pyb.ExtInt(pyb.Pin.board.Y3, pyb.ExtInt.IRQ_FALLING, pyb.Pin.PULL_DOWN, None)
    nm3_extint = pyb.ExtInt(pyb.Pin.board.Y3, pyb.ExtInt.IRQ_FALLING, pyb.Pin.PULL_DOWN, nm3_callback)

    uart = machine.UART(1, 9600, bits=8, parity=None, stop=1, timeout=1000)
    nm3_modem = Nm3(uart)

    nm3_network = Nm3NetworkSimple(nm3_modem)
    gateway_address = 7


    while True:
        try:

            # The order and flow below will change.
            # However sending packets onwards to the submodules will occur on receipt of incoming messages.
            # At the moment the RTC is timed to wakeup, take a sensor reading, and send it to the gateway
            # via Nm3 as simple unicast before returning to sleep. This will be expanded to accommodate the network
            # protocol with wakeup, resynchronise on beacon, time offset for transmission etc.

            # Start of the wake loop
            # 1. Incoming NM3 MessagePackets (HW Wakeup)
            # 2. Periodic Sensor Readings (RTC)

            # Enable power supply to 232 driver
            pyb.Pin.board.EN_3V3.on()

            #
            # 1. Incoming NM3 MessagePackets (HW Wakeup)
            #
            # _nm3_callback_flag
            # _nm3_callback_datetime
            # _nm3_callback_millis - loops after 12.4 days. pauses during sleep modes.
            # _nm3_callback_micros - loops after 17.8 minutes. pauses during sleep modes.
            if _nm3_callback_flag:
                _nm3_callback_flag = False  # Clear flag
                jotter.get_jotter().jot("NM3 Hardware Flag set.", source_file=__name__)
                # Packet incoming - although it may not be for us - try process for 2 seconds?
                start_millis = pyb.millis()

                while pyb.elapsed_millis(start_millis) < 2000: # This will need to be changed

                    nm3_modem.poll_receiver()
                    nm3_modem.process_incoming_buffer()

                    if nm3_modem.has_received_packet():
                        message_packet = nm3_modem.get_received_packet()
                        # Copy the HW triggered timestamps over
                        message_packet.timestamp = utime.localtime(_nm3_callback_seconds)
                        message_packet.timestamp_millis = _nm3_callback_millis
                        message_packet.timestamp_micros = _nm3_callback_micros

                        # Send the packet on to the relevant sub module
                        # Network
                        # Localisation
                        # Other

            #
            # 2. Periodic Sensor Readings (RTC)
            #
            # _rtc_callback_flag
            # If is time to take a sensor reading (eg hourly)
            if _rtc_callback_flag:
                _rtc_callback_flag = False  # Clear the flag
                jotter.get_jotter().jot("RTC Flag set.", source_file=__name__)
                # Get from sensor payload: data as bytes
                sensor = sensor_payload.get_sensor_payload_instance()
                sensor.start_acquisition()
                while not sensor.is_completed():
                    sensor.process_acquisition()

                sensor_data_bytes = sensor.get_latest_data_as_bytes()
                message_bytes = bytes([ord('T'), ord('S')] + list(sensor_data_bytes))

                # Make up the payload and send via the Network to be relayed.
                # For now we will use the simple network that sends direct to gateway using unicast with ack and retries
                jotter.get_jotter().jot("Sending message to nm3_network.", source_file=__name__)
                max3221e.tx_force_on()  # Enable Tx Driver
                nm3_network.send_message(gateway_address, message_bytes, retries=3, timeout=5.0)
                max3221e.tx_force_off()  # Disable Tx Driver

            #
            # Prepare to sleep
            #

            # Power down 3V3 regulator
            pyb.Pin.board.EN_3V3.off()

            # Sleep
            # a. Light Sleep
            # pyb.stop()
            #_rtc_callback_flag = False  # Clear the callback flags
            #machine.lightsleep()
            # b. Deep Sleep - followed by hard reset
            # pyb.standby()
            # machine.deepsleep()
            # c. poll flag without sleeping
            while not _rtc_callback_flag:
                continue
            #_rtc_callback_flag = False

            #
            # Wake up
            #

            # RTC or incoming NM3 packet? Only way to know is by flags set in the callbacks.
            # machine.wake_reason() is not implemented on PYBD!
            # https://docs.micropython.org/en/latest/library/machine.html



        except Exception as the_exception:
            jotter.get_jotter().jot_exception(the_exception)
            pass
            # Log to file



