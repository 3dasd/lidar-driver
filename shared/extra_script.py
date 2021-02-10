# This file is referenced from ../platformio.ini and is
# used to define a pre-upload hook that drives the Arduino
# RST pin when uploading from RPi over the GPIO serial
# connection (/deb/ttyS0)
#
# Based on: https://community.platformio.org/t/uploading-to-avr-using-raspberry-pi-gpio-avrdude-gpio-reset/360/37

import subprocess
import time

Import("env")

print("[DEBUG] extra_script.py is running")

def reset_pin_before_upload(source, target, env):
    print("[DEBUG] driving RST pin")
    subprocess.call(["gpio", "-g", "mode", "18", "out"])
    subprocess.call(["gpio", "-g", "write", "18", "0"])
    time.sleep(1)
    subprocess.call(["gpio", "-g", "write", "18", "1"])

env.AddPreAction("upload", reset_pin_before_upload)