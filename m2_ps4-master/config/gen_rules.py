#!/usr/bin/env python3

import os
import yaml

import os

abspath = os.path.abspath(__file__)
dname = os.path.dirname(abspath)
os.chdir(dname)

with open("ds4-data.yml", "rb") as f:
    data = yaml.load(f, Loader=yaml.SafeLoader)
    for k in data:
        print("KERNEL==\"js*\", SUBSYSTEM==\"input\", ATTRS{uniq}==\"%s\", ATTRS{name}==\"*Wireless Controller\", SYMLINK+=\"%s\"" % (data[k], k))
        print("KERNEL==\"event*\", SUBSYSTEM==\"input\", ATTRS{uniq}==\"%s\", ATTRS{name}==\"*Wireless Controller\", MODE=\"0666\", SYMLINK+=\"%s_ff\"" % (data[k], k))
        print("KERNEL==\"event*\", SUBSYSTEM==\"input\", ATTRS{uniq}==\"%s\", ATTRS{name}==\"*Wireless Controller Touchpad\", MODE=\"0666\", SYMLINK+=\"%s_tpad\"" % (data[k], k))
        print("KERNEL==\"event*\", SUBSYSTEM==\"input\", ATTRS{uniq}==\"%s\", ATTRS{name}==\"*Wireless Controller Motion Sensors\", MODE=\"0666\", SYMLINK+=\"%s_motion\"" % (data[k], k))

print("KERNEL==\"0003:054C:05C4.*\", SUBSYSTEM==\"leds\", PROGRAM=\"/usr/bin/sudo -u root sh -c 'chmod -R 777 /sys%p'\"")
print("KERNEL==\"0005:054C:05C4.*\", SUBSYSTEM==\"leds\", PROGRAM=\"/usr/bin/sudo -u root sh -c 'chmod -R 777 /sys%p'\"")
print("KERNEL==\"0005:054C:09CC.*\", SUBSYSTEM==\"leds\", PROGRAM=\"/usr/bin/sudo -u root sh -c 'chmod -R 777 /sys%p'\"")

# symlink for events (testing)
#KERNEL=="event*", SUBSYSTEM=="input", ATTRS{uniq}=="8c:41:f2:8c:dd:a2", ATTRS{name}=="*Wireless Controller Touchpad", MODE="0666", SYMLINK+="ds4berry_tpad"
#KERNEL=="event*", SUBSYSTEM=="input", ATTRS{uniq}=="8c:41:f2:8c:dd:a2", ATTRS{name}=="*Wireless Controller Motion Sensors", MODE="0666", SYMLINK+="ds4berry_gyro"
#KERNEL=="event*", SUBSYSTEM=="input", ATTRS{uniq}=="8c:41:f2:8c:dd:a2", ATTRS{name}=="*Wireless Controller", MODE="0666", SYMLINK+="ds4berry_event"

# SYSFS devices that we want to utilize
#/sys/class/power_supply/sony_controller_battery_8c:41:f2:8c:dd:a2/capacity
#/sys/class/leds/0005:054C:05C4.0004:red/brightness
#/sys/class/leds/0005:054C:05C4.0004:green/brightness
#/sys/class/leds/0005:054C:05C4.0004:blue/brightness
#udevadm info --query=path /dev/input/js1 | rev | cut -d/ -f4 | rev

