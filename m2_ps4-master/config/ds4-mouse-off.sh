#!/bin/bash
#id=` xinput | grep "Wireless Controller Touchpad" | cut -d= -f2 | cut -f 1`
#xinput set-prop $id "Device Enabled" 0

xinput | grep "Wireless Controller Touchpad" | cut -d= -f2 | cut -f 1 | xargs -0 -i -d '\n' xinput set-prop {} "Device Enabled" 0
