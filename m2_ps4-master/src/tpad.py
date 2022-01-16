#!/usr/bin/env python3

import os
import time

from evdev import ecodes, InputDevice, categorize
import rospy
from pyudev import Context

from m2_ps4.msg import Ps4Tpad


class TpadNode:
    def __init__(self):
        self.last_msg = Ps4Tpad()
        rospy.init_node("tpad_node")

        # initialize tpad evdev
        dev_file = rospy.get_param("~dev", "/dev/input/js0_tpad")

        if dev_file == "/dev/input/js0_tpad":
            # search for possible ds4 devices
            rospy.logdebug(
                "no target controller specified, attempting to search connected ds4"
            )

            context = Context()
            for device in context.list_devices(subsystem="input"):
                if device.sys_name.startswith("event") and device.parent.get(
                    "NAME"
                ).endswith('Wireless Controller Touchpad"'):
                    dev_file = "/dev/input/" + device.sys_name
                    break

            if dev_file is None:
                rospy.logwarn(
                    "no touchpads of ds4 controllers are found and no target controller specified, aborting."
                )
                return

        wait_time = 1
        while not rospy.is_shutdown() and not os.path.exists(dev_file):
            rospy.logwarn("tpad file {} missing".format(dev_file))
            time.sleep(wait_time)
            wait_time = min(wait_time * 2, 10)

        self.dev = InputDevice(dev_file)

        self.abs_info = dict(self.dev.capabilities()[ecodes.EV_ABS])

        # publisher to publish tpad events
        pub_topic = rospy.get_param("~tpad_topic", "ps4_tpad")
        self.pub = rospy.Publisher(pub_topic, Ps4Tpad, queue_size=1000)

        # set up polling rate as 100
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            # read event one by one
            event = self.dev.read_one()

            # process only if there is an event
            if event is not None:
                # categorize the event
                decoded = categorize(event)

                # process key events
                if event.type == ecodes.EV_KEY:
                    if "BTN_LEFT" in decoded.keycode or "BTN_MOUSE" in decoded.keycode:
                        self.last_msg.pressed = event.value == 1
                    elif "BTN_TOUCH" == decoded.keycode:
                        self.last_msg.touch_down = event.value == 1
                    # elif "BTN_TOOL_FINGER" == decoded.keycode:
                    #     self.last_msg.finger_down = event.value == 1
                    elif "BTN_TOOL_DOUBLETAP" == decoded.keycode:
                        self.last_msg.double_tapped = event.value == 1
                # process absolute axis events
                elif event.type == ecodes.EV_ABS:
                    if event.code == ecodes.ABS_MT_TRACKING_ID:
                        self.last_msg.mt_tracking_id = event.value
                    if event.code == ecodes.ABS_MT_POSITION_X:
                        self.last_msg.touch_2_x = self.normalize(
                            ecodes.ABS_MT_POSITION_X, event.value
                        )
                    elif event.code == ecodes.ABS_MT_POSITION_Y:
                        self.last_msg.touch_2_y = self.normalize(
                            ecodes.ABS_MT_POSITION_Y, event.value
                        )
                    elif event.code == ecodes.ABS_X:
                        self.last_msg.touch_1_x = self.normalize(
                            ecodes.ABS_X, event.value
                        )
                    elif event.code == ecodes.ABS_Y:
                        self.last_msg.touch_1_y = self.normalize(
                            ecodes.ABS_Y, event.value
                        )

                # publish to topic
                self.pub.publish(self.last_msg)
            else:
                # sleep if there are no events
                rate.sleep()

    def normalize(self, event_code, event_value):
        abs_info = self.abs_info[event_code]
        min = abs_info.min
        max = abs_info.max
        return (event_value - min) / (max - min)


if __name__ == "__main__":
    TpadNode()
