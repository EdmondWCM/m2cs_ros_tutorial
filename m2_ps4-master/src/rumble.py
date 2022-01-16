#!/usr/bin/env python3

import os
from typing import Optional

from evdev import ecodes, InputDevice, ff
import rospy
from pyudev import Context

from m2_ps4.msg import Rumble


class RumbleNode:
    def __init__(self):
        rospy.init_node("rumble_node")

        topic = rospy.get_param("~topic", "rumble")

        # initialize tpad evdev
        dev_file = rospy.get_param("~dev", "/dev/input/js0_ff")

        if dev_file == "/dev/input/js0_ff":
            # search for possible ds4 devices
            rospy.logdebug(
                "no target controller specified, attempting to search connected ds4"
            )

            context = Context()
            for device in context.list_devices(subsystem="input"):
                if device.sys_name.startswith("event") and device.parent.get(
                    "NAME"
                ).endswith('Wireless Controller"'):
                    dev_file = "/dev/input/" + device.sys_name
                    break

            if dev_file is None:
                rospy.logwarn(
                    "no ds4 controllers are found and no target controller specified, aborting."
                )
                return

        if not os.path.exists(dev_file):
            rospy.logwarn("ff file {} missing".format(dev_file))
            return

        self.dev = InputDevice(dev_file)

        self.effect: Optional[int] = None
        self.upload(0, 0)

        rospy.Subscriber(
            topic, Rumble, lambda msg: self.upload(msg.strong, msg.weak), queue_size=1
        )

        rospy.on_shutdown(self.clear)

        rate = rospy.Rate(100)
        count = 0
        while not rospy.is_shutdown():
            # pylint: disable=no-member
            self.dev.write(ecodes.EV_FF, self.effect, 1)
            rate.sleep()
            count += 1

            # Prevent DS5 from occasionally sleeping
            if count > 1000:
                self.upload(self.strong, self.weak)

    def upload(self, strong: int, weak: int):
        if self.effect is not None:
            self.dev.erase_effect(self.effect)

        self.strong = strong
        self.weak = weak
        rumble = ff.Rumble(strong_magnitude=strong, weak_magnitude=weak)
        self.effect = self.dev.upload_effect(
            ff.Effect(
                # pylint: disable=no-member
                ecodes.FF_RUMBLE,
                -1,
                0,
                ff.Trigger(0, 0),
                ff.Replay(1000, 0),
                ff.EffectType(ff_rumble_effect=rumble),
            )
        )

    def clear(self):
        if self.effect is not None:
            self.dev.erase_effect(self.effect)


if __name__ == "__main__":
    RumbleNode()
