#!/usr/bin/env python3

import os

from evdev import ecodes, InputDevice, categorize
import rospy
from pyudev import Context

from m2_ps4.msg import Ps4Motion


class MotionNode:
    def __init__(self):
        self.last_msg = Ps4Motion()
        rospy.init_node("motion_node")

        # initialize tpad evdev
        dev_file = rospy.get_param("~dev", "/dev/input/js0_motion")

        if dev_file == "/dev/input/js0_motion":
            # search for possible ds4 devices
            rospy.logdebug(
                "no target controller specified, attempting to search connected ds4"
            )

            context = Context()
            for device in context.list_devices(subsystem="input"):
                if device.sys_name.startswith("event") and device.parent.get(
                    "NAME"
                ).endswith('Wireless Controller Motion Sensors"'):
                    dev_file = "/dev/input/" + device.sys_name
                    break

            if dev_file is None:
                rospy.logwarn(
                    "no motion sensors of ds4 controllers are found and no target controller specified, aborting."
                )
                return

        if not os.path.exists(dev_file):
            rospy.logwarn("motion sensors file {} missing".format(dev_file))
            return
        self.dev = InputDevice(dev_file)

        self.abs_info = dict(self.dev.capabilities()[ecodes.EV_ABS])

        # publisher to publish motion events
        pub_topic = rospy.get_param("~motion_topic", "ps4_motion")
        self.pub = rospy.Publisher(pub_topic, Ps4Motion, queue_size=1000)

        # set up polling rate as 100
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            # read event one by one
            event = self.dev.read_one()

            # process only if there is an event
            if event is not None:
                # categorize the event
                decoded = categorize(event)

                # process absolute axis events
                if event.type == ecodes.EV_ABS:
                    if event.code == ecodes.ABS_X:
                        self.last_msg.x = self.normalize(ecodes.ABS_X, event.value)
                    elif event.code == ecodes.ABS_Y:
                        self.last_msg.y = self.normalize(ecodes.ABS_Y, event.value)
                    elif event.code == ecodes.ABS_Z:
                        self.last_msg.z = self.normalize(ecodes.ABS_Z, event.value)
                    elif event.code == ecodes.ABS_RX:
                        self.last_msg.rx = self.normalize(ecodes.ABS_RX, event.value)
                    elif event.code == ecodes.ABS_RY:
                        self.last_msg.ry = self.normalize(ecodes.ABS_RY, event.value)
                    elif event.code == ecodes.ABS_RZ:
                        self.last_msg.rz = self.normalize(ecodes.ABS_RZ, event.value)

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
    MotionNode()
