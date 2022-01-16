#!/usr/bin/env python3

import rospy
import subprocess
from typing import Optional

from m2_ps4.msg import Ps4Data
from sensor_msgs.msg import Joy


class TransformDataNode:
    def __init__(self):
        rospy.init_node("transform_data_node")

        self.last_msg: Optional[Ps4Data] = None

        rospy.Subscriber(
            rospy.get_param("~input_topic", "joy"), Joy, self.joy_cb, queue_size=1
        )
        self.pub = rospy.Publisher(
            rospy.get_param("~ps4_topic", "/input/ps4_data"), Ps4Data, queue_size=1
        )

        # pylint: disable=broad-except
        try:
            kernel_version = subprocess.check_output(["uname", "-r"])
            if kernel_version.startswith(b"5."):
                minor = int(kernel_version.split(b".")[1])
                if minor < 12:
                    rospy.logwarn_once(
                        "WARNING: Kernel version does not support PS5 controllers. Please upgrade kernel. \
https://github.com/m2robocon/m2_wiki/wiki/udev-rules#upgrading-linux-kernel"
                    )
        except Exception:
            rospy.logerr("Failed to check kernel version")

        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            self.loop()
            rate.sleep()

    def loop(self):
        if self.last_msg is not None:
            self.pub.publish(self.last_msg)

    def joy_cb(self, msg: Joy):
        self.last_msg = transform(msg)


def transform(joy):
    data = Ps4Data(
        hat_lx=joy.axes[0],
        hat_ly=joy.axes[1],
        l2_analog=(-(joy.axes[2] - 1) / 2) if joy.buttons[6] else 0,
        hat_rx=joy.axes[3],
        hat_ry=joy.axes[4],
        r2_analog=(-(joy.axes[5] - 1) / 2) if joy.buttons[7] else 0,
        dpad_x=joy.axes[6],
        dpad_y=joy.axes[7],
        cross=joy.buttons[0],
        circle=joy.buttons[1],
        triangle=joy.buttons[2],
        square=joy.buttons[3],
        l1=joy.buttons[4],
        r1=joy.buttons[5],
        l2=joy.buttons[6],
        r2=joy.buttons[7],
        share=joy.buttons[8],
        options=joy.buttons[9],
        ps=joy.buttons[10],
        l3=joy.buttons[11],
        r3=joy.buttons[12],
    )
    return data


if __name__ == "__main__":
    TransformDataNode()
