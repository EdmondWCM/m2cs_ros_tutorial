#!/usr/bin/env python3

import rospy
from pci import Pci

from std_msgs.msg import Int32


class BatteryNode:
    def __init__(self):
        rospy.init_node("battery_node")

        topic = rospy.get_param("~topic", "battery_level")
        self.pub = rospy.Publisher(topic, Int32, queue_size=1)

        self.pci = Pci()

        self.battery_level = None

        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.loop()
        rate.sleep()

    def loop(self):
        self.pci.check()
        if self.pci.uniq is not None:
            path1 = f"/sys/class/power_supply/sony_controller_battery_{self.pci.uniq}/capacity"
            path2 = f"/sys/class/power_supply/ps-controller-battery-{self.pci.uniq}/capacity"

            # pylint: disable=broad-except
            try:
                try:
                    with open(path1) as fh:
                        battery_level = int(fh.read())
                except (IOError, ValueError):
                    with open(path2) as fh:
                        battery_level = int(fh.read())
            except Exception as e:
                rospy.logerr_throttle(60, f"Error reading battery level: {e}")
                battery_level = None

            if battery_level is not None and battery_level != self.battery_level:
                self.battery_level = battery_level
                rospy.loginfo(f"Battery level: {battery_level}%")

            self.pub.publish(battery_level)

            if battery_level is not None and battery_level <= 20:
                rospy.logwarn_throttle(
                    10, f"Battery level critically low: {battery_level}%"
                )

        else:
            rospy.logerr_throttle(
                60, "Batery level cannot be read: device PCI not detected"
            )


if __name__ == "__main__":
    BatteryNode()
