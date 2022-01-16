#!/usr/bin/env python3

import random
import rospy
from pci import Pci
from typing import List

from m2_ps4.msg import RgbTime
from m2_ps4.srv import *


class LedNode:
    def __init__(self):
        rospy.init_node("led_node")

        self.pci = Pci()

        self.led_state = led_random_state(
            range(0, 64), range(0, 64), range(0, 64), 0.1, 0.9
        )

        rospy.Service("/set_led", SetRgb, self.set_led_cb)
        rospy.on_shutdown(self.on_shutdown)

        detect_pci_loop = 0
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            (r, g, b, t) = next(self.led_state)
            self.set_led("red", r)
            self.set_led("green", g)
            self.set_led("blue", b)

            iterations = int(t * 100)
            while iterations > 0:
                state_before_sleep = self.led_state
                iterations -= 1
                rate.sleep()

                # Retry detect_pci() in case it reconnected
                detect_pci_loop += 1
                if detect_pci_loop >= 100:  # every second
                    self.pci.check()
                    detect_pci_loop = 0

                if state_before_sleep is not self.led_state:
                    break  # LED pattern updated, fetch next state immediately

    def on_shutdown(self):
        self.set_led("red", 10)
        self.set_led("green", 0)
        self.set_led("blue", 0)

    def set_led(self, channel, value):
        pci = self.pci.pci
        if pci is not None:
            # pylint: disable=broad-except
            try:
                with open(f"/sys/class/leds/{pci}:{channel}/brightness", "w") as fh:
                    fh.write(str(value))
            except StopIteration:
                rospy.logerr_throttle(600, "Cannot find PCI kernel attribute")
            except Exception as e:
                rospy.logerr_throttle(600, f"Error writing LED: {e}")

    def set_led_cb(self, req: SetRgbRequest):
        if req.random:
            seq = req.rgb_sequence
            if len(seq) == 0:
                self.led_state = led_random_state(
                    range(0, 256), range(0, 256), range(0, 256), 1, 1
                )
            elif len(seq) == 1:
                self.led_state = led_random_state(
                    range(0, 256),
                    range(0, 256),
                    range(0, 256),
                    seq[0].seconds,
                    seq[0].seconds,
                )
            elif len(seq) == 2:
                self.led_state = led_random_state(
                    range(seq[0].red, seq[1].red),
                    range(seq[0].green, seq[1].green),
                    range(seq[0].blue, seq[1].blue),
                    seq[0].seconds,
                    seq[1].seconds,
                )
        else:
            if req.rgb_sequence == 0:
                return {}
            self.led_state = led_seq_state(req.rgb_sequence)

        return {}


def led_seq_state(seq: List[RgbTime]):
    while True:
        for rgbt in seq:
            yield (rgbt.red, rgbt.green, rgbt.blue, rgbt.seconds)


def led_random_state(
    range_r: range, range_g: range, range_b: range, t0: float, t1: float
):
    while True:
        yield (
            random.choice(range_r),
            random.choice(range_g),
            random.choice(range_b),
            random.randint(int(t0 * 10), int(t1 * 10)) / 10.0,
        )


if __name__ == "__main__":
    LedNode()
