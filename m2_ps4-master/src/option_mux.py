#!/usr/bin/env python3

from itertools import chain
from time import monotonic_ns

from typing import List, Optional

import rospy
from m2_ps4.msg import ChannelProvider, Ps4Data, RgbTime
from m2_ps4.srv import SetRgb
from std_msgs.msg import Int32


class Option:
    def __init__(self, name: str, pub: rospy.Publisher, priority: int, color: RgbTime):
        self.name = name
        self.pub = pub
        self.priority = priority
        self.color = color
        self.timeout = monotonic_ns()

    def touch(self):
        self.timeout = monotonic_ns()

    def __str__(self):
        return f"option {self.name}"


class OptionMux:
    def __init__(self):
        rospy.init_node("ds4_option_mux")
        self.last: Optional[Ps4Data] = None

        self.set_color = rospy.ServiceProxy("/set_led", SetRgb)
        self.set_color.wait_for_service()

        self.choose(None)

        options = rospy.get_param("/ds4_option_mux/topics", "?/ps4/mux/topics")
        self.options: List[Option] = []
        if options[0] == "?":
            self.options_topic: Optional[str] = options[1:]
            rospy.Subscriber(self.options_topic, ChannelProvider, self.channel_acceptor)
        else:
            self.options_topic: Optional[str] = None
            colors = rospy.get_param("~channel_colors", "")
            colors: Optional[List[str]] = colors.split(",") if colors != "" else None

            options = options.split(",")
            for i, option in enumerate(options):
                pub = rospy.Publisher(option, Ps4Data, queue_size=1)
                color = parse_rgb(colors[i]) if colors is not None else None
                self.options.append(Option(option, pub, i, color))

        channel_topic = rospy.get_param("~channel_topic", "")
        if channel_topic != "":
            self.channel_pub = rospy.Publisher(channel_topic, Int32, queue_size=1)
        else:
            self.channel_pub = None

        rospy.Subscriber("/ps4/unmuxed", Ps4Data, self.raw_cb, queue_size=1)

        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            rate.sleep()
            if self.options_topic is not None:
                self.topic_cleanup()

    def channel_acceptor(self, msg: ChannelProvider):
        for option in self.options:
            if option.name == msg.topic:
                option.touch()
                return

        # It is unreasonable to set topic as relative namespace
        topic = msg.topic
        if not topic.startswith("/"):
            topic = "/" + topic

        option = Option(
            topic,
            rospy.Publisher(msg.topic, Ps4Data, queue_size=1),
            msg.priority,
            msg.color,
        )
        index = sum(1 for option in self.options if option.priority < msg.priority)
        self.options.insert(index, option)
        rospy.logwarn(f"[DS4 OptionMux] Registered new muxed channel {msg.topic}")
        self.choose(0)

    def topic_cleanup(self):
        removals: List[int] = []
        for i, option in enumerate(self.options):
            if monotonic_ns() - option.timeout > 1e9:  # dead for one second
                rospy.logwarn(f"PS4 channel {self.options[i]} timeout, deleted")
                removals.append(i)

        removals.reverse()
        for i in removals:
            self.options.pop(i)
            if self.choice == i:
                self.choose(None)
                rospy.logwarn("Active PS4 channel timeout and removed")

    def raw_cb(self, data: Ps4Data):
        last = self.last
        self.last = data

        if last is None:
            return

        if data.ps and not last.ps:
            choice = (self.choice + 1) if self.choice is not None else 0
            if choice >= len(self.options):
                choice = 0
            self.choose(choice)

        if not data.ps and self.choice is not None:
            self.options[self.choice].pub.publish(data)

    def choose(self, choice: Optional[int]):
        self.choice = choice
        if self.choice is None:
            rgb_sequence = [
                RgbTime(255, grad * 255 // 10, grad * 255 // 10, 0.1)
                for grad in chain(range(0, 10), range(10, 0, -1))
            ]
            self.set_color(random=False, rgb_sequence=rgb_sequence)
            rospy.logwarn("[DS4 OptionMux] No channels alive")
        else:
            if self.channel_pub is not None:
                self.channel_pub.publish(self.choice)
            if self.options[self.choice].color is not None:
                self.set_color(
                    random=False, rgb_sequence=[self.options[self.choice].color]
                )
            rospy.logwarn(f"[DS4 OptionMux] Switched to {self.options[self.choice]}")


def parse_rgb(string: str) -> RgbTime:
    if string.startswith("#"):
        string = string[1:]
    r = string[0:2]
    g = string[2:4]
    b = string[4:6]
    return RgbTime(int(r, 16), int(g, 16), int(b, 16), 1)


if __name__ == "__main__":
    OptionMux()
