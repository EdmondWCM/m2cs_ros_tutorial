#!/usr/bin/env python3

import rospy

from m2_ps4.msg import ChannelProvider
from option_mux import parse_rgb


class ChannelProviderNode:
    def __init__(self):
        rospy.init_node("channel_provider_node", anonymous=True)

        provider_topic = rospy.get_param("~provider_topic", "/ps4/mux/topics")
        topic = rospy.get_param("~topic")
        priority = rospy.get_param("~priority")
        color = rospy.get_param("~color")
        if isinstance(color, int):
            raise Exception("Use # in front of color")
        color = parse_rgb(color)

        rospy.loginfo(f"Publishing to {provider_topic}")
        pub = rospy.Publisher(provider_topic, ChannelProvider, queue_size=1)
        msg = ChannelProvider(topic=topic, priority=priority, color=color)

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            pub.publish(msg)
            rate.sleep()


if __name__ == "__main__":
    ChannelProviderNode()
