#!/usr/bin/env python3

import rospy
from manual_node import ManualNode

from std_msgs.msg import Bool


class ManualIoNode(ManualNode):
    def __init__(self):
        super().__init__("manual_io")

        init_state = rospy.get_param("~init_state", "0")
        if init_state not in ["0", "1"]:
            rospy.logerr("Invalid init_state")
            return
        self.state = init_state == "1"

        getter_topic = rospy.get_param("~getter", None)
        if getter_topic is not None:
            self.getter = rospy.Publisher(getter_topic, Bool, queue_size=1)
        else:
            self.getter = None

        setter_topic = rospy.get_param("~setter", None)
        if setter_topic is not None:
            rospy.Subscriber(setter_topic, Bool, self.force_set, queue_size=1)

        target_topic = rospy.get_param("~target_topic")
        if target_topic is not None:
            self.target_pub = rospy.Publisher(target_topic, Bool, queue_size=1)
            self.target_pub.publish(data=init_state)
        else:
            self.target_pub = None

        repeat = rospy.get_param("~repeat_rate", None)
        if repeat is None:
            rospy.spin()
        else:
            rate = rospy.Rate(repeat)
            while not rospy.is_shutdown():
                self.flush()
                rate.sleep()

    def on_data(self, last_button, button):
        if button and not last_button:
            self.set_io(not self.state)

    def force_set(self, b: Bool):
        self.set_io(b.data)

    def set_io(self, b: bool):
        self.state = b
        self.target_pub.publish(data=b)
        if self.getter is not None:
            self.getter.publish(data=b)

    def flush(self):
        self.target_pub.publish(data=self.state)


if __name__ == "__main__":
    ManualIoNode()
