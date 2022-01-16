#!/usr/bin/env python3

import rospy
from manual_node import ManualNode

from std_srvs.srv import Trigger


class ManualTriggerNode(ManualNode):
    def __init__(self):
        super().__init__("manual_trigger")

        target_srv = rospy.get_param("~target_srv")
        self.target_srv = rospy.ServiceProxy(target_srv, Trigger)

        rospy.spin()

    def on_data(self, last_button, button):
        if button and not last_button:
            self.target_srv()


if __name__ == "__main__":
    ManualTriggerNode()
