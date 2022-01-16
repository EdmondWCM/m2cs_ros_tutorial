import rospy
import time
from ds4cond import Ds4Cond

from m2_ps4.msg import Ps4Data


class ManualNode:
    def __init__(self, name):
        rospy.init_node(name, anonymous=True)

        self.last_key = None

        trigger_condition = rospy.get_param("~trigger_condition", None)
        self.trigger_condition = Ds4Cond(trigger_condition)

        self.trigger_key = rospy.get_param("~trigger_key")

        ps4_topic = rospy.get_param("~ps4_topic", "input/ps4_data")
        ps4_subs = []
        for topic in ps4_topic.split(","):
            ps4_subs.append(rospy.Subscriber(topic, Ps4Data, self.ps4_cb))

        time.sleep(2)  # wait for other nodes to be ready first

        self.trigger_condition.log(ps4_subs)

    def ps4_cb(self, data: Ps4Data):
        key = getattr(data, self.trigger_key)
        if self.last_key is not None and self.trigger_condition.test(data):
            self.on_data(self.last_key, key)
        self.last_key = key

    # pylint: disable=R0201
    def on_data(self, old, new):
        raise Exception("Subclass did not override on_data")
