#!/usr/bin/env python3

import time
import rospy
from typing import Optional, Union
from manual_node import ManualNode

from std_msgs.msg import Float32, Int32


class ContinuousFunction:
    def __init__(self):
        self.step_size = float(rospy.get_param("~step_size"))
        self.continuous_step_size = float(
            rospy.get_param("~continuous_step_size", self.step_size)
        )
        self.first = True

    def sample(self, value: float) -> float:
        if self.first:
            self.first = False
            return self.step_size * value
        return self.continuous_step_size * value


class GeometricFunction:
    def __init__(self):
        self.step_size = float(rospy.get_param("~step_size"))
        self.alpha = rospy.get_param("~geometric_alpha")

    def sample(self, value: float) -> float:
        ret = self.step_size * value
        self.step_size *= self.alpha
        return ret


class PauseWrapper:
    def __init__(
        self, function: Union[ContinuousFunction, GeometricFunction], delay: int
    ):
        self.function = function
        self.count = 0
        self.delay = delay

    def sample(self, value: float) -> float:
        self.count += 1
        if self.count == 1 or self.delay < self.count:
            return self.function.sample(value)
        return 0.0


class ManualPosNode(ManualNode):
    def __init__(self):
        self.last_feedback: Optional[Union[int, float]] = None
        self.last_online = 0.0

        super().__init__("manual_pos")

        target_setpoint = rospy.get_param("~target_setpoint")
        target_feedback = rospy.get_param("~target_feedback", None)

        types = {"Int32": (Int32, int), "Float32": (Float32, lambda x: x)}
        ty = rospy.get_param("~target_type", "Int32")
        target_type, adapter = types[ty]
        self.adapter = adapter

        self.min = rospy.get_param("~min", None)
        self.max = rospy.get_param("~max", None)

        self.delay_ticks = int(rospy.get_param("~down_pause_time", 0.0) * 100)

        function = rospy.get_param("~function", "continuous")
        if function == "continuous":
            self.function = ContinuousFunction
        elif function == "geometric":
            self.function = GeometricFunction
        else:
            rospy.logerr(f"Unsupported function {function}")
            return

        self.current_function = None

        self.getter: Optional[rospy.Publisher] = None
        getter_topic = rospy.get_param("~getter", None)
        if getter_topic is not None:
            self.getter = rospy.Publisher(getter_topic, target_type, queue_size=1)

        self.target_pub = rospy.Publisher(target_setpoint, target_type, queue_size=1)
        if target_feedback is not None:
            rospy.Subscriber(
                target_feedback, target_type, self.feedback_cb, queue_size=1
            )
            self.self_feedback = False
        else:
            self.self_feedback = True
            self.last_feedback = rospy.get_param("~initial", 0.0)
            self.publish(self.last_feedback)

        setter_topic = rospy.get_param("~setter", None)
        if setter_topic is not None:
            rospy.Subscriber(setter_topic, target_type, self.force_set, queue_size=1)

        rospy.spin()

    def create_function(self) -> PauseWrapper:
        func = self.function
        return PauseWrapper(func(), self.delay_ticks)

    def on_data(self, _old, value):
        if self.last_feedback is None:
            rospy.logwarn_throttle(5, "Cannot set position without feedback")
            return

        if value == 0.0:
            self.current_function = None
            return

        if self.current_function is None:
            self.current_function = self.create_function()

        value = self.current_function.sample(value)
        output = self.last_feedback + value

        if self.min is not None and output < self.min:
            output = self.min
        if self.max is not None and output > self.max:
            output = self.max

        if output != self.last_feedback:
            self.publish(self.adapter(output))
            self.last_feedback = output
            self.last_online = time.monotonic()

    def feedback_cb(self, msg: Union[Int32, Float32]):
        if self.last_feedback is None or time.monotonic() - self.last_online > 0.5:
            self.last_feedback = msg.data

    def force_set(self, msg):
        self.last_feedback = msg.data
        self.publish(msg.data)

    def publish(self, value):
        self.target_pub.publish(data=value)
        if self.getter is not None:
            self.getter.publish(data=value)


if __name__ == "__main__":
    ManualPosNode()
