import re
import rospy
from typing import List, Union

from m2_ps4.msg import Ps4Data


class Ds4Cond:
    """Ps4Data condition string parser"""

    # pylint: disable=used-before-assignment
    def __init__(self, string: str):
        self.conds = []
        if string is None:
            return

        for cond in re.split(r"[\+\& ]+", string):
            target = True
            if cond.startswith("!"):
                target = False
                cond = cond[1:]
            self.conds.append((cond, target))

    def test(self, data: Ps4Data) -> bool:
        for key, target in self.conds:
            if getattr(data, key) != target:
                return False

        return True

    def str(self) -> str:
        return " + ".join(
            (target if cond else f"not {target}") for target, cond in self.conds
        )

    def __str__(self):
        return self.str()

    def log(self, sub: Union[rospy.Subscriber, List[rospy.Subscriber]]):
        if sub.__class__ is list:
            topic = "|".join(sub_item.resolved_name for sub_item in sub)
        else:
            topic = sub.resolved_name
        trigger_key = rospy.get_param("~trigger_key")
        description = rospy.get_param("~description", "")
        trigger = f"{self} + {trigger_key}" if len(self.conds) > 0 else trigger_key
        rospy.loginfo(f"[PS4 action] {topic}: {trigger} = {description}")
