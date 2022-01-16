import os
import rospy
import subprocess

# pylint: disable=consider-using-with
DEVNULL = open(os.devnull, "wb")


class Pci:
    def __init__(self):
        self.js_path = rospy.get_param("~dev")
        self.pci = None
        self.uniq = None

    def check(self):
        try:
            cmd = subprocess.check_output(
                ["udevadm", "info", "-a", "-n", self.js_path],
                stderr=DEVNULL,
                universal_newlines=True,
            )
            pci, uniq = parse_udevadm(cmd)

            self.pci = pci
            self.uniq = uniq
        except subprocess.CalledProcessError as e:
            if not rospy.is_shutdown():
                rospy.logerr_throttle(
                    600, f"Udevadm returned with non-zero ({e.returncode}) exit status"
                )


def parse_udevadm(body):
    lines = body.split("\n")
    # DualShock 4 product ID is 054c:05c4(gen1)/054c:09cc(gen2)
    # Reference: https://wiki.gentoo.org/wiki/Sony_DualShock#Hardware
    # DualShock 5 product ID seems to be 054c:0ce6 experimentally.
    line_dev = next(
        line
        for line in lines
        if any(
            id in line.lower() for id in [":054c:05c4.", ":054c:09cc.", ":054c:0ce6."]
        )
        and "KERNELS==" in line
    )
    line_bat = next(line for line in lines if "attrs{uniq}==" in line.lower())
    return (
        line_dev[(line_dev.find('"') + 1) : (len(line_dev) - 1)],
        line_bat[(line_bat.find('"') + 1) : (len(line_bat) - 1)],
    )
