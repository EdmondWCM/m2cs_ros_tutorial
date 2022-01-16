import os
import subprocess

from distutils.core import setup

setup(version="0.0.1")

cwd = os.path.dirname(__file__)

subprocess.run(["sudo", "apt-get", "install", "-y", "ros-noetic-joy", "udev"], capture_output=True)
subprocess.run(["pip3", "install", "-r", "requirements.txt"], capture_output=True)
subprocess.run(["sudo", "sh", "-c", f"python3 {cwd}/config/gen_rules.py > /etc/udev/rules.d/51-ds4-setup.rules"], capture_output=True)
subprocess.run(["sudo", "udevadm", "control", "--reload"], capture_output=True)
subprocess.run(["sudo", "udevadm", "trigger"], capture_output=True)

try:
    os.symlink(os.path.join(cwd, "connect.py"), os.path.expanduser("~/quickblue"))
except FileExistsError:
    pass
