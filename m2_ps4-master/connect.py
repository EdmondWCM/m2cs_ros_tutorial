#!/usr/bin/env python3

import os
import subprocess
import sys
import time
import yaml

path = os.path.join(os.path.dirname(os.path.realpath(__file__)), "config/ds4-data.yml")
with open(path, "rb") as f:
    data = yaml.load(f, Loader=yaml.SafeLoader)

try:
    name = sys.argv[1]
    addrs = [(k, v) for k, v in data.items() if name in k]
    if len(addrs) != 1:
        print(f"Name mismatch: {addrs}")
        sys.exit(1)
    name, addr = addrs[0]
except IndexError:
    import inquirer
    name = inquirer.prompt([inquirer.List("name",
        message="Choose a controller", choices=list(data))])["name"]
    addr = data[name]
print(f"Trying to connect to {name} {addr}")

subprocess.run(["bluetoothctl", "remove", addr])
while True:
    try:
        subprocess.run(["bluetoothctl", "scan", "on"], timeout=5)
    except subprocess.TimeoutExpired:
        pass
    result = subprocess.run(["bluetoothctl", "pair", addr])
    if result.returncode == 0:
        break
    time.sleep(1)

subprocess.run(["bluetoothctl", "connect", addr], check=True)
subprocess.run(["bluetoothctl", "trust", addr], check=True)
