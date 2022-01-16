# m2_ps4

This package gathers the nodes that is ps4-related, including the config files and code.
Please setup the environment beforehand following the instructions [here](https://github.com/m2robocon/m2_wiki/wiki/Setting-up-the-working-environment#user-content-the-case-with-ds4-controller).

To read the ds4/ps4 as joystick in ros, `ros-noetic-joy` is required.
Linux kernel version also affects the joy readings, for details please check from https://wiki.gentoo.org/wiki/Sony_DualShock. The best approach is to follow the setup [here](https://github.com/m2robocon/m2_wiki/wiki/Setting-up-the-skull-canyon-&-network#user-content-linux-version).

Avoid using the `ps` button in other packages; it is reserved for PS4 topic mux.

## transform_data.py
- joy -> **this node** -> Ps4Data
- translates joy message into customised Ps4Data

### API Summary
- Parameters
	- `dev` (string, default `/dev/input/js0`), e.g. `/dev/ds5lambda`
- Subscriptions
	- `rosparam "~input_topic"` (default `/joy`): `sensor_msgs/Joy` Output of `joy_node`
- Publications
	- `rosparam "~ps4_topic"` (default `/input/ps4_data`): `Ps4Data` Transformed PS4 data
	- Guaranteed to be 100 Hz

## rumble.py
### API Summary
- Parameters
	- `dev` (string, default `/dev/input/js0_ff` autodetect), e.g. `/dev/ds5lambda_ff`
- Subscriptions
	- `rosparam "~topic"` (default `rumble`): `Rumble` Commands to set ruble state

## tpad.py
- read system event file and publish Ps4Tpad data
- operates on the event device (`evdev`) of the linux kernel
- can read single-touch or multi-touch data from controller

### API Summary
- Parameters
	- `dev` (string, default `/dev/input/js0_tpad` autodetect), e.g. `/dev/ds5lambda_tpad`
- Publications
	- `rosparam "~tpad_topic"` (default `ps4_tpad`): `Ps4Tpad` touchpad output

## motion.py
- read system event file and publish Ps4Motion data
- operates on the event device (`evdev`) of the linux kernel
- can read XYZ motion sensors data from DS4 Controller

### API Summary
- Parameters
	- `dev` (string, default `/dev/input/js0_tpad` autodetect), e.g. `/dev/ds5lambda_tpad`
- Publications
	- `rosparam "~motion_topic"` (default `ps4_motion`): `Ps4Motion` motion output

## `mux.launch`
Redirects PS4 input to different topics. Click the <kbd>PS</kbd> button to switch.

- Arguments
	- `topics`: Publishes `ChannelProvider`

### `channel_keepalive.py`
This adds a channel to the mux.

- Parameters
	- `provider_topic`: The mux topic, default `?/ps4/mux/topics`
	- `topic`: The output topic
	- `priority`: The priority of the channel
	- `color`: The corresponding LED color to set.

## Manual nodes
The `manual_*.py` files are utility nodes used to control particular topics.

### Common API summary
- Parameters
	- `ps4_topic`: string, default `/input/ps4_data`. Can combine multiple topics with `|`.
	- `trigger_condition`: string, default `null`
		- a list of string-separated `Ps4Data` boolean fields
		- each field can be prefixed with `!` to match the inverse.
	- `trigger_key`: the `Ps4Data` boolean field to activate the action.
	- `description`: the description of the action to display on roslog.
- Subscriptions
	- `rosparam ps4_topic`: the PS4 input to read from

### `manual_trigger.py`
Calls a `Trigger` service once when button is pressed down.

- Required services
	- `rosparam "~target_srv"`: `std_srvs/Trigger`, the service to call

### `manual_io.py`
Publishes a `Bool` topic regularly, toggling output upon button down.

- Parameters
	- `init_state`: Integer. The initial `0`/`1` value. Default `0`.
	- `getter`: optional string. Alternative, reader-friendly topic to publish to.
	- `setter`: optional string. A topic to override the user input.
- Subscriptions
	- `rosparam "~setter"`: `std_msgs/Bool`.
- Publications
	- `rosparam "~target_topic"`: `std_msgs/Bool`, the topic to publish to.
	- `rosparam "~getter"`: `std_msgs/Bool`.

### `manual_pos.py`
Publishes an `Int32`/`Float32` topic based on a float32 PS4 key.

- Parameters
	- `step_size`: number, the amount to change every unit change.
	- `target_setpoint`: The topic to publish to.
	- `target_feedback`: The topic to read feedback from, used for calibration after releasing butotn.
	- `target_type`: `Int32` or `Float32`. The target topic type.
	- `min`: The minimum value.
	- `max`: The maximum value.
	- `down_pause_time`: the number of seconds to pause upon button down after releasing button.
	- `function`: Sets the trend of step size change.
		- `continuous`: Subsequent frames after pause have constant delta.
			- Parameter `continuous_step_size`: Overrides the step size of subsequent frames. Default `step_size`.
				Set as 0 to make the function change once per click.
		- `geometric`: Subsequent frames have increasing delta.
			- Parameter `geometric_alpha`: Sets the increase per frame.

These nodes can be directly used in the launchfile by setting the appropriate conditions:

```xml
<node name="rotate" pkg="m2_ps4" type="manual_pos.py" output="screen">
	<param name="ps4_topic" value="/input/ps4_data" />
	<param name="trigger_condition" value="r1" />
	<param name="trigger_key" value="dpad_x" />
	<param name="step_size" value="20000" />
	<param name="target_setpoint" value="dji/p_setpoint" />
	<param name="target_feedback" value="dji/p_feedback/motor_enc" />
	<param name="description" value="Right shooter yaw" />
</node>
```

`trigger_condition` is a set of `bool` fields in `Ps4Data` joined by spaces.
`trigger_key` is a field in `Ps4Data` that triggers the action when `trigger_condition` is satisfied.
Multiple conditions in `trigger_condition` are joined by logical AND,
and can be inverted with the `!` operator,
e.g. `l1 !r1` means the action is only triggered when `l1` is pressed but `r1` is not.

## Message types
### Ps4Data.msg
- ps4 button and analog readings

### Ps4Tpad.msg
- DS4 touchpad single-touch or multi-touch readings
- `touch_*_x` and `touch_*_y` are normalized into floating numbers ranging from [0, 1]

### Ps4Motion.msg
- DS4 motion sensors data
- `(r)x`, `(r)y` and `(r)z` are normalized into floating numbers ranging from [0, 1]


## Serice types
### SetRgb.srv
- custom service message for carrying rgb intensities

## Web emulator
If a DS4 controller is not available,
use the `websim.py` node instead.
Simply `rosrun m2_ps4 websim.py` on the robot,
and visit <https://docs.m2stud.io/m2_ps4/> in your own browser
(on a computer with a physical keyboard).
Set the address to `ws://{ROS_IP}:22951` and press "Reconnect".
Click on the main textarea and press the keys indicated in the textarea.

## Quickblue
quickblue is an utility to connect to a controller in one line.
It is an executable python script located at [connect.py](./connect.py) of this repository,
but it is conventionally symlinked `ln -s ~/catkin_ws/src/m2_ps4/connect.py ~/quickblue`.

Simply write (a unique substring of) the devname of the controller as the first parameter,
e.g.

```
~/quickblue blue
```

(`blue` is a unique substring of the devname `ds4blue`)

This script is used for **both** disconnection and reconnection of a controller.
To disconnect, simply run the command directly with the connected controller.
To connect/reconnect, after running the command, long-press <kbd>Share</kbd> (left small button) + <kbd>PS</kbd>
until the LED of the controller blinks quickly.

### ps4_msg.launch
- launches joy_node from `ros-kinetic-joy`
- take care of device naming using arguments
- have default launch params for `joy_node`
