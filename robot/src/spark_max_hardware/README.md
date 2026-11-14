# spark_max_hardware

ros2_control `SystemInterface` for REV SPARK MAX controllers over SocketCAN, with feedback.
Targeted at ROS 2 Jazzy (the `on_init` signature differs on newer distros).

## Status

| Part | State |
|---|---|
| SocketCAN open/send/receive (`can_socket.cpp`) | Implemented, generic Linux |
| ros2_control plumbing (lifecycle, interfaces, gear ratio, inversion, NaN handling) | Implemented |
| Feedback staleness check, neutral-on-deactivate, TX failure limit | Implemented |
| **SPARK MAX frame encode/decode** (`spark_max_can_protocol.cpp`) | **Not implemented, on purpose** |
| Simulation backend (`protocol=simulation`) | Implemented; integrates all commands into feedback state without CAN |
| `test_loopback` protocol + `test/fake_motor.py` | Invented frames for bench-testing the framework only |

With `protocol=spark_max`, `on_activate` refuses to start until `implemented()` returns true.

## Architecture

    controllers -> SparkMaxSystem -> SparkMaxProtocol (encode/decode) -> CanSocket -> can0

Everything that depends on the on-the-wire format sits behind `SparkMaxProtocol`.
Protocol code returns SI units at the motor shaft; `SparkMaxSystem` applies `gear_ratio` and `invert`.

## Parameters

Hardware: `can_interface` (default `can0`), `protocol` (`simulation` | `spark_max` | `test_loopback`),
`feedback_timeout_ms` (default 200, 0 disables).
Joint: `device_id` (required), `max_velocity`, `gear_ratio`, `invert`.
Command interfaces (exactly one per joint): `velocity` (converted to duty as cmd / max_velocity, open loop) or `duty_cycle`.
State interfaces: `position`, `velocity`, `current`, `temperature` (export what you declare).

## Failure behaviour

- No feedback from any motor within `feedback_timeout_ms` -> `read()` returns ERROR.
- 3 consecutive cycles with a failed CAN send -> `write()` returns ERROR.
- NaN/inf commands are treated as 0. On deactivate, a neutral (zero duty) command is sent.
- ROS-side zeroing cannot help if the bus itself is gone. What the controller does when commands stop
  is a property of the controller; establish it from documentation and test it before relying on it.

## CDR simulation

From the robot workspace, launch the complete ros2_control stack without CAN hardware:

    source /opt/ros/jazzy/setup.bash
    colcon build
    source install/setup.bash
    ros2 launch robot_launch robot.launch.py

The launch defaults to `protocol=simulation` and leaves Azure Kinect nodes disabled. Drive commands
use `/drive/cmd_vel` (`geometry_msgs/TwistStamped`); mechanism commands use
`/excavation/commands` with `[bucket, actuator_left, actuator_right]` and `/deposition/commands`
with `[conveyor]`. Select `protocol:=spark_max` only after the real CAN frames have been verified.

## Bench test without hardware (framework only)

    sudo modprobe vcan
    sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0
    python3 test/fake_motor.py --iface vcan0 --ids 1 2 3 4 5     # in one terminal
    candump vcan0                                                # to watch traffic

Set `can_interface=vcan0`, `protocol=test_loopback` in the xacro, launch, then publish to the
controllers. Run fake_motor with `--mute-after 5` to confirm the staleness path stops the hardware.

## Implementing the real protocol

Follow the checklist at the top of `src/spark_max_can_protocol.cpp`. Record the source of every constant
(document section or captured frame) and unit-test encode/decode against captured byte strings.

## Known limits

- `velocity` commands are open loop (duty = cmd / max_velocity). Measured velocity is reported but not
  used to close a loop here. Closing it needs either the controller's own velocity mode (add it to the
  protocol interface) or a PID layer in ROS.
- Classic CAN only (no CAN FD). Joint position starts from whatever the controller reports.
