import math

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray


class DriverConsole(Node):
    """Translate an Xbox-style joystick into rover commands."""

    COMMAND_PERIOD_SECONDS = 0.02
    JOY_TIMEOUT_SECONDS = 0.25
    MAX_LINEAR_SPEED_M_S = 0.5

    def __init__(self) -> None:
        super().__init__("operator_interface")

        self.declare_parameter("joy_topic", "/joy")
        self.declare_parameter("cmd_vel_topic", "/drive/cmd_vel")
        self.declare_parameter("max_linear_speed_m_s", 0.5)
        self.declare_parameter("max_angular_speed_rad_s", 0.5)
        self.declare_parameter("deadband", 0.08)
        self.declare_parameter("dpad_deadband", 0.1)
        self.declare_parameter("axis_linear", 1)
        self.declare_parameter("axis_angular", 3)
        self.declare_parameter("axis_dpad_vertical", 7)
        self.declare_parameter("button_bucket_reverse", 4)  # LB
        self.declare_parameter("axis_bucket_forward", 2)  # LT
        self.declare_parameter("axis_deposition", 5)  # RT
        self.declare_parameter("invert_linear", True)
        self.declare_parameter("invert_angular", False)

        joy_topic = self.get_parameter("joy_topic").value
        cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        configured_linear_speed = float(
            self.get_parameter("max_linear_speed_m_s").value
        )
        self.max_linear_speed = min(
            max(configured_linear_speed, 0.0), self.MAX_LINEAR_SPEED_M_S
        )
        self.max_angular_speed = max(
            float(self.get_parameter("max_angular_speed_rad_s").value), 0.0
        )
        self.deadband = min(
            max(float(self.get_parameter("deadband").value), 0.0), 0.99
        )
        self.dpad_deadband = min(
            max(float(self.get_parameter("dpad_deadband").value), 0.0), 0.99
        )
        self.axis_linear = int(self.get_parameter("axis_linear").value)
        self.axis_angular = int(self.get_parameter("axis_angular").value)
        self.axis_dpad_vertical = int(
            self.get_parameter("axis_dpad_vertical").value
        )
        self.button_bucket_reverse = int(
            self.get_parameter("button_bucket_reverse").value
        )
        self.axis_bucket_forward = int(
            self.get_parameter("axis_bucket_forward").value
        )
        self.axis_deposition = int(
            self.get_parameter("axis_deposition").value
        )
        self.invert_linear = bool(self.get_parameter("invert_linear").value)
        self.invert_angular = bool(self.get_parameter("invert_angular").value)

        self._joy_data: Joy | None = None
        self._last_joy_time = None

        self._drive_pub = self.create_publisher(TwistStamped, cmd_vel_topic, 10)
        self._excavation_pub = self.create_publisher(
            Float64MultiArray, "/excavation/commands", 10
        )
        self._deposition_pub = self.create_publisher(
            Float64MultiArray, "/deposition/commands", 10
        )
        self._joy_sub = self.create_subscription(Joy, joy_topic, self._joy_callback, 10)
        self._timer = self.create_timer(
            self.COMMAND_PERIOD_SECONDS, self._publish_commands
        )

        if configured_linear_speed > self.MAX_LINEAR_SPEED_M_S:
            self.get_logger().warning(
                "max_linear_speed_m_s is capped at %.2f m/s",
                self.MAX_LINEAR_SPEED_M_S,
            )
        self.get_logger().info("Driver console started")
        self.get_logger().info(
            "Drive: left stick vertical / right stick horizontal; "
            "bucket: LB reverse, LT forward; actuators: D-pad up/down; "
            "conveyor: hold RT"
        )

    def _joy_callback(self, msg: Joy) -> None:
        self._joy_data = msg
        self._last_joy_time = self.get_clock().now()

    def _publish_commands(self) -> None:
        if not self._joy_is_current():
            self._publish_stop()
            return

        self._drive_pub.publish(self._drive_command())
        self._excavation_pub.publish(
            Float64MultiArray(data=self._excavation_command())
        )
        self._deposition_pub.publish(
            Float64MultiArray(data=[self._deposition_command()])
        )

    def _joy_is_current(self) -> bool:
        if self._joy_data is None or self._last_joy_time is None:
            return False
        age = (self.get_clock().now() - self._last_joy_time).nanoseconds / 1e9
        return age <= self.JOY_TIMEOUT_SECONDS

    def _drive_command(self) -> TwistStamped:
        command = TwistStamped()
        command.header.stamp = self.get_clock().now().to_msg()
        linear = self._axis(self.axis_linear, self.invert_linear)
        angular = self._axis(self.axis_angular, self.invert_angular)
        command.twist.linear.x = linear * self.max_linear_speed
        command.twist.angular.z = angular * self.max_angular_speed
        return command

    def _excavation_command(self) -> list[float]:
        bucket = float(
            self._trigger(self.axis_bucket_forward)
            - self._button(self.button_bucket_reverse)
        )
        actuators = self._dpad_vertical()
        return [bucket, actuators, actuators]

    def _deposition_command(self) -> float:
        return self._trigger(self.axis_deposition)

    def _axis(self, index: int, invert: bool = False) -> float:
        if self._joy_data is None or index < 0 or index >= len(self._joy_data.axes):
            return 0.0
        value = float(self._joy_data.axes[index])
        if not math.isfinite(value):
            return 0.0
        value = -value if invert else value
        return 0.0 if abs(value) < self.deadband else max(-1.0, min(value, 1.0))

    def _dpad_vertical(self) -> float:
        if self._joy_data is None:
            return 0.0
        if (
            self.axis_dpad_vertical < 0
            or self.axis_dpad_vertical >= len(self._joy_data.axes)
        ):
            return 0.0
        value = float(self._joy_data.axes[self.axis_dpad_vertical])
        if not math.isfinite(value) or abs(value) < self.dpad_deadband:
            return 0.0
        return max(-1.0, min(value, 1.0))

    def _button(self, index: int) -> int:
        if self._joy_data is None or index < 0 or index >= len(self._joy_data.buttons):
            return 0
        return 1 if self._joy_data.buttons[index] else 0

    def _trigger(self, index: int) -> float:
        if self._joy_data is None or index < 0 or index >= len(self._joy_data.axes):
            return 0.0
        value = float(self._joy_data.axes[index])
        if not math.isfinite(value):
            return 0.0
        # joy_node commonly reports triggers in [-1, 1], while some drivers
        # report [0, 1]. Normalize both forms to an unpressed-to-pressed range.
        if value < 0.0:
            value = (value + 1.0) / 2.0
        return max(0.0, min(value, 1.0))

    def _publish_stop(self) -> None:
        stop = TwistStamped()
        stop.header.stamp = self.get_clock().now().to_msg()
        self._drive_pub.publish(stop)
        self._excavation_pub.publish(Float64MultiArray(data=[0.0, 0.0, 0.0]))
        self._deposition_pub.publish(Float64MultiArray(data=[0.0]))

    def destroy_node(self):
        self._publish_stop()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    driver_console = DriverConsole()
    try:
        rclpy.spin(driver_console)
    finally:
        driver_console.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
