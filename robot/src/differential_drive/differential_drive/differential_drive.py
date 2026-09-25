import math

import rclpy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from rclpy.time import ClockType, Duration


class DifferentialDriveNode(rclpy.Node):
    def __init__(self) -> None:
        super().__init__("DifferentialDriveNode")

        self.declare_parameter("max_linear_velocity_mps", 2.0)
        self.declare_parameter("max_angular_velocity_radps", math.pi)
        self.declare_parameter("linear_vel_axis", 0)
        self.declare_parameter("angular_vel_axis", 1)

        self._max_linear_vel_mps = (
            self.get_parameter("max_linear_velocity_mps")
            .get_parameter_value()
            .double_value
        )
        self._max_angular_vel_radps = (
            self.get_parameter("max_angular_velocity_radps")
            .get_parameter_value()
            .double_value
        )

        self._linear_vel_axis = (
            self.get_parameter("linear_vel_axis").get_parameter_value().integer_value
        )

        self._angular_vel_axis = (
            self.get_parameter("angular_vel_axis").get_parameter_value().integer_value
        )

        self._joyVal: Joy | None = None
        self._joySub = self.create_subscription(Joy, "/joy", self.joyCallback, 10)
        self._velPub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._velTimer = self.create_timer(0.01, self.pubVel)

    def joyCallback(self, msg: Joy) -> None:
        self._joyVal = msg

    def pubVel(self) -> None:
        msg = Twist
        if not self._joyVal or self.get_clock().now() - \
            self._joyVal.header.stamp > Duration(seconds=0.1):  # type: ignore
            msg.angular = 0
            msg.linear = 0
        else:
            msg.angular = self._max_angular_vel_radps * self._joyVal.axes[0]


def main():
    print("Hi from differential_drive.")


if __name__ == "__main__":
    main()
