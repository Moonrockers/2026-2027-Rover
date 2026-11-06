import math

import time

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Parameter

from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Joy, Image


class AutonomyController(Node):
    DT = 0.02

    def __init__(self) -> None:
        super().__init__("operator_interface")
        self.declare_parameter("joy_topic", "joy")
        self.declare_parameter("cmd_vel_topic", "/drive/cmd_vel")
        self.declare_parameter("max_linear_speed_m_s", 0.5)
        self.declare_parameter("max_angular_speed_rad_s", 0.5)
        self.declare_parameter("deadband", 0.05)
        self.declare_parameter("axis_linear", 1)
        self.declare_parameter("axis_angular", 0)
        self.declare_parameter("invert_linear", False)
        self.declare_parameter("invert_angular", False)

        self._drive_pub = self.create_publisher(
            TwistStamped,
            self.get_parameter("cmd_vel_topic").get_parameter_value().string_value,
            10,
        )

        # these would do some form of output but are not for diagrams
        self._image_sub1 = self.create_subscription(
            Image, "/camera1/Image", lambda msg: None, 10
        )
        self._image_sub1 = self.create_subscription(
            Image, "/camera2/Image", lambda msg: None, 10
        )

        self._excavation_buckets_pub = self.create_publisher(
            Float64, "/excavation/duty_cycle_setpoint5", 10
        )
        self._excavation_actuator_pub = self.create_publisher(
            Float64, "/excavation/duty_cycle_setpoint6", 10
        )
        self._excavation_actuator2_pub = self.create_publisher(
            Float64, "/excavation/duty_cycle_setpoint7", 10
        )

        self._excavation_actuator2_pub = self.create_publisher(
            Float64, "/deposition/duty_cycle_setpoint8", 10
        )

        self._timer = self.create_timer(self.DT, self.writeVelocity)

        self._joyData: Joy | None = None

        self.maxLinearSpeed = (
            self.get_parameter("max_linear_speed_m_s")
            .get_parameter_value()
            .double_value
        )
        self.maxAngularSpeed = (
            self.get_parameter("max_angular_speed_rad_s")
            .get_parameter_value()
            .double_value
        )
        self.deadband = (
            self.get_parameter("deadband").get_parameter_value().double_value
        )
        self.axisLinear = (
            self.get_parameter("axis_linear").get_parameter_value().integer_value
        )
        self.axisAngular = (
            self.get_parameter("axis_angular").get_parameter_value().integer_value
        )
        self.invertLinear = (
            self.get_parameter("invert_linear").get_parameter_value().bool_value
        )
        self.invertAngular = (
            self.get_parameter("invert_angular").get_parameter_value().bool_value
        )

        self.get_logger().info("Operator Interface node has been started.")
        self.get_logger().info(
            f"Subscribed to joystick topic: {self.get_parameter('joy_topic').get_parameter_value().string_value}"
        )
        self.get_logger().info(
            f"Publishing velocity commands to: {self.get_parameter('cmd_vel_topic').get_parameter_value().string_value}"
        )
        self.get_logger().info(f"Max axis_linear speed: {self.maxLinearSpeed} m/s")
        self.get_logger().info(f"Max axis_angular speed: {self.maxAngularSpeed} rad/s")
        self.get_logger().info(f"Deadband: {self.deadband}")
        self.get_logger().info(f"Axis for linear control: {self.axisLinear}")
        self.get_logger().info(f"Axis for angular control: {self.axisAngular}")
        self.get_logger().info(f"Invert linear axis: {self.invertLinear}")
        self.get_logger().info(f"Invert angular axis: {self.invertAngular}")

    def joyCallback(self, msg: Joy) -> None:
        self._joyData = msg

    def writeVelocity(self) -> None:
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        try:
            if self._joyData is None:
                twist.twist.linear.x = 0.0
                twist.twist.angular.z = 0.0
                self._drive_pub.publish(twist)
                return
            twist.twist.linear.x = self._applyDeadband(
                self._joyData.axes[self.axisLinear]
                * self.maxLinearSpeed
                * (-1 if self.invertLinear else 1)
            )

            twist.twist.angular.z = self._applyDeadband(
                self._joyData.axes[self.axisAngular]
                * self.maxAngularSpeed
                * (-1 if self.invertAngular else 1)
            )
        except Exception as e:
            self.get_logger().error(f"Sending 0 velocity because got error: {e}")
            twist.twist.linear.x = 0.0
            twist.twist.angular.z = 0.0

        self._drive_pub.publish(twist)

    def _applyDeadband(self, value: float) -> float:
        if abs(value) < self.deadband:
            return 0.0
        return value

    def destroy_node(self):
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.twist.linear.x = 0
        twist.twist.angular.z = 0
        self._drive_pub.publish(twist)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    autonomy_controller = AutonomyController()
    rclpy.spin(autonomy_controller)
    autonomy_controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
