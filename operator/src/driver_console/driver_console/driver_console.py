import math

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Joy, Image


class DriverConsole(Node):
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
        self.declare_parameter("axis_bucket", 2)
        self.declare_parameter("axis_actuator_left", 3)
        self.declare_parameter("axis_actuator_right", 4)
        self.declare_parameter("axis_deposition", 5)
        self.declare_parameter("invert_linear", False)
        self.declare_parameter("invert_angular", False)

        self._drive_pub = self.create_publisher(
            TwistStamped,
            self.get_parameter("cmd_vel_topic").get_parameter_value().string_value,
            10,
        )

        self._image_sub1 = self.create_subscription(
            Image, "/camera1/Image", lambda msg: None, 10
        )
        self._image_sub2 = self.create_subscription(
            Image, "/camera2/Image", lambda msg: None, 10
        )
        self._joy_sub = self.create_subscription(Joy, "joy", self.joyCallback, 10)

        self._excavation_pub = self.create_publisher(
            Float64MultiArray, "/excavation/commands", 10
        )
        self._deposition_pub = self.create_publisher(
            Float64MultiArray, "/deposition/commands", 10
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
        self.axisBucket = (
            self.get_parameter("axis_bucket").get_parameter_value().integer_value
        )
        self.axisActuatorLeft = (
            self.get_parameter("axis_actuator_left").get_parameter_value().integer_value
        )
        self.axisActuatorRight = (
            self.get_parameter("axis_actuator_right").get_parameter_value().integer_value
        )
        self.axisDeposition = (
            self.get_parameter("axis_deposition").get_parameter_value().integer_value
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
                self._excavation_pub.publish(Float64MultiArray(data=[0.0, 0.0, 0.0]))
                self._deposition_pub.publish(Float64MultiArray(data=[0.0]))
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
            self._excavation_pub.publish(
                Float64MultiArray(
                    data=[
                        self._axis_value(self.axisBucket),
                        self._axis_value(self.axisActuatorLeft),
                        self._axis_value(self.axisActuatorRight),
                    ]
                )
            )
            self._deposition_pub.publish(
                Float64MultiArray(data=[self._axis_value(self.axisDeposition)])
            )
        except Exception as e:
            self.get_logger().error(f"Sending 0 velocity because got error: {e}")
            twist.twist.linear.x = 0.0
            twist.twist.angular.z = 0.0
            self._excavation_pub.publish(Float64MultiArray(data=[0.0, 0.0, 0.0]))
            self._deposition_pub.publish(Float64MultiArray(data=[0.0]))

        self._drive_pub.publish(twist)

    def _applyDeadband(self, value: float) -> float:
        if abs(value) < self.deadband:
            return 0.0
        return value

    def _axis_value(self, axis: int) -> float:
        if self._joyData is None or axis < 0 or axis >= len(self._joyData.axes):
            return 0.0
        return self._applyDeadband(self._joyData.axes[axis])

    def destroy_node(self):
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.twist.linear.x = 0
        twist.twist.angular.z = 0
        self._drive_pub.publish(twist)
        self._excavation_pub.publish(Float64MultiArray(data=[0.0, 0.0, 0.0]))
        self._deposition_pub.publish(Float64MultiArray(data=[0.0]))
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    driver_console = DriverConsole()
    rclpy.spin(driver_console)
    driver_console.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
