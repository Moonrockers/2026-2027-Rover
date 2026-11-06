#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

class DiffDriveKinematics : public rclcpp::Node {
 public:
  DiffDriveKinematics() : Node("DiffDriveKinematics") {
    m_velSub = create_subscription<geometry_msgs::msg::TwistStamped>(
        "cmd_vel", 10,
        [this](geometry_msgs::msg::TwistStamped::UniquePtr msg) -> void {});

    m_dutyCyclePub1 =
        create_publisher<std_msgs::msg::Float64>("duty_cycle_setpoint1", 10);

    m_dutyCyclePub2 =
        create_publisher<std_msgs::msg::Float64>("duty_cycle_setpoint2", 10);

    m_dutyCyclePub3 =
        create_publisher<std_msgs::msg::Float64>("duty_cycle_setpoint3", 10);

    m_dutyCyclePub4 =
        create_publisher<std_msgs::msg::Float64>("duty_cycle_setpoint4", 10);
  }

 private:
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr m_velSub;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr m_dutyCyclePub1;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr m_dutyCyclePub2;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr m_dutyCyclePub3;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr m_dutyCyclePub4;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DiffDriveKinematics>());
  rclcpp::shutdown();
  return 0;
}