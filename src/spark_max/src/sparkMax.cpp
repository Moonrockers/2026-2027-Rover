#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <chrono>
#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "spark_max/sparkMaxDriver.h"
#include "std_msgs/msg/float64.hpp"

class SparkMaxNode : public rclcpp::Node {
 public:
  SparkMaxNode() : Node("SparkMaxNode") {
    int fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (fd < 0) {
      perror("socket");
    }

    struct ifreq ifr{};
    std::strncpy(ifr.ifr_name, "can0", IFNAMSIZ - 1);  // <-- set the name first
    if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {           // <-- SIOC, not SIOG
      perror("ioctl SIOCGIFINDEX");
    }

    struct sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
      perror("bind failed");
    }

    rclcpp::Parameter canIDParam{"can_id", 1};
    get_parameter("can_id", canIDParam);
    m_driver = std::make_unique<moonrockersrev::SparkMaxCanDriver>(
        fd, canIDParam.as_int());

    subscription_ = create_subscription<std_msgs::msg::Float64>(
        "duty_cycle_setpoint", 10,
        [this](std_msgs::msg::Float64::UniquePtr msg) -> void {
          dutyCycleSetpoint = msg->data;
        });

    dutyCycleTimer_ =
        create_wall_timer(std::chrono::milliseconds(5),
                          std::bind(&SparkMaxNode::setDutyCycle, this));
  };

 private:
  std::unique_ptr<moonrockersrev::SparkMaxCanDriver> m_driver;
  double dutyCycleSetpoint = 0.0;

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr dutyCycleTimer_;

  void setDutyCycle() { m_driver->SetPower(dutyCycleSetpoint); }
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SparkMaxNode>());
  rclcpp::shutdown();
  return 0;
}