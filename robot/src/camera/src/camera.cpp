#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

class CameraNode : public rclcpp::Node {
 public:
  CameraNode() : Node("CameraNode") {
    m_imagePub = create_publisher<sensor_msgs::msg::Image>("Image", 10);
  }

 private:
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_imagePub;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraNode>());
  rclcpp::shutdown();
  return 0;
}
