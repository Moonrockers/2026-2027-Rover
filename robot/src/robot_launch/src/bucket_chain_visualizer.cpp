#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace
{
constexpr double kLadderLength = 20.0 * 0.0254;
constexpr double kChainHalfHeight = 0.12;
constexpr double kPerimeter = 2.0 * (kLadderLength + 2.0 * kChainHalfHeight);
constexpr std::size_t kBucketCount = 4;
}

class BucketChainVisualizer : public rclcpp::Node
{
public:
  BucketChainVisualizer()
  : Node("bucket_chain_visualizer")
  {
    publisher_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      [this](const sensor_msgs::msg::JointState::SharedPtr message) {
        update(message);
      });
  }

private:
  static std::pair<double, double> path_position(double distance)
  {
    distance = std::fmod(distance, kPerimeter);
    if (distance < 0.0) {
      distance += kPerimeter;
    }

    if (distance < kLadderLength) {
      return {distance, kChainHalfHeight};
    }
    distance -= kLadderLength;
    if (distance < 2.0 * kChainHalfHeight) {
      return {kLadderLength, kChainHalfHeight - distance};
    }
    distance -= 2.0 * kChainHalfHeight;
    if (distance < kLadderLength) {
      return {kLadderLength - distance, -kChainHalfHeight};
    }
    distance -= kLadderLength;
    return {0.0, -kChainHalfHeight + distance};
  }

  void update(const sensor_msgs::msg::JointState::SharedPtr & message)
  {
    auto bucket_joint = std::find(message->name.begin(), message->name.end(), "bucket_joint");
    if (bucket_joint == message->name.end()) {
      return;
    }
    const auto index = static_cast<std::size_t>(
      std::distance(message->name.begin(), bucket_joint));
    if (index >= message->position.size()) {
      return;
    }

    sensor_msgs::msg::JointState visual_state;
    visual_state.header.stamp = now();
    visual_state.name.reserve(kBucketCount * 2);
    visual_state.position.reserve(kBucketCount * 2);

    const double phase = message->position[index];
    for (std::size_t bucket = 0; bucket < kBucketCount; ++bucket) {
      const auto position = path_position(
        phase + static_cast<double>(bucket) * kPerimeter / kBucketCount);
      visual_state.name.push_back(
        "bucket_chain_" + std::to_string(bucket) + "_x_joint");
      visual_state.position.push_back(position.first);
      visual_state.name.push_back(
        "bucket_chain_" + std::to_string(bucket) + "_z_joint");
      visual_state.position.push_back(position.second);
    }
    publisher_->publish(visual_state);
  }

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BucketChainVisualizer>());
  rclcpp::shutdown();
  return 0;
}
