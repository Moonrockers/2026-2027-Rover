#pragma once
#include <chrono>
#include <cstdint>
#include <memory>
#include <limits>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "spark_max_hardware/can_socket.hpp"
#include "spark_max_hardware/protocol.hpp"

namespace spark_max_hardware
{
class SparkMaxSystem : public hardware_interface::SystemInterface
{
public:
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  using Clock = std::chrono::steady_clock;

  struct Motor
  {
    std::string joint;
    uint8_t device_id = 0;
    double max_velocity = 1.0;  // joint rad/s at 100% duty (open-loop scaling of velocity cmds)
    double gear_ratio = 1.0;    // motor revolutions per joint revolution
    double sign = 1.0;          // -1 if inverted
    double min_position = -std::numeric_limits<double>::infinity();
    double max_position = std::numeric_limits<double>::infinity();

    bool has_velocity_cmd = false;
    bool has_duty_cmd = false;
    bool st_position = false, st_velocity = false, st_current = false, st_temperature = false;

    // Exported (joint space)
    double cmd_velocity = 0.0, cmd_duty = 0.0;
    double pos = 0.0, vel = 0.0, current = 0.0, temperature = 0.0;

    // Latest raw feedback (motor shaft, SI)
    double fb_pos = 0.0, fb_vel = 0.0, fb_current = 0.0, fb_temperature = 0.0;
    Clock::time_point last_feedback{};
  };

  Motor * find_motor(uint8_t device_id);
  bool send_neutral();

  std::vector<Motor> motors_;
  std::unique_ptr<SparkMaxProtocol> protocol_;
  CanSocket can_;
  std::string can_interface_;
  bool simulation_ = false;
  Clock::duration feedback_timeout_{};
  int tx_failures_ = 0;
  std::vector<can_frame> rx_, tx_;
};
}  // namespace spark_max_hardware
