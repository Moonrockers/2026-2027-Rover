#include "spark_max_hardware/spark_max_system.hpp"

#include <algorithm>
#include <cmath>
#include <set>

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/logging.hpp"

namespace spark_max_hardware {
namespace {
const rclcpp::Logger kLog = rclcpp::get_logger("SparkMaxSystem");
rclcpp::Clock g_clock(RCL_STEADY_TIME);
constexpr size_t kMaxFramesPerCycle = 256;
constexpr int kMaxConsecutiveTxFailures = 3;

template <typename Map>
std::string get(const Map& m, const std::string& key, const std::string& def) {
  auto it = m.find(key);
  return it == m.end() ? def : it->second;
}

bool to_double(const std::string& s, double& out) {
  try {
    size_t pos = 0;
    out = std::stod(s, &pos);
    return pos == s.size() && std::isfinite(out);
  } catch (...) {
    return false;
  }
}
}  // namespace

SparkMaxSystem::Motor* SparkMaxSystem::find_motor(uint8_t id) {
  for (auto& m : motors_) {
    if (m.device_id == id) {
      return &m;
    }
  }
  return nullptr;
}

SparkMaxSystem::CallbackReturn SparkMaxSystem::on_init(
    const hardware_interface::HardwareInfo& info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
      CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  can_interface_ = get(info.hardware_parameters, "can_interface", "can0");

  const std::string protocol_name =
      get(info.hardware_parameters, "protocol", "spark_max");
  simulation_ = protocol_name == "simulation";
  if (!simulation_) {
    protocol_ = make_protocol(protocol_name);
  }
  if (!simulation_ && !protocol_) {
    RCLCPP_ERROR(kLog, "Unknown protocol '%s'", protocol_name.c_str());
    return CallbackReturn::ERROR;
  }

  double timeout_ms = 0.0;
  if (!to_double(get(info.hardware_parameters, "feedback_timeout_ms", "200"),
                 timeout_ms) ||
      timeout_ms < 0.0) {
    RCLCPP_ERROR(
        kLog,
        "feedback_timeout_ms must be a number >= 0 (0 disables the check)");
    return CallbackReturn::ERROR;
  }
  feedback_timeout_ = std::chrono::duration_cast<Clock::duration>(
      std::chrono::duration<double, std::milli>(timeout_ms));

  motors_.clear();
  std::set<int> seen_ids;
  for (const auto& j : info.joints) {
    Motor m;
    m.joint = j.name;

    double id = 0.0;
    auto id_it = j.parameters.find("device_id");
    if (id_it == j.parameters.end() || !to_double(id_it->second, id) ||
        id < 0 || id > 255 || id != std::floor(id)) {
      RCLCPP_ERROR(kLog, "Joint '%s': 'device_id' (integer 0-255) is required",
                   j.name.c_str());
      return CallbackReturn::ERROR;
    }
    m.device_id = static_cast<uint8_t>(id);
    if (!seen_ids.insert(m.device_id).second) {
      RCLCPP_ERROR(kLog, "Joint '%s': duplicate device_id %d", j.name.c_str(),
                   m.device_id);
      return CallbackReturn::ERROR;
    }
    if (!to_double(get(j.parameters, "max_velocity", "1.0"), m.max_velocity) ||
        m.max_velocity <= 0.0 ||
        !to_double(get(j.parameters, "gear_ratio", "1.0"), m.gear_ratio) ||
        m.gear_ratio <= 0.0) {
      RCLCPP_ERROR(
          kLog, "Joint '%s': max_velocity and gear_ratio must be numbers > 0",
          j.name.c_str());
      return CallbackReturn::ERROR;
    }
    m.sign = (get(j.parameters, "invert", "false") == "true") ? -1.0 : 1.0;

    for (const auto& ci : j.command_interfaces) {
      if (ci.name == hardware_interface::HW_IF_VELOCITY) {
        m.has_velocity_cmd = true;
      } else if (ci.name == "duty_cycle") {
        m.has_duty_cmd = true;
      } else {
        RCLCPP_ERROR(kLog, "Joint '%s': unsupported command interface '%s'",
                     j.name.c_str(), ci.name.c_str());
        return CallbackReturn::ERROR;
      }
    }
    if (m.has_velocity_cmd == m.has_duty_cmd) {
      RCLCPP_ERROR(kLog,
                   "Joint '%s' needs exactly one command interface: velocity "
                   "or duty_cycle",
                   j.name.c_str());
      return CallbackReturn::ERROR;
    }

    for (const auto& si : j.state_interfaces) {
      if (si.name == hardware_interface::HW_IF_POSITION) {
        m.st_position = true;
      } else if (si.name == hardware_interface::HW_IF_VELOCITY) {
        m.st_velocity = true;
      } else if (si.name == "current") {
        m.st_current = true;
      } else if (si.name == "temperature") {
        m.st_temperature = true;
      } else {
        RCLCPP_ERROR(kLog, "Joint '%s': unsupported state interface '%s'",
                     j.name.c_str(), si.name.c_str());
        return CallbackReturn::ERROR;
      }
    }
    motors_.push_back(std::move(m));
  }
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
SparkMaxSystem::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> out;
  for (auto& m : motors_) {
    if (m.st_position) {
      out.emplace_back(m.joint, hardware_interface::HW_IF_POSITION, &m.pos);
    }
    if (m.st_velocity) {
      out.emplace_back(m.joint, hardware_interface::HW_IF_VELOCITY, &m.vel);
    }
    if (m.st_current) {
      out.emplace_back(m.joint, "current", &m.current);
    }
    if (m.st_temperature) {
      out.emplace_back(m.joint, "temperature", &m.temperature);
    }
  }
  return out;
}

std::vector<hardware_interface::CommandInterface>
SparkMaxSystem::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> out;
  for (auto& m : motors_) {
    if (m.has_velocity_cmd) {
      out.emplace_back(m.joint, hardware_interface::HW_IF_VELOCITY,
                       &m.cmd_velocity);
    } else {
      out.emplace_back(m.joint, "duty_cycle", &m.cmd_duty);
    }
  }
  return out;
}

SparkMaxSystem::CallbackReturn SparkMaxSystem::on_activate(
    const rclcpp_lifecycle::State&) {
  if (simulation_) {
    const auto now = Clock::now();
    for (auto& m : motors_) {
      m.cmd_velocity = m.cmd_duty = 0.0;
      m.last_feedback = now;
      m.pos = m.vel = m.current = m.temperature = 0.0;
    }
    tx_failures_ = 0;
    RCLCPP_INFO(kLog, "Active in simulation mode, %zu motor(s)", motors_.size());
    return CallbackReturn::SUCCESS;
  }
  if (!protocol_->implemented()) {
    RCLCPP_ERROR(kLog,
                 "Protocol '%s' is not implemented yet (see "
                 "src/spark_max_can_protocol.cpp). "
                 "Refusing to activate.",
                 protocol_->name().c_str());
    return CallbackReturn::ERROR;
  }
  if (!can_.open(can_interface_)) {
    RCLCPP_ERROR(kLog, "Cannot open CAN interface '%s': %s",
                 can_interface_.c_str(), can_.last_error().c_str());
    return CallbackReturn::ERROR;
  }
  const auto now = Clock::now();
  for (auto& m : motors_) {
    m.cmd_velocity = m.cmd_duty = 0.0;
    m.last_feedback = now;  // grace period starts now
  }
  tx_failures_ = 0;
  rx_.reserve(kMaxFramesPerCycle);
  RCLCPP_INFO(kLog, "Active on %s using protocol '%s', %zu motor(s)",
              can_interface_.c_str(), protocol_->name().c_str(),
              motors_.size());
  return CallbackReturn::SUCCESS;
}

bool SparkMaxSystem::send_neutral() {
  bool ok = true;
  for (auto& m : motors_) {
    can_frame f{};
    if (!protocol_->encode_duty_cycle(m.device_id, 0.0, f) || !can_.send(f)) {
      ok = false;
    }
  }
  return ok;
}

SparkMaxSystem::CallbackReturn SparkMaxSystem::on_deactivate(
    const rclcpp_lifecycle::State&) {
  if (can_.is_open()) {
    if (!send_neutral()) {
      RCLCPP_WARN(kLog,
                  "Could not confirm neutral command was sent on deactivate");
    }
    can_.close();
  }
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type SparkMaxSystem::read(const rclcpp::Time&,
                                                     const rclcpp::Duration& period) {
  if (simulation_) {
    const double period_seconds = period.seconds();
    const double dt =
        std::isfinite(period_seconds) ? std::clamp(period_seconds, 0.0, 0.1)
                                      : 0.0;
    for (auto& m : motors_) {
      const double duty = m.has_velocity_cmd
                              ? m.cmd_velocity / m.max_velocity
                              : m.cmd_duty;
      const double bounded_duty =
          std::isfinite(duty) ? std::clamp(duty, -1.0, 1.0) : 0.0;
      // Simulation commands are already expressed in joint space. The sign
      // correction is only needed when translating to a physical motor.
      const double target_velocity = m.max_velocity * bounded_duty;
      m.vel = target_velocity;
      m.pos += m.vel * dt;
      m.current = std::abs(bounded_duty) * 20.0;
      m.temperature = 25.0 + m.current * 0.25;
    }
    return hardware_interface::return_type::OK;
  }
  rx_.clear();
  if (can_.receive(rx_, kMaxFramesPerCycle) < 0) {
    RCLCPP_ERROR(kLog, "CAN receive failed: %s", can_.last_error().c_str());
    return hardware_interface::return_type::ERROR;
  }

  const auto now = Clock::now();
  for (const auto& frame : rx_) {
    uint8_t id = 0;
    FeedbackUpdate u;
    if (!protocol_->decode_feedback(frame, id, u)) {
      continue;
    }
    Motor* m = find_motor(id);
    if (!m) {
      continue;
    }
    if (u.position_rad) {
      m->fb_pos = *u.position_rad;
    }
    if (u.velocity_rad_s) {
      m->fb_vel = *u.velocity_rad_s;
    }
    if (u.current_a) {
      m->fb_current = *u.current_a;
    }
    if (u.temperature_c) {
      m->fb_temperature = *u.temperature_c;
    }
    m->last_feedback = now;
  }

  for (auto& m : motors_) {
    if (feedback_timeout_.count() > 0 &&
        now - m.last_feedback > feedback_timeout_) {
      RCLCPP_ERROR(
          kLog, "Joint '%s' (device %d): no feedback within %.0f ms",
          m.joint.c_str(), m.device_id,
          std::chrono::duration<double, std::milli>(feedback_timeout_).count());
      return hardware_interface::return_type::ERROR;
    }
    m.pos = m.sign * m.fb_pos / m.gear_ratio;
    m.vel = m.sign * m.fb_vel / m.gear_ratio;
    m.current = m.fb_current;
    m.temperature = m.fb_temperature;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type SparkMaxSystem::write(const rclcpp::Time&,
                                                      const rclcpp::Duration&) {
  if (simulation_) {
    return hardware_interface::return_type::OK;
  }
  tx_.clear();
  protocol_->periodic_frames(tx_);

  for (auto& m : motors_) {
    double duty =
        m.has_velocity_cmd ? m.cmd_velocity / m.max_velocity : m.cmd_duty;
    if (!std::isfinite(duty)) {
      duty = 0.0;
    }  // NaN/inf -> stop
    duty = std::clamp(duty, -1.0, 1.0);
    can_frame f{};
    if (!protocol_->encode_duty_cycle(m.device_id, m.sign * duty, f)) {
      RCLCPP_ERROR(kLog, "Joint '%s': failed to encode duty cycle",
                   m.joint.c_str());
      return hardware_interface::return_type::ERROR;
    }
    tx_.push_back(f);
  }

  bool all_sent = true;
  for (const auto& f : tx_) {
    if (!can_.send(f)) {
      all_sent = false;
      RCLCPP_ERROR_THROTTLE(kLog, g_clock, 1000, "CAN send failed: %s",
                            can_.last_error().c_str());
    }
  }
  tx_failures_ = all_sent ? 0 : tx_failures_ + 1;
  return tx_failures_ >= kMaxConsecutiveTxFailures
             ? hardware_interface::return_type::ERROR
             : hardware_interface::return_type::OK;
}
}  // namespace spark_max_hardware

PLUGINLIB_EXPORT_CLASS(spark_max_hardware::SparkMaxSystem,
                       hardware_interface::SystemInterface)
