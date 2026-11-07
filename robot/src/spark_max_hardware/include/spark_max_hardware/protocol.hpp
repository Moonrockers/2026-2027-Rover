#pragma once
#include <linux/can.h>

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace spark_max_hardware
{
// Latest values decoded from a frame, in SI units at the MOTOR SHAFT.
// Only fields actually present in the frame are set; the rest stay nullopt.
// Gear ratio and inversion are applied by SparkMaxSystem, not by the protocol.
struct FeedbackUpdate
{
  std::optional<double> position_rad;
  std::optional<double> velocity_rad_s;
  std::optional<double> current_a;
  std::optional<double> temperature_c;
};

// Everything that depends on the controller's on-the-wire format lives behind this.
class SparkMaxProtocol
{
public:
  virtual ~SparkMaxProtocol() = default;

  virtual std::string name() const = 0;

  // False => on_activate refuses to start. Lets an unfinished protocol fail loudly
  // instead of silently doing nothing.
  virtual bool implemented() const = 0;

  // Frames that must go out every control cycle independent of motor commands
  // (e.g. a keep-alive/enable frame, IF the real protocol needs one). May be empty.
  virtual void periodic_frames(std::vector<can_frame> & out) = 0;

  // duty in [-1, 1], already inverted for motor direction. Returns false on failure.
  virtual bool encode_duty_cycle(uint8_t device_id, double duty, can_frame & out) = 0;

  // If `frame` is feedback from a controller, set device_id/update and return true.
  // Return false for any frame that is not recognised feedback (it is then ignored).
  virtual bool decode_feedback(
    const can_frame & frame, uint8_t & device_id, FeedbackUpdate & update) = 0;
};

// "spark_max"      -> real protocol (skeleton, see spark_max_can_protocol.cpp)
// "test_loopback"  -> made-up protocol for bench-testing the framework on vcan. NOT SPARK MAX.
std::unique_ptr<SparkMaxProtocol> make_protocol(const std::string & name);
std::unique_ptr<SparkMaxProtocol> make_spark_max_protocol();
}  // namespace spark_max_hardware
