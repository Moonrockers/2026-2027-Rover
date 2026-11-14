#include "spark_max_hardware/protocol.hpp"

// =============================================================================
// SPARK MAX CAN protocol: SKELETON. Nothing here is implemented on purpose.
//
// Fill this in ONLY from sources you can verify, for example:
//   - REV documentation for the SPARK MAX CAN interface, and/or
//   - frames you capture (candump -L) from a controller driven by a known-good
//     setup (e.g. REV Hardware Client / roboRIO) while you change one thing at
//     a time.
//
// Facts to establish and record (with their source) before writing any code:
//   [ ] Arbitration ID layout and whether frames are 11-bit or 29-bit
//   (CAN_EFF_FLAG) [ ] Bitrate the controllers are configured for (set on the
//   interface, not here) [ ] Whether the controller needs an enable /
//   keep-alive frame, its ID, payload, period [ ] Duty-cycle setpoint frame:
//   ID, payload layout, scaling, endianness, valid range [ ] Status/feedback
//   frames: which IDs, payload layout, units, how often they are sent,
//       and whether the periodic status rates must first be configured on the
//       controller
//   [ ] Conversion of native units to SI at the motor shaft (rad, rad/s, A,
//   degC) [ ] What the controller itself does when commands stop arriving (its
//   own timeout)
//
// Verification suggestion: unit-test encode/decode against byte strings you
// captured, so every constant is traceable to a real frame or a document.
// =============================================================================

namespace spark_max_hardware {
namespace {
class SparkMaxCanProtocol : public SparkMaxProtocol {
 public:
  std::string name() const override { return "spark_max"; }

  // Flip to true only once the functions below are implemented AND verified.
  bool implemented() const override { return false; }

  void periodic_frames(std::vector<can_frame>&) override {
    // TODO: emit enable/keep-alive frame(s) if (and only if) the protocol
    // requires them.
  }

  bool encode_duty_cycle(uint8_t /*device_id*/, double /*duty*/,
                         can_frame& /*out*/) override {
    // TODO: build the duty-cycle setpoint frame.
    return false;
  }

  bool decode_feedback(const can_frame& /*frame*/, uint8_t& /*device_id*/,
                       FeedbackUpdate& /*update*/) override {
    // TODO: recognise status frames; fill only the fields present in each
    // frame.
    return false;
  }
};
}  // namespace

std::unique_ptr<SparkMaxProtocol> make_spark_max_protocol() {
  return std::make_unique<SparkMaxCanProtocol>();
}
}  // namespace spark_max_hardware
