#include "spark_max_hardware/protocol.hpp"

#include <algorithm>
#include <cstring>

namespace spark_max_hardware
{
namespace
{
// Test double so the ros2_control plumbing (lifecycle, feedback, staleness, shutdown)
// can be exercised on a virtual CAN bus. The frame layout below is INVENTED for testing
// and has nothing to do with the SPARK MAX protocol.
//   command : std id 0x100+device, dlc 4, float32 LE duty
//   feedback: std id 0x200+device, dlc 8, float32 LE position_rad, float32 LE velocity_rad_s
class TestLoopbackProtocol : public SparkMaxProtocol
{
public:
  std::string name() const override {return "test_loopback";}
  bool implemented() const override {return true;}
  void periodic_frames(std::vector<can_frame> &) override {}

  bool encode_duty_cycle(uint8_t id, double duty, can_frame & out) override
  {
    out = can_frame{};
    out.can_id = 0x100 + id;
    out.can_dlc = 4;
    const float d = static_cast<float>(std::clamp(duty, -1.0, 1.0));
    std::memcpy(out.data, &d, 4);
    return true;
  }

  bool decode_feedback(const can_frame & f, uint8_t & id, FeedbackUpdate & u) override
  {
    if (f.can_id & CAN_EFF_FLAG) {return false;}
    const canid_t cid = f.can_id & CAN_SFF_MASK;
    if (cid < 0x200 || cid > 0x2FF || f.can_dlc < 8) {return false;}
    float pos, vel;
    std::memcpy(&pos, f.data, 4);
    std::memcpy(&vel, f.data + 4, 4);
    id = static_cast<uint8_t>(cid - 0x200);
    u.position_rad = pos;
    u.velocity_rad_s = vel;
    return true;
  }
};
}  // namespace

std::unique_ptr<SparkMaxProtocol> make_protocol(const std::string & name)
{
  if (name == "spark_max") {return make_spark_max_protocol();}
  if (name == "test_loopback") {return std::make_unique<TestLoopbackProtocol>();}
  return nullptr;
}
}  // namespace spark_max_hardware
