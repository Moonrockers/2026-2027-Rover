#pragma once
#include <linux/can.h>

#include <string>
#include <vector>

namespace spark_max_hardware
{
// Thin non-blocking wrapper over a raw SocketCAN socket (classic CAN, not CAN FD).
// Knows nothing about SPARK MAX; that lives behind SparkMaxProtocol.
class CanSocket
{
public:
  CanSocket() = default;
  ~CanSocket() {close();}
  CanSocket(const CanSocket &) = delete;
  CanSocket & operator=(const CanSocket &) = delete;

  bool open(const std::string & ifname);
  void close();
  bool is_open() const {return fd_ >= 0;}

  // Non-blocking. False if the frame was not queued (e.g. TX queue full, interface down).
  bool send(const can_frame & frame);

  // Non-blocking. Appends up to max_frames pending frames to `out`.
  // Returns the number appended, or -1 on a socket error.
  int receive(std::vector<can_frame> & out, size_t max_frames);

  const std::string & last_error() const {return last_error_;}

private:
  int fd_ = -1;
  std::string last_error_;
};
}  // namespace spark_max_hardware
