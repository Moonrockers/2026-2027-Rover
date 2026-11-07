#include "spark_max_hardware/can_socket.hpp"

#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>

namespace spark_max_hardware
{
bool CanSocket::open(const std::string & ifname)
{
  close();
  fd_ = ::socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW);
  if (fd_ < 0) {
    last_error_ = std::string("socket(): ") + std::strerror(errno);
    return false;
  }
  ifreq ifr{};
  std::strncpy(ifr.ifr_name, ifname.c_str(), IFNAMSIZ - 1);
  if (::ioctl(fd_, SIOCGIFINDEX, &ifr) < 0) {
    last_error_ = "interface '" + ifname + "': " + std::strerror(errno);
    close();
    return false;
  }
  sockaddr_can addr{};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (::bind(fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
    last_error_ = std::string("bind(): ") + std::strerror(errno);
    close();
    return false;
  }
  return true;
}

void CanSocket::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool CanSocket::send(const can_frame & frame)
{
  if (fd_ < 0) {last_error_ = "socket not open"; return false;}
  const ssize_t n = ::write(fd_, &frame, sizeof(frame));
  if (n == static_cast<ssize_t>(sizeof(frame))) {return true;}
  last_error_ = std::string("write(): ") + (n < 0 ? std::strerror(errno) : "short write");
  return false;
}

int CanSocket::receive(std::vector<can_frame> & out, size_t max_frames)
{
  if (fd_ < 0) {last_error_ = "socket not open"; return -1;}
  int count = 0;
  while (static_cast<size_t>(count) < max_frames) {
    can_frame f{};
    const ssize_t n = ::read(fd_, &f, sizeof(f));
    if (n == static_cast<ssize_t>(sizeof(f))) {
      out.push_back(f);
      ++count;
    } else if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      break;
    } else if (n < 0 && errno == EINTR) {
      continue;
    } else {
      last_error_ = std::string("read(): ") + (n < 0 ? std::strerror(errno) : "short read");
      return -1;
    }
  }
  return count;
}
}  // namespace spark_max_hardware
