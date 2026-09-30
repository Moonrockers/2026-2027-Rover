#include "spark_max/waveshareCANDriver.hpp"

#include <errno.h>
#include <fcntl.h>
#include <linux/can.h>
#include <termios.h>
#include <unistd.h>

#include <cstring>
#include <iostream>
#include <optional>

namespace SerialUtils {
/*
opens a serial port form a given file descriptor. Returns the filepath. You are
responsible for checking the fd >= 0
*/
int openSerialPort(const char* portname) {
  // responsibility is on the user of this function to check for fd >= 0
  int fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
  return fd;
}

/*
Configures the serial port on the file descriptor to be at the baud rate.
Does other config
*/
bool configureSerialPort(int fd, int baud) {
  struct termios tty;
  if (tcgetattr(fd, &tty) != 0) {
    return false;
  }

  cfsetospeed(&tty, baud);
  // TODO: verify these configs
  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;  // 8-bit characters
  tty.c_iflag &= ~IGNBRK;                      // disable break processing
  tty.c_lflag = 0;      // no signaling chars, no echo, no
                        // canonical processing
  tty.c_oflag = 0;      // no remapping, no delays
  tty.c_cc[VMIN] = 0;   // read doesn't block
  tty.c_cc[VTIME] = 5;  // 0.5 seconds read timeout

  tty.c_iflag &= ~(IXON | IXOFF | IXANY);  // shut off xon/xoff ctrl

  tty.c_cflag |= (CLOCAL | CREAD);    // ignore modem controls,
                                      // enable reading
  tty.c_cflag &= ~(PARENB | PARODD);  // shut off parity
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;

  if (tcsetattr(fd, TCSANOW, &tty) != 0) {
    return false;
  }
  return true;
}

/*
Reads data from the port. Returns a status from read and modified buf in place
*/
int readFromPort(int fd, char* buf, size_t size) { return read(fd, buf, size); }

bool readByte(int fd, u_int8_t& b) {
  return SerialUtils::readFromPort(fd, reinterpret_cast<char*>(&b), 1) == 1;
}

int writeToPort(int fd, const char* buf, size_t size) {
  return write(fd, buf, size);
}

void closePort(int fd) { close(fd); }

}  // namespace SerialUtils

namespace WaveshareCAN {
const size_t FRAME_SIZE_MAX = 32;  // 13 ? 32 should be overkill

std::optional<int> openCanPort(const char* portName) {
  int fd = SerialUtils::openSerialPort(portName);
  if (fd < 0) {
    return {};
  }
  return fd;
}

std::optional<can_frame> readFrame(int fd) {
  u_int8_t b;

  // Sync to start byte
  do {
    if (!SerialUtils::readByte(fd, b)) return std::nullopt;
  } while (b != 0xAA);

  u_int8_t type;
  if (!SerialUtils::readByte(fd, type)) return std::nullopt;

  const bool isExtended = type & 0x20;
  const bool isRemote = type & 0x10;
  const u_int8_t len = type & 0x0F;
  if (len > 8) return std::nullopt;

  const size_t idLen = isExtended ? 4 : 2;
  const size_t n = idLen + len + 1;  // id + data + end code
  u_int8_t buf[4 + 8 + 1];
  for (size_t i = 0; i < n; ++i) {
    if (!SerialUtils::readByte(fd, buf[i])) return std::nullopt;
  }
  if (buf[n - 1] != 0x55) return std::nullopt;  // bad end code

  u_int32_t id = 0;
  for (size_t i = 0; i < idLen; ++i) id |= u_int32_t(buf[i]) << (8 * i);

  can_frame out{};
  out.can_id =
      isExtended ? ((id & CAN_EFF_MASK) | CAN_EFF_FLAG) : (id & CAN_SFF_MASK);
  if (isRemote) out.can_id |= CAN_RTR_FLAG;
  out.can_dlc = len;
  std::memcpy(out.data, buf + idLen, len);
  return out;
}

bool writeFrame(int fd, const can_frame& frame) {
  const u_int8_t len = frame.len;
  if (len > 8) {
    return false;
  }

  const bool isExtended = frame.can_id & CAN_EFF_FLAG;
  const bool isRemote = frame.can_id & CAN_RTR_FLAG;
  const size_t idLen = isExtended ? 4 : 2;

  const u_int32_t id = isExtended ? (frame.can_id & CAN_EFF_MASK)
                                  : (frame.can_id & CAN_SFF_MASK);

  // header + type + id + data + end code <= 1 + 1 + 4 + 8 + 1
  u_int8_t buf[15];
  size_t n = 0;

  buf[n++] = 0xAA;
  buf[n++] = 0xC0 | (isExtended ? 0x20 : 0) | (isRemote ? 0x10 : 0) | len;

  for (size_t i = 0; i < idLen; i++) {
    buf[n++] = (id >> (8 * i)) & 0xFF;
  }

  std::memcpy(buf + n, frame.data, len);
  n += len;

  buf[n++] = 0x55;

  // Send as one buffer so the frame isn't interleaved with anything else,
  // and loop to handle partial writes.
  size_t sent = 0;
  while (sent < n) {
    const ssize_t w = SerialUtils::writeToPort(
        fd, reinterpret_cast<const char*>(buf + sent), n - sent);
    if (w <= 0) {
      return false;
    }
    sent += static_cast<size_t>(w);
  }
  return true;
}
}  // namespace WaveshareCAN