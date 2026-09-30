#pragma once

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
int openSerialPort(const char* portname);

/*
Configures the serial port on the file descriptor to be at the baud rate.
Does other config
*/
bool configureSerialPort(int fd, int baud);

/*
Reads data from the port. Returns a status from read and modified buf in place
*/
int readFromPort(int fd, char* buf, size_t size);

bool readByte(int fd, uint8_t& b);

int writeToPort(int fd, const char* buf, size_t size);

void closePort(int fd);

}  // namespace SerialUtils

namespace WaveshareCAN {
const size_t FRAME_SIZE_MAX = 32;  // 13 ? 32 should be overkill

std::optional<int> openCanPort(const char* portName);

std::optional<can_frame> readFrame(int fd);
}  // namespace WaveshareCAN