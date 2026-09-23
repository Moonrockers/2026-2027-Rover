// spark_max_can_driver.cpp

#include "spark_max/sparkMaxDriver.h"

#include <unistd.h>

#include <algorithm>
#include <cstring>
#include <iostream>

namespace moonrockersrev {

using detail::Api;
using detail::BuildCanId;
using detail::ExtractApiId;
using detail::ExtractDeviceId;
using detail::ExtractManufacturer;
using detail::kManufacturer;

namespace {

float Clamp(float value, float lo, float hi) {
  return std::max(lo, std::min(hi, value));
}

}  // namespace

// ---------------------------------------------------------------------------
// Construction / destruction
// ---------------------------------------------------------------------------

SparkMaxCanDriver::SparkMaxCanDriver(int fd, uint8_t deviceId)
    : fd_(fd), deviceId_(deviceId) {
  if (deviceId_ > 62) {
    throw std::invalid_argument("Device ID must be 0-62");
  }

  // Software keep-alive thread: resend the last setpoint frame at
  // kKeepAliveInterval so the SPARK MAX watchdog doesn't fault.
  keepAliveThread_ = std::thread(&SparkMaxCanDriver::KeepAliveLoop, this);

  // std::cerr << "SparkMaxCanDriver: device_id=" << static_cast<int>(deviceId_)
  //           << " initialized\n";
}

SparkMaxCanDriver::~SparkMaxCanDriver() {
  running_ = false;
  if (keepAliveThread_.joinable()) {
    keepAliveThread_.join();
  }
}

// ---------------------------------------------------------------------------
// Core control interface
// ---------------------------------------------------------------------------

void SparkMaxCanDriver::SetPower(float power) {
  power = Clamp(power, -1.0f, 1.0f);
  controlMode_ = ControlMode::kDutyCycle;
  lastSetpoint_ = power;
  SendFloat(static_cast<uint16_t>(Api::kCmdDutyCycle), power);
}

void SparkMaxCanDriver::SetInverted(bool invert) {
  inverted_ = invert;
  uint8_t buf[8] = {0};
  buf[3] = invert ? 0x01 : 0x00;
  SendRaw(static_cast<uint16_t>(Api::kCfgMotor), buf, sizeof(buf));
}

void SparkMaxCanDriver::SetIdleMode(IdleMode mode) {
  uint8_t buf[8] = {0};
  buf[0] = static_cast<uint8_t>(mode);
  SendRaw(static_cast<uint16_t>(Api::kCfgIdleMode), buf, 1);
}

float SparkMaxCanDriver::GetAppliedOutput() const {
  std::lock_guard<std::mutex> lock(statusMutex_);
  return status_.appliedOutput;
}

void SparkMaxCanDriver::ClearFaults() {
  uint8_t buf[8] = {0};
  SendRaw(static_cast<uint16_t>(Api::kCmdClearFaults), buf, 1);
}

// ---------------------------------------------------------------------------
// Extended control
// ---------------------------------------------------------------------------

void SparkMaxCanDriver::SetControlMode(ControlMode mode) {
  controlMode_ = mode;
}

void SparkMaxCanDriver::SetReference(float value) {
  uint16_t apiId;
  switch (controlMode_.load()) {
    case ControlMode::kDutyCycle:
      value = Clamp(value, -1.0f, 1.0f);
      apiId = static_cast<uint16_t>(Api::kCmdDutyCycle);
      break;
    case ControlMode::kVelocity:
      apiId = static_cast<uint16_t>(Api::kCmdSpeed);
      break;
    case ControlMode::kPosition:
      apiId = static_cast<uint16_t>(Api::kCmdPosition);
      break;
    case ControlMode::kFollower:
      // std::cerr
      //     << "Use SetFollower() for follower mode, ignoring
      //     SetReference()\n";
      return;
    default:
      apiId = static_cast<uint16_t>(Api::kCmdDutyCycle);
      break;
  }

  lastSetpoint_ = value;
  SendFloat(apiId, value);
}

void SparkMaxCanDriver::SetFollower(uint8_t leaderId, bool invert) {
  controlMode_ = ControlMode::kFollower;

  // Build the leader's full 29-bit CAN device ID (API = 0 for device address).
  const uint32_t leaderFullId = BuildCanId(0, leaderId);

  uint8_t buf[8] = {0};
  buf[0] = static_cast<uint8_t>(leaderFullId & 0xFF);
  buf[1] = static_cast<uint8_t>((leaderFullId >> 8) & 0xFF);
  buf[2] = static_cast<uint8_t>((leaderFullId >> 16) & 0xFF);
  buf[3] = static_cast<uint8_t>((leaderFullId >> 24) & 0xFF);
  buf[4] = invert ? 0x40 : 0x00;  // bit 6 = invert

  SendRaw(static_cast<uint16_t>(Api::kCmdFollower), buf, sizeof(buf));
}

// ---------------------------------------------------------------------------
// PID configuration
// ---------------------------------------------------------------------------

void SparkMaxCanDriver::SetPID(float kP, float kI, float kD, float kF,
                               int slot) {
  const uint16_t offset = static_cast<uint16_t>(slot * 0x10);
  SendFloat(static_cast<uint16_t>(Api::kCfgPidP) + offset, kP);
  SendFloat(static_cast<uint16_t>(Api::kCfgPidI) + offset, kI);
  SendFloat(static_cast<uint16_t>(Api::kCfgPidD) + offset, kD);
  SendFloat(static_cast<uint16_t>(Api::kCfgPidF) + offset, kF);
}

void SparkMaxCanDriver::SetIZone(float iZone, int slot) {
  const uint16_t offset = static_cast<uint16_t>(slot * 0x10);
  SendFloat(static_cast<uint16_t>(Api::kCfgPidIZone) + offset, iZone);
}

void SparkMaxCanDriver::SetOutputRange(float minOutput, float maxOutput,
                                       int slot) {
  const uint16_t offset = static_cast<uint16_t>(slot * 0x10);
  SendFloat(static_cast<uint16_t>(Api::kCfgOutputMin) + offset, minOutput);
  SendFloat(static_cast<uint16_t>(Api::kCfgOutputMax) + offset, maxOutput);
}

// ---------------------------------------------------------------------------
// Soft limits
// ---------------------------------------------------------------------------

void SparkMaxCanDriver::EnableSoftLimits(bool forward, bool reverse) {
  uint8_t buf[8] = {0};
  buf[0] = forward ? 0x01 : 0x00;
  buf[1] = reverse ? 0x01 : 0x00;
  SendRaw(static_cast<uint16_t>(Api::kCfgSoftLimitEn), buf, 2);
}

void SparkMaxCanDriver::SetSoftLimit(bool forward, float limitRotations) {
  const uint16_t apiId = forward ? static_cast<uint16_t>(Api::kCfgSoftLimitFwd)
                                 : static_cast<uint16_t>(Api::kCfgSoftLimitRev);
  SendFloat(apiId, limitRotations);
}

// ---------------------------------------------------------------------------
// Flash
// ---------------------------------------------------------------------------

void SparkMaxCanDriver::BurnFlash() {
  uint8_t buf[8] = {0};
  buf[0] = 0xA3;
  buf[1] = 0x3A;
  SendRaw(static_cast<uint16_t>(Api::kCfgBurnFlash), buf, 2);
}

// ---------------------------------------------------------------------------
// Telemetry accessors
// ---------------------------------------------------------------------------

float SparkMaxCanDriver::GetVelocity() const {
  std::lock_guard<std::mutex> lock(statusMutex_);
  return status_.velocityRpm;
}

float SparkMaxCanDriver::GetPosition() const {
  std::lock_guard<std::mutex> lock(statusMutex_);
  return status_.positionRotations;
}

float SparkMaxCanDriver::GetTemperature() const {
  std::lock_guard<std::mutex> lock(statusMutex_);
  return status_.temperatureC;
}

float SparkMaxCanDriver::GetBusVoltage() const {
  std::lock_guard<std::mutex> lock(statusMutex_);
  return status_.busVoltage;
}

float SparkMaxCanDriver::GetCurrent() const {
  std::lock_guard<std::mutex> lock(statusMutex_);
  return status_.currentAmps;
}

uint16_t SparkMaxCanDriver::GetFaults() const {
  std::lock_guard<std::mutex> lock(statusMutex_);
  return status_.faults;
}

uint16_t SparkMaxCanDriver::GetStickyFaults() const {
  std::lock_guard<std::mutex> lock(statusMutex_);
  return status_.stickyFaults;
}

bool SparkMaxCanDriver::HasFault(Fault fault) const {
  std::lock_guard<std::mutex> lock(statusMutex_);
  return (status_.faults & static_cast<uint16_t>(fault)) != 0;
}

SparkMaxStatus SparkMaxCanDriver::GetStatus() const {
  std::lock_guard<std::mutex> lock(statusMutex_);
  return status_;
}

// ---------------------------------------------------------------------------
// Incoming frame parsing
// ---------------------------------------------------------------------------

bool SparkMaxCanDriver::HandleIncoming(const can_frame& frame) {
  if (!(frame.can_id & CAN_EFF_FLAG)) {
    return false;  // not an extended-ID frame
  }
  const uint32_t canId = frame.can_id & CAN_EFF_MASK;

  if (ExtractManufacturer(canId) != kManufacturer) return false;
  if (ExtractDeviceId(canId) != deviceId_) return false;

  const uint16_t apiId = ExtractApiId(canId);

  if (apiId == static_cast<uint16_t>(Api::kStatus0)) {
    return ParseStatus0(frame.data, frame.can_dlc);
  } else if (apiId == static_cast<uint16_t>(Api::kStatus1)) {
    return ParseStatus1(frame.data, frame.can_dlc);
  } else if (apiId == static_cast<uint16_t>(Api::kStatus2)) {
    return ParseStatus2(frame.data, frame.can_dlc);
  }

  return false;
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

void SparkMaxCanDriver::SendFloat(uint16_t apiId, float value) {
  uint8_t data[4];
  std::memcpy(data, &value, sizeof(value));  // assumes little-endian host
  SendRaw(apiId, data, sizeof(data));
}

void SparkMaxCanDriver::SendRaw(uint16_t apiId, const uint8_t* data,
                                uint8_t length) {
  can_frame frame{};
  frame.can_id = BuildCanId(apiId, deviceId_) | CAN_EFF_FLAG;
  frame.can_dlc = std::min<uint8_t>(length, 8);
  std::memcpy(frame.data, data, frame.can_dlc);

  const ssize_t written = write(fd_, &frame, sizeof(frame));
  if (written != sizeof(frame)) {
    // std::cerr << "CAN send error (device " << static_cast<int>(deviceId_)
    //           << ", api 0x" << std::hex << apiId << std::dec << ")\n";
  }
}

void SparkMaxCanDriver::KeepAliveLoop() {
  while (running_.load(std::memory_order_relaxed)) {
    uint16_t apiId = static_cast<uint16_t>(Api::kCmdDutyCycle);
    switch (controlMode_.load()) {
      case ControlMode::kVelocity:
        apiId = static_cast<uint16_t>(Api::kCmdSpeed);
        break;
      case ControlMode::kPosition:
        apiId = static_cast<uint16_t>(Api::kCmdPosition);
        break;
      default:
        break;
    }
    SendFloat(apiId, lastSetpoint_.load());
    std::this_thread::sleep_for(kKeepAliveInterval);
  }
}

bool SparkMaxCanDriver::ParseStatus0(const uint8_t* data, uint8_t len) {
  // [0..1] applied output as int16 / 32767.0  -> [-1, 1]
  // [2..3] faults bitmask (uint16 LE)
  // [4..5] sticky faults bitmask (uint16 LE)
  // [6]    bit 3 = inverted, bit 7 = motor running
  if (len < 7) return false;

  int16_t rawOut;
  uint16_t faults, stickyFaults;
  std::memcpy(&rawOut, data + 0, sizeof(rawOut));
  std::memcpy(&faults, data + 2, sizeof(faults));
  std::memcpy(&stickyFaults, data + 4, sizeof(stickyFaults));

  std::lock_guard<std::mutex> lock(statusMutex_);
  status_.appliedOutput = rawOut / 32767.0f;
  status_.faults = faults;
  status_.stickyFaults = stickyFaults;
  status_.isInverted = (data[6] & 0x08) != 0;
  status_.motorRunning = (data[6] & 0x80) != 0;
  return true;
}

bool SparkMaxCanDriver::ParseStatus1(const uint8_t* data, uint8_t len) {
  // [0..3] velocity (float32 LE, RPM)
  // [4]    temperature (uint8, degrees C)
  // [5..6] bus voltage uint16 LE: value * (32.0 / 4096.0) = volts
  // [7]    current uint8: value * 0.125 = amps
  if (len < 8) return false;

  float velocity;
  uint16_t rawVoltage;
  std::memcpy(&velocity, data + 0, sizeof(velocity));
  std::memcpy(&rawVoltage, data + 5, sizeof(rawVoltage));

  std::lock_guard<std::mutex> lock(statusMutex_);
  status_.velocityRpm = velocity;
  status_.temperatureC = static_cast<float>(data[4]);
  status_.busVoltage = rawVoltage * (32.0f / 4096.0f);
  status_.currentAmps = data[7] * 0.125f;
  return true;
}

bool SparkMaxCanDriver::ParseStatus2(const uint8_t* data, uint8_t len) {
  // [0..3] position (float32 LE, rotations)
  if (len < 4) return false;

  float position;
  std::memcpy(&position, data + 0, sizeof(position));

  std::lock_guard<std::mutex> lock(statusMutex_);
  status_.positionRotations = position;
  return true;
}

}  // namespace moonrockersrev