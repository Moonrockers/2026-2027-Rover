// spark_max_can_driver.hpp
//
// C++ SPARK MAX driver using Linux SocketCAN (raw AF_CAN socket).
// Idiomatic C++ translation of the Python python-can based driver.
//
// Hardware setup:
//   Jetson Nano SPI pins -> MCP2515 module -> CAN bus -> SPARK MAX controllers
//   (or any USB-CAN adapter that exposes a SocketCAN interface)
//
// Bring up the CAN interface before running:
//   sudo modprobe mcp251x
//   sudo ip link set can0 up type can bitrate 1000000
//   sudo ip link set can0 txqueuelen 1000
//
// This driver expects the caller to open and bind a SocketCAN raw socket
// (AF_CAN, SOCK_RAW, CAN_RAW) and share the file descriptor across every
// SparkMaxCanDriver instance on the same bus, mirroring how the Python
// version shares a single python-can Bus object across motors.
//
// REV SPARK MAX CAN ID (29-bit extended):
//   Bits [28:24] = 0x02  Device Type  (Motor Controller)
//   Bits [23:16] = 0x05  Manufacturer (REV Robotics)
//   Bits [15:6]  = API Class + Index
//   Bits [5:0]   = Device ID (0-62)

#pragma once

#include <linux/can.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <thread>

namespace moonrockersrev {

// ---------------------------------------------------------------------------
// REV SPARK MAX CAN constants
// ---------------------------------------------------------------------------

namespace detail {

constexpr uint8_t kDeviceType   = 0x02;  // Motor Controller
constexpr uint8_t kManufacturer = 0x05;  // REV Robotics

// Outbound / inbound API IDs (bits [15:6] of the 29-bit CAN ID)
enum class Api : uint16_t {
    kCmdDutyCycle = 0x002,
    kCmdSpeed     = 0x012,
    kCmdPosition  = 0x032,
    kCmdFollower  = 0x062,

    kCfgIdleMode      = 0x054,
    kCfgMotor         = 0x056,  // inverted flag
    kCfgPidP          = 0x058,
    kCfgPidI          = 0x05A,
    kCfgPidD          = 0x05C,
    kCfgPidF          = 0x05E,
    kCfgPidIZone      = 0x060,
    kCfgOutputMin     = 0x064,
    kCfgOutputMax     = 0x066,
    kCfgSoftLimitFwd  = 0x068,
    kCfgSoftLimitRev  = 0x06A,
    kCfgSoftLimitEn   = 0x06C,
    kCfgBurnFlash     = 0x072,

    kCmdClearFaults = 0x06E,

    // Incoming periodic status frame IDs
    kStatus0 = 0x060,  // Applied output, faults        (10 ms)
    kStatus1 = 0x061,  // Velocity, temp, voltage, cur   (20 ms)
    kStatus2 = 0x062,  // Position                       (20 ms)
};

constexpr uint32_t BuildCanId(uint16_t apiId, uint8_t deviceId) {
    return ((static_cast<uint32_t>(kDeviceType) & 0x1Fu) << 24) |
           ((static_cast<uint32_t>(kManufacturer) & 0xFFu) << 16) |
           ((static_cast<uint32_t>(apiId) & 0x3FFu) << 6) |
           (static_cast<uint32_t>(deviceId) & 0x3Fu);
}

constexpr uint16_t ExtractApiId(uint32_t canId) { return (canId >> 6) & 0x3FFu; }
constexpr uint8_t ExtractDeviceId(uint32_t canId) { return canId & 0x3Fu; }
constexpr uint8_t ExtractManufacturer(uint32_t canId) { return (canId >> 16) & 0xFFu; }

}  // namespace detail

enum class ControlMode : uint8_t {
    kDutyCycle = 0,
    kVelocity  = 1,
    kPosition  = 3,
    kFollower  = 5,
};

enum class IdleMode : uint8_t {
    kCoast = 0,
    kBrake = 1,
};

enum class Fault : uint16_t {
    kBrownout     = 1u << 0,
    kOvercurrent  = 1u << 1,
    kIWDTReset    = 1u << 2,
    kMotorFault   = 1u << 3,
    kSensorFault  = 1u << 4,
    kStall        = 1u << 5,
    kEEPROMCRC    = 1u << 6,
    kCANTx        = 1u << 7,
    kCANRx        = 1u << 8,
    kHasReset     = 1u << 9,
    kDRVFault     = 1u << 10,
    kOtherFault   = 1u << 11,
    kSoftLimitFwd = 1u << 12,
    kSoftLimitRev = 1u << 13,
    kHardLimitFwd = 1u << 14,
    kHardLimitRev = 1u << 15,
};

// ---------------------------------------------------------------------------
// Telemetry snapshot
// ---------------------------------------------------------------------------

struct SparkMaxStatus {
    // Status 0
    float appliedOutput = 0.0f;
    uint16_t faults = 0;
    uint16_t stickyFaults = 0;
    bool isInverted = false;
    bool motorRunning = false;
    // Status 1
    float velocityRpm = 0.0f;    // RPM
    float temperatureC = 0.0f;   // degrees C
    float busVoltage = 0.0f;     // V
    float currentAmps = 0.0f;    // A
    // Status 2
    float positionRotations = 0.0f;  // rotations
};

// ---------------------------------------------------------------------------
// SparkMaxCanDriver
// ---------------------------------------------------------------------------

// Driver for a single REV SPARK MAX over a shared SocketCAN raw socket.
//
// Usage:
//   int fd = OpenAndBindSocketCan("can0");   // caller-provided
//   revrobotics::SparkMaxCanDriver motor(fd, /*deviceId=*/1);
//   motor.SetIdleMode(revrobotics::IdleMode::kBrake);
//   motor.SetPower(0.5f);   // 50% duty cycle forward
//   ...
//   motor.SetPower(0.0f);
//
// The driver does not open, bind, or close the socket -- that is the
// caller's responsibility, since the fd is typically shared across every
// motor on the bus.
class SparkMaxCanDriver {
public:
    // Keep-alive interval (SPARK MAX faults if no frame for ~1 s).
    static constexpr std::chrono::milliseconds kKeepAliveInterval{25};

    // fd must be an already-open, bound AF_CAN/SOCK_RAW/CAN_RAW socket.
    // deviceId is the SPARK MAX CAN device ID (0-62).
    SparkMaxCanDriver(int fd, uint8_t deviceId);
    ~SparkMaxCanDriver();

    SparkMaxCanDriver(const SparkMaxCanDriver&) = delete;
    SparkMaxCanDriver& operator=(const SparkMaxCanDriver&) = delete;

    // -- Core control interface (matches FrcMotorController-style API) ------
    void SetPower(float power);
    void SetInverted(bool invert = true);
    void SetIdleMode(IdleMode mode);
    float GetAppliedOutput() const;
    void ClearFaults();

    // -- Extended control -----------------------------------------------
    void SetControlMode(ControlMode mode);
    void SetReference(float value);
    void SetFollower(uint8_t leaderId, bool invert = false);

    // -- PID configuration -------------------------------------------------
    void SetPID(float kP, float kI, float kD, float kF = 0.0f, int slot = 0);
    void SetIZone(float iZone, int slot = 0);
    void SetOutputRange(float minOutput, float maxOutput, int slot = 0);

    // -- Soft limits ---------------------------------------------------
    void EnableSoftLimits(bool forward, bool reverse);
    void SetSoftLimit(bool forward, float limitRotations);

    // -- Flash -----------------------------------------------------------
    void BurnFlash();

    // -- Telemetry accessors ---------------------------------------------
    float GetVelocity() const;
    float GetPosition() const;
    float GetTemperature() const;
    float GetBusVoltage() const;
    float GetCurrent() const;
    uint16_t GetFaults() const;
    uint16_t GetStickyFaults() const;
    bool HasFault(Fault fault) const;
    SparkMaxStatus GetStatus() const;

    // Parse an incoming CAN frame. Call this from your bus receive loop for
    // every frame received on the shared socket:
    //
    //   can_frame frame;
    //   while (read(fd, &frame, sizeof(frame)) == sizeof(frame)) {
    //     for (auto& motor : motors) motor.HandleIncoming(frame);
    //   }
    //
    // Returns true if the frame was addressed to and consumed by this device.
    bool HandleIncoming(const can_frame& frame);

private:
    void SendFloat(uint16_t apiId, float value);
    void SendRaw(uint16_t apiId, const uint8_t* data, uint8_t length);
    void KeepAliveLoop();

    bool ParseStatus0(const uint8_t* data, uint8_t len);
    bool ParseStatus1(const uint8_t* data, uint8_t len);
    bool ParseStatus2(const uint8_t* data, uint8_t len);

    int fd_;
    uint8_t deviceId_;

    std::atomic<ControlMode> controlMode_{ControlMode::kDutyCycle};
    std::atomic<float> lastSetpoint_{0.0f};
    std::atomic<bool> inverted_{false};

    mutable std::mutex statusMutex_;
    SparkMaxStatus status_;

    // Software keep-alive: resends the last setpoint at kKeepAliveInterval.
    // (The Python version prefers the kernel BCM for this; a background
    // thread is the portable equivalent when BCM isn't used directly.)
    std::thread keepAliveThread_;
    std::atomic<bool> running_{true};
};

}  // namespace revrobotics