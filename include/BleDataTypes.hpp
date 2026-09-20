#pragma once

#include <stdint.h>

namespace w8band::Hardware::BleTypes
{
/// @brief Commands accepted through the Control Point characteristic.
enum class BleCommandType : uint8_t
{
    /// @brief No valid command was received.
    None = 0x00,

    /// @brief Starts a new device calibration.
    Calibrate = 0x01
};

/// @brief Result returned after handling a Control Point command.
enum class CommandResult : uint8_t
{
    /// @brief The command was accepted.
    Ok = 0x00,

    /// @brief The command value is not supported.
    InvalidCommand = 0x01,

    /// @brief The command cannot run in the current device state.
    InvalidState = 0x02,

    /// @brief The device cannot accept another command yet.
    Busy = 0x03
};

/// @brief Stable values exposed by the Status GATT characteristic.
enum class BleDeviceStatus : uint8_t
{
    /// @brief Device services are starting.
    Initializing = 0x00,

    /// @brief Device is waiting for a calibration command.
    Idle = 0x01,

    /// @brief Device is calculating sensor bias.
    Calibrating = 0x02,

    /// @brief Device is waiting for the repetition to start.
    Buffering = 0x03,

    /// @brief Device is recording one repetition.
    Recording = 0x04,

    /// @brief Device is calculating the result.
    Processing = 0x05,

    /// @brief Device is ready to record a repetition.
    Armed = 0x06,

    /// @brief A processed result is ready for transmission.
    ResultReady = 0x07,

    /// @brief Device entered an error condition.
    Error = 0xFF
};

/// @brief Command received from the Control Point characteristic.
struct BleCommand
{
    /// @brief Decoded command value.
    BleCommandType command = BleCommandType::None;
};

/// @brief Two-byte response sent through the Control Point characteristic.
struct CommandResponse
{
    /// @brief Command value copied from the request.
    uint8_t command;

    /// @brief Command handling result stored as a CommandResult value.
    uint8_t result;
};

/// @brief Need to split transmition into these phases
enum class ResultTxPhase : uint8_t
{
    /// @brief No result transmission is active.
    Idle,

    /// @brief General result information is sent.
    Begin,

    /// @brief Repetition timing values are sent.
    Timing,

    /// @brief Motion metrics are sent.
    Motion,

    /// @brief Power metrics are sent when available.
    Power,

    /// @brief Trajectory points are sent in chunks.
    Trajectory,

    /// @brief The result transmission is finished.
    End
};

/// @brief MessageType sent in each transmition, so reciever knows what is being sent
enum class ResultMessageType : uint8_t
{
    /// @brief General result information frame.
    Begin = 0x10,

    /// @brief Repetition timing frame.
    Timing = 0x11,

    /// @brief Motion metrics frame.
    Motion = 0x12,

    /// @brief Power metrics frame.
    Power = 0x13,

    /// @brief Trajectory chunk frame.
    Trajectory = 0x14,

    /// @brief Final result frame.
    End = 0x1F
};

} // namespace w8band::Hardware::BleTypes
