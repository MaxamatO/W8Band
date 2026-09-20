#pragma once

namespace w8band::StateMachine
{
/// @brief Enum of all States inside StateMachine
enum class StateId
{
    /// @brief Device waits for a calibration command.
    IdleState,

    /// @brief Device calculates accelerometer bias.
    CalibrationState,

    /// @brief Device waits for lift-off and movement.
    BufferringState,

    /// @brief Reserved state for an armed device.
    ArmedState,

    /// @brief Device records one repetition.
    RecordingState,

    /// @brief Device calculates trajectory and motion metrics.
    ProcessingState,

    /// @brief Reserved state for result transmission.
    SendingState
};
} // namespace w8band::StateMachine
