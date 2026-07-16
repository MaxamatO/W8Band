#pragma once

namespace w8band::StateMachine
{
/// @brief Enum of all States inside StateMachine
enum class StateId
{
    IdleState,
    CalibrationState,
    BufferringState,
    ArmedState,
    RecordingState,
    ProcessingState,
    SendingState
};
} // namespace w8band::StateMachine
