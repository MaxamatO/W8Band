#pragma once
#include "DataContext.hpp"
#include "IState.hpp"
#include "W8BandFsm.hpp"
#include "helpers/BiasAccumulator.hpp"
#include "helpers/SlidingWindow.hpp"
#include <string>

/// @brief Number of samples used to detect stillness and motion start.
#define MOTION_START_WINDOW_SIZE 24

namespace w8band::StateMachine
{
/// @brief Bufferring phases for correctly determining start of lift.
/// WaitForLift waits for Wake Up interrupt to know, that user has taken the
/// barbell off the rack. WaitStillness phase waits for bar to be still for
/// around 0.2s. WaitMotion phase waits for Not Still signal from
/// StillnessDetector to know that user has began their rep.
enum class BufferringPhase
{
    /// @brief Waits for the IMU lift-off interrupt.
    WaitLiftOff,

    /// @brief Waits until the barbell becomes still after lift-off.
    WaitStillness,

    /// @brief Waits for movement that starts the repetition.
    WaitMotion
};

/// @brief Waits for lift-off, a short still period and the start of movement.
/// Keeps a rolling pre-record buffer and then enters RecordingState.
class BufferringState : public fsm::IState<DataContext, StateId>
{
public:
    /// @brief Constructs BufferringState using shared data and its parent FSM.
    /// @param[in,out] rDataCtx Shared acquisition and recording context.
    /// @param[in,out] rFsm State machine used to request transitions.
    BufferringState(DataContext &rDataCtx, W8BandFsm &rFsm);

    /// @brief @see IState::OnEnter()
    void OnEnter() override;

    /// @brief @see IState::OnExit()
    void OnExit() override;

    /// @brief @see IState::Update()
    void Update() override;

    /// @brief @see IState::GetSateId()
    /// @return StateId::BufferringState.
    StateId GetStateId() const override;

    /// @brief Human-readable name, kept separate from GetStateId() so the
    ///        FSM's internal lookups stay a cheap enum compare.
    /// @return Bufferring state name.
    std::string GetStateName() const override;

private:
    /// @brief Helper method for handling WaitLiftOff phase.
    void HandleWaitLiftOff();

    /// @brief Helper method for handling WaitStillness phase.
    /// @param[in] rPacket Current synchronized IMU sample.
    void HandleWaitStillness(data::SamplePacket &rPacket);

    /// @brief Helper method for handling WaitMotion phase.
    /// @param[in] rPacket Current synchronized IMU sample.
    void HandleWaitMotion(data::SamplePacket &rPacket);

    /// @brief Peephole/window struct used for determining device's stillness
    helpers::StillnessDetector<MOTION_START_WINDOW_SIZE> m_Window;

    /// @brief Reference to State Machine for state transition
    W8BandFsm &m_rFsm;

    /// @brief Current buffering phase.
    BufferringPhase m_Phase;

    /// @brief True if device has went under stillness threshold.
    bool m_InStillnessStartDwell = false;

    /// @brief Sensor timestamp at which stillness dwell began.
    uint32_t m_StillnessStartedTicks = 0;

    /// @brief True if device has went through motion threshold.
    bool m_InMotionStartDwell = false;

    /// @brief Sensor timestamp at which motion dwell began.
    uint32_t m_MotionStartedTicks = 0;
};
} // namespace w8band::StateMachine
