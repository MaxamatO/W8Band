#pragma once
#include "DataContext.hpp"
#include "IState.hpp"
#include "W8BandFsm.hpp"
#include "helpers/BiasAccumulator.hpp"
#include "helpers/SlidingWindow.hpp"
#include <string>

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
    WaitLiftOff,
    WaitStillness,
    WaitMotion
};

/// @brief BufferringState entered after a successful calibration. Resests m_WakeupDetected
/// flag, keeps populating rolling buffer for PRE_ROCORD_SAMPLES amount. Waits
/// for another m_WakeupDetected flag set for transition to ArmedState
class BufferringState : public fsm::IState<DataContext, StateId>
{
public:
    BufferringState(DataContext &rDataCtx, W8BandFsm &rFsm);

    /// @brief @see IState::OnEnter()
    void OnEnter() override;

    /// @brief @see IState::OnExit()
    void OnExit() override;

    /// @brief @see IState::Update()
    void Update() override;

    /// @brief @see IState::GetSateId()
    StateId GetStateId() const override;

    /// @brief Human-readable name, kept separate from GetStateId() so the
    ///        FSM's internal lookups stay a cheap enum compare.
    std::string GetStateName() const override;

private:
    /// @brief Helper method for handling WaitLiftOff phase.
    void HandleWaitLiftOff();

    /// @brief Helper method for handling WaitStillness phase.
    void HandleWaitStillness(data::SamplePacket &rPacket);

    /// @brief Helper method for handling WaitMotion phase.
    void HandleWaitMotion(data::SamplePacket &rPacket);

    /// @brief Peephole/window struct used for determining device's stillness
    helpers::StillnessDetector<MOTION_START_WINDOW_SIZE> m_Window;

    /// @brief Reference to State Machine for state transition
    W8BandFsm &m_rFsm;

    /// @brief Reference to current bufferring phase
    BufferringPhase m_Phase;

    /// @brief Time in ms in which lift off occurred.
    uint32_t m_LiftOffBeganMs;

    /// @brief True if device has went under stillness threshold.
    bool m_InStillnessStartDwell = false;

    /// @brief Time in ms since device triggerred stillness dwell.
    uint32_t m_InStillnessTimeMs = 0;

    /// @brief True if device has went through motion threshold.
    bool m_InMotionStartDwell = false;

    /// @brief Time in ms since m_InMotionStartDwell is true.
    uint32_t m_InMotionTimeMs = 0;
};
}