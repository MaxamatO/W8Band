#pragma once

#include "DataContext.hpp"
#include "IState.hpp"
#include "W8BandFsm.hpp"
#include "helpers/EndMotionDetector.hpp"
#include "helpers/ProcessingHelpers.hpp"
#include <string>

#define END_MOTION_WINDOW_SIZE 24

namespace w8band::StateMachine
{
/// Online phases only gate the end detector. The precise turnaround sample is
/// still determined later from the complete, zero-velocity-corrected signal.
enum class RecordingPhase
{
    /// Waiting for confirmed negative vertical velocity.
    AwaitingEccentric,
    /// Barbell is moving down; waiting for confirmed upward reversal.
    Eccentric,
    /// Barbell is moving up; monitoring terminal deceleration.
    Concentric,
    /// Strict end thresholds passed; validating them with hysteresis.
    StopCandidate,
    /// End confirmed; collecting a stationary tail for ZUPT.
    PostStopCapture
};

/// @brief Records one repetition and detects its end using phase history,
/// filtered vertical acceleration, integrated velocity and rolling stillness.
class RecordingState : public fsm::IState<DataContext, StateId>
{
public:
    /// @brief Constructs RecordingState using shared data and its parent FSM.
    /// @param[in,out] rDataCtx Shared acquisition and recording context.
    /// @param[in,out] rFsm State machine used to request transitions.
    RecordingState(DataContext &rDataCtx, W8BandFsm &rFsm);

    /// @brief Resets online processing and replays the pre-record buffer.
    void OnEnter() override;

    /// @brief Handles leaving RecordingState. Currently performs no cleanup.
    void OnExit() override;

    /// @brief Acquires and processes the next sample or fires a safety stop.
    void Update() override;

    /// @brief Returns the identifier used by the state machine.
    /// @return StateId::RecordingState.
    StateId GetStateId() const override;

    /// @brief Returns a human-readable state name.
    /// @return Recording state name.
    std::string GetStateName() const override;

private:
    /// @brief Updates the online signal path using one synchronized IMU sample.
    /// @param[in] rPacket Sample containing sensor timestamp, acceleration,
    /// gravity vector and orientation quaternion.
    void ProcessSample(const data::SamplePacket &rPacket);

    /// @brief Evaluates phase-transition conditions for the current sample.
    /// @param[in] timestampTicks Raw LSM6DSV16X timestamp of current sample.
    void UpdatePhase(uint32_t timestampTicks);

    /// @brief Changes phase and initializes phase-specific timestamps.
    /// @param[in] phase New recording phase.
    /// @param[in] timestampTicks Raw timestamp at which phase was entered.
    void SetPhase(RecordingPhase phase, uint32_t timestampTicks);

    /// @brief Confirms that a directional condition remains true for a dwell.
    /// @param[in] condition Current directional condition value.
    /// @param[in] timestampTicks Raw timestamp of current sample.
    /// @param[in] dwellMs Required uninterrupted dwell duration in milliseconds.
    /// @return True once the condition has remained true for dwellMs.
    bool UpdateDirectionalDwell(bool condition, uint32_t timestampTicks,
                                uint32_t dwellMs);

    /// @brief Checks strict thresholds required to enter StopCandidate.
    /// @return True when velocity and both stillness metrics meet entry limits.
    bool IsStrictEndCandidate() const;

    /// @brief Checks wider thresholds used to keep StopCandidate active.
    /// @return True while the candidate remains inside hysteresis limits.
    bool IsRelaxedEndCandidate() const;

    /// @brief State machine used to request transition to ProcessingState.
    W8BandFsm &m_rFsm;

    /// @brief Rolling acceleration statistics used for end detection.
    helpers::EndMotionDetector<END_MOTION_WINDOW_SIZE> m_EndDetector;

    /// @brief Causal filter applied to vertical world-frame acceleration.
    helpers::Processing::LowPassFilter m_VerticalAccelFilter;

    /// @brief Previous quaternion used to preserve quaternion sign continuity.
    helpers::Processing::Quat m_PreviousQuat{1.0f, 0.0f, 0.0f, 0.0f};

    /// @brief Current phase of the online repetition observer.
    RecordingPhase m_Phase = RecordingPhase::AwaitingEccentric;

    /// @brief True after the integrator receives its first sample.
    bool m_HavePrevSample = false;

    /// @brief Previous filtered vertical acceleration in m/s^2.
    float m_PrevFilteredWorldAz = 0.0f;

    /// @brief Raw sensor timestamp of the previous integrated sample.
    uint32_t m_PrevTimestampTicks = 0;

    /// @brief Current online estimate of vertical velocity in m/s.
    float m_CurrVelocityZ = 0.0f;

    /// @brief Maximum vertical velocity observed during concentric phase.
    float m_PeakVelocityZ = 0.0f;

    /// @brief True after velocity drops sufficiently from its concentric peak.
    bool m_TerminalDecelerationSeen = false;

    /// @brief True while a directional transition dwell is being measured.
    bool m_DirectionalDwellActive = false;

    /// @brief Sensor timestamp at which directional dwell began.
    uint32_t m_DirectionalDwellStartedTicks = 0;

    /// @brief Sensor timestamp at which StopCandidate was entered.
    uint32_t m_StopCandidateStartedTicks = 0;

    /// @brief Sensor timestamp at which post-stop capture began.
    uint32_t m_PostStopStartedTicks = 0;
};
} // namespace w8band::StateMachine
