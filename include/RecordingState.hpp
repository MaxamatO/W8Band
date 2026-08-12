#pragma once
#include "DataContext.hpp"
#include "DataTypes.hpp"
#include "IState.hpp"
#include "W8BandFsm.hpp"
#include "helpers/SlidingWindow.hpp"

#define STOP_MOTION_WINDOW_SIZE 80

/// Safety recording timeout, in case end-of-motion never fires cleanly
#define RECORDING_TIMEOUT_MS 8000

/// Value in ms describing how long a signal must stay near-zero in order to fire end-of-motion
#define END_OF_MOTION_DWELL_MS 250

/// @brief Describes current motion direction during Recording State
enum class MotionDirection
{
    Unknown,
    GoingUp,
    GoingDown,
    NearZero
};

namespace w8band::StateMachine
{
class RecordingState : public fsm::IState<DataContext, StateId>
{
public:
    RecordingState(DataContext &rDataCtx, W8BandFsm &rFsm);

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
    /// @brief Helper for setting direction using Schmitt Hysteresis for more accurate Motion Direction
    void UpdateDirection();

    /// @brief Helper for calculating velocity in real time in order to
    /// accuratelly determine end of motion and set proper MotinDirection
    /// @param[in] rPacket Reference to currently obtained data for calculation
    void IntegrateVelocityRT(data::SamplePacket &rPacket);

    /// @brief Helper used to check the direction and properly transtition to
    /// ProcessingState based on device's OZ velocity
    void CheckForEndOfMotion();

    /// @brief Reference to State Machine for state transition
    W8BandFsm &m_rFsm;

    /// @brief Stillness detector for end of motion detection
    helpers::StillnessDetector<STOP_MOTION_WINDOW_SIZE> m_Window;

    /// @brief Tells whether we have previous sample packet for integrating
    bool m_HavePrevSample = false;

    /// @brief Acceleration in Z axis 1 sample before
    float m_PrevWorldAz = 0.0f;

    /// @brief Timestamp of 1 sample before
    uint32_t m_PrevTimestampMs = 0;

    /// @brief Current direction
    MotionDirection m_Direction;

    /// @brief Current velocity in Z axis
    float m_CurrVelocityZ = 0.0f;

    /// @brief Set when device's velocity has passed kUpThreshold
    bool m_HasSeenGoingUp = false;

    /// @brief True if device is in near zero state
    bool m_IsNearZeroDwell;

    /// @brief Time in ms in which the device is in near zero state
    uint32_t m_NearZeroTimeMs;
};
}