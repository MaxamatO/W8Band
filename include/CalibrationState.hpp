#pragma once
#include "DataContext.hpp"
#include "IState.hpp"
#include "W8BandFsm.hpp"
#include "helpers/BiasAccumulator.hpp"
#include "helpers/SlidingWindow.hpp"
#include <string>

/// @brief A 60-sample window covers about 0.5 s at a 120 Hz sample rate.
#define WINDOW_SIZE 60

/// @brief Number of stationary samples used to calculate sensor bias.
#define CALIBRATION_DATA_COUNT 360

namespace w8band::StateMachine
{
/// @brief Steps performed during sensor calibration.
enum class CalibrationPhase
{
    /// @brief Waits until the device is held still.
    WaitingForStillness,

    /// @brief Collects stationary sensor samples.
    Accumulating,

    /// @brief Calibration data is ready to be stored.
    Done,

    /// @brief Calibration did not finish before its timeout.
    Failed
};

/// @brief Calibration state - Entered from IdleState on Calibrate command.
/// Device is supposed to lie still on a barbell placed on the rack. Stationary
/// IMU samples are collected for a few seconds to calculate a constant bias.
class CalibrationState : public fsm::IState<DataContext, StateId>
{
public:
    /// @brief Constructs CalibrationState using shared data and its parent FSM.
    /// @param[in,out] rDataCtx Shared acquisition and calibration context.
    /// @param[in,out] rFsm State machine used to request transitions.
    CalibrationState(DataContext &rDataCtx, W8BandFsm &rFsm);

    /// @brief @see IState::OnEnter()
    void OnEnter() override;

    /// @brief @see IState::OnExit()
    void OnExit() override;

    /// @brief @see IState::Update()
    void Update() override;

    /// @brief @see IState::GetSateId()
    /// @return StateId::CalibrationState.
    StateId GetStateId() const override;

    /// @brief Human-readable name, kept separate from GetStateId() so the
    ///        FSM's internal lookups stay a cheap enum compare.
    /// @return Calibration state name.
    std::string GetStateName() const override;

private:
    /// @brief Reference to State Machine for state transition
    W8BandFsm &m_rFsm;

    /// @brief Used for determining phase of calibration
    CalibrationPhase m_Phase;

    /// @brief Peephole/window struct used for determining device's stillness
    helpers::StillnessDetector<WINDOW_SIZE> m_Window;

    /// @brief Struct used to calcualte Accelerometer bias in flight
    BiasAccumulator m_BiasAccumulator;

    /// @brief Count of samples left for discarding - trash before FIFO is
    /// stable. Set OnEnter
    uint8_t m_DiscardCount;

    /// @brief Time in ms at which CalibrationState was entered.
    uint32_t m_StartedAtMs;

    /// @brief Helper method for handling CalibrationPhase::WaitingForStillness
    /// Moves sliding window across obtained data in order to determine if the
    /// device is steady
    /// @param[in] rPacket Current synchronized IMU sample.
    void StillnessWaitHandler(data::SamplePacket &rPacket);

    /// @brief Helper method for accumulating data for 3s in order to calculate
    /// bias and gravity. If during that phase, device moves, and
    /// StillnessDetector::IsStill() returns false, we go back to
    /// WaitingForStillness phase. If not, we transfer to CalibrationDone
    /// phase.
    /// @param[in] rPacket Reference to obtained data packet for storing.
    void AccumulatingHandler(data::SamplePacket &rPacket);

    /// @brief Stores the calculated accelerometer bias and leaves calibration.
    void CalibrationDoneHandler();

    /// @brief Invalidates calibration after a timeout or failed calibration.
    void CalibratingErrorHandler();
};

} // namespace w8band::StateMachine
