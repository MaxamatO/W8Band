#pragma once
#include "DataContext.hpp"
#include "IState.hpp"
#include "W8BandFsm.hpp"
#include "helpers/BiasAccumulator.hpp"
#include "helpers/SlidingWindow.hpp"
#include <string>

namespace w8band::StateMachine
{
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
    /// @brief Reference to State Machine for state transition
    W8BandFsm &m_rFsm;
};
}