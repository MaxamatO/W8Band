#pragma once
#include "DataContext.hpp"
#include "IState.hpp"
#include "W8BandFsm.hpp"
#include <string>

namespace w8band::StateMachine
{

/// @brief Idle state - BLE advertising, device waits for a connection and
/// Calibrate command sent by a user from application.
class IdleState : public fsm::IState<DataContext, StateId>
{
public:
    IdleState(DataContext &rDataCtx, W8BandFsm &rFsm);

    void OnEnter() override;
    void OnExit() override;
    void Update() override;
    StateId GetStateId() const override;

    /// @brief Human-readable name, kept separate from GetStateId() so the
    ///        FSM's internal lookups stay a cheap enum compare.
    static std::string GetStateName();

private:
    W8BandFsm &m_rFsm;
};

} // namespace w8band::StateMachine
