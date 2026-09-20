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
    /// @brief Constructs IdleState using shared data and its parent FSM.
    /// @param[in,out] rDataCtx Shared application context.
    /// @param[in,out] rFsm State machine used to request transitions.
    IdleState(DataContext &rDataCtx, W8BandFsm &rFsm);

    /// @brief Resets data from the previous measurement.
    void OnEnter() override;

    /// @brief Handles leaving IdleState. Currently performs no cleanup.
    void OnExit() override;

    /// @brief Waits for commands handled by the service manager.
    void Update() override;

    /// @brief Returns the identifier used by the state machine.
    /// @return StateId::IdleState.
    StateId GetStateId() const override;

    /// @brief Human-readable name, kept separate from GetStateId() so the
    ///        FSM's internal lookups stay a cheap enum compare.
    /// @return Idle state name.
    std::string GetStateName() const override;

private:
    /// @brief State machine used to request transitions.
    W8BandFsm &m_rFsm;
};

} // namespace w8band::StateMachine
