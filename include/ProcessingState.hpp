#pragma once
#include "DataContext.hpp"
#include "IState.hpp"
#include "W8BandFsm.hpp"
#include <string>

namespace w8band::StateMachine
{
/// @brief Processing state - responsible for analysing complete data and
/// calculating bar trajectory instantly after recording stops.
class ProcessingState : public fsm::IState<DataContext, StateId>
{
public:
    /// @brief Constructs ProcessingState using shared data and its parent FSM.
    /// @param[in,out] rDataCtx Shared raw data, configuration and result.
    /// @param[in,out] rFsm State machine used to request transitions.
    ProcessingState(DataContext &rDataCtx, W8BandFsm &rFsm);

    /// @brief @see IState::OnEnter
    void OnEnter() override;

    /// @brief @see IState::OnExit
    void OnExit() override;

    /// @brief @see IState::Update
    void Update() override;

    /// @brief @see IState::GetStateId
    /// @return StateId::ProcessingState.
    StateId GetStateId() const override;

    /// @brief Human-readable name, kept separate from GetStateId() so the
    ///        FSM's internal lookups stay a cheap enum compare.
    /// @return Processing state name.
    std::string GetStateName() const override;

private:
    /// @brief State machine used to resume buffering after processing.
    W8BandFsm &m_rFsm;
};

} // namespace w8band::StateMachine
