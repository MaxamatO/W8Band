#include "ErrorState.hpp"

namespace w8band::StateMachine
{
ErrorState::ErrorState(DataContext &rDataCtx, W8BandFsm &rFsm)
    : fsm::IState<DataContext, StateId>(rDataCtx), m_rFsm(rFsm)
{}

void ErrorState::OnEnter() { m_rContext.Reset(); }
void ErrorState::OnExit() {}

void ErrorState::Update()
{
    // Does nothing - waits for User Reset command
}

StateId ErrorState::GetStateId() const { return StateId::ErrorState; }

std::string ErrorState::GetStateName() const { return "ErrorState"; }
} // namespace w8band::StateMachine
