#include "IdleState.hpp"

namespace w8band::StateMachine
{
IdleState::IdleState(DataContext &rDataCtx, W8BandFsm &rFsm)
    : fsm::IState<DataContext, StateId>(rDataCtx), m_rFsm(rFsm)
{}

void IdleState::OnEnter() {}
void IdleState::OnExit() {}

void IdleState::Update()
{
    Serial.println("Transfering to CALIBRATION");
    m_rFsm.RequestTransition(StateId::CalibrationState);
}

StateId IdleState::GetStateId() const { return StateId::IdleState; }

std::string IdleState::GetStateName() { return "IdleState"; }
} // namespace w8band::StateMachine
