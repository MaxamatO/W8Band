#include "ArmedState.hpp"

namespace w8band::StateMachine
{
ArmedState::ArmedState(DataContext &rDataCtx, W8BandFsm &rFsm)
    : fsm::IState<DataContext, StateId>(rDataCtx), m_rFsm(rFsm)
{}

void ArmedState::OnEnter() { m_rContext.m_rLsmServiceManager.ResetFIFO(); }
void ArmedState::OnExit() {}

void ArmedState::Update()
{
    // Serial.printf("Currently inside ArmedState::Update.");
    // Serial.printf("Requesting transition to ArmedState.");
    // m_rContext.m_rLsmServiceManager.ObtainData(m_SamplePacket);
    // Serial.print("Quaternion z sample packet: ");
    // Serial.print(m_SamplePacket.q[3], 4);
    // Serial.print(", ");
    // Serial.print(m_SamplePacket.q[0], 4);
    // Serial.print(", ");
    // Serial.print(m_SamplePacket.q[1], 4);
    // Serial.print(", ");
    // Serial.println(m_SamplePacket.q[2], 4);
    // m_rFsm.RequestTransition(StateId::ArmedState);
    // Example of how a state requests a transition - the switch is
    // applied at the top of the *next* Fsm.Update() call, not here.
    // if(v_WakeUpDetected)
    //     m_rFsm.RequestTransition(StateId::ArmedState);
}

StateId ArmedState::GetStateId() const { return StateId::ArmedState; }

std::string ArmedState::GetStateName() { return "ArmedState"; }

} // namespace w8band::StateMachine
