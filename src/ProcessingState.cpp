#include "ProcessingState.hpp"

namespace w8band::StateMachine
{
ProcessingState::ProcessingState(DataContext &rDataCtx, W8BandFsm &rFsm)
    : fsm::IState<DataContext, StateId>(rDataCtx), m_rFsm(rFsm)
{}

void ProcessingState::OnEnter()
{
    Serial.println(
        "timestamp_ms,qw,qx,qy,qz,ax,ay,az,gvx,gvy,gvz,biasx,biasy,biasz");
    for(const auto &rPacket : m_rContext.m_Data)
    {
        Serial.print(rPacket.timestamp_ms);
        Serial.print(",");
        Serial.print(rPacket.q[0]);
        Serial.print(",");
        Serial.print(rPacket.q[1]);
        Serial.print(",");
        Serial.print(rPacket.q[2]);
        Serial.print(",");
        Serial.print(rPacket.q[3]);
        Serial.print(",");
        Serial.print(rPacket.a[0]);
        Serial.print(",");
        Serial.print(rPacket.a[1]);
        Serial.print(",");
        Serial.print(rPacket.a[2]);
        Serial.print(",");
        Serial.print(rPacket.gv[0]);
        Serial.print(",");
        Serial.print(rPacket.gv[1]);
        Serial.print(",");
        Serial.print(rPacket.gv[2]);
        Serial.print(",");
        Serial.print(m_rContext.m_AccelBias.axBias);
        Serial.print(",");
        Serial.print(m_rContext.m_AccelBias.ayBias);
        Serial.print(",");
        Serial.println(m_rContext.m_AccelBias.azBias);
    }
}
void ProcessingState::OnExit() {}

void ProcessingState::Update() {}

StateId ProcessingState::GetStateId() const
{ return StateId::ProcessingState; }

std::string ProcessingState::GetStateName() const { return "ProcessingState"; }
} // namespace w8band::StateMachine
