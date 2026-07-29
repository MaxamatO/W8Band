#include "BufferringState.hpp"

namespace w8band::StateMachine
{
BufferringState::BufferringState(DataContext &rDataCtx, W8BandFsm &rFsm)
    : IState<DataContext, StateId>(rDataCtx), m_rFsm(rFsm)
{}

/// @brief @see IState::OnEnter()
void BufferringState::OnEnter()
{
    m_rContext.m_PreIndex = 0;
    m_rContext.m_PreCount = 0;
    m_rContext.m_WakeUpDetected = false;
}

/// @brief @see IState::OnExit()
void BufferringState::OnExit() {}

/// @brief @see IState::Update()
void BufferringState::Update()
{
    data::SamplePacket packet;
    m_rContext.m_rLsmServiceManager.ObtainData(packet);
    m_rContext.PushToPreBuffer(packet);
    if(m_rContext.m_WakeUpDetected)
    {
        Serial.println("Detected 2nd WU_INT, transitioning to ArmedState");
        m_rContext.m_WakeUpDetected = false;
        m_rFsm.RequestTransition(StateId::ArmedState);
    }
}

StateId BufferringState::GetStateId() const
{ return StateId::BufferringState; }
std::string BufferringState::GetStateName() const
{ return "Bufferring State"; }
} // namespace
