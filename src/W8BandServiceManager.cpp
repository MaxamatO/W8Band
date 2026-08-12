#include "W8BandServiceManager.hpp"
#include <Arduino.h>

namespace w8band
{

static W8BandServiceManager *instance = nullptr;

volatile bool v_WakeUpDetected = false;
volatile static int ISRcount = 0;

W8BandServiceManager::W8BandServiceManager(Hardware::BleServiceManager &rBleSM,
                                           Hardware::LsmServiceManager &rLsmSM,
                                           DataContext &rDataContext)
    : m_rDataContext(rDataContext), m_rBleServiceManager(rBleSM),
      m_rLsmServiceManager(rLsmSM), m_Fsm(m_rDataContext)
{ instance = this; }

void W8BandServiceManager::Init()
{
    m_Fsm.AddState(
        StateMachine::StateId::IdleState,
        std::make_unique<StateMachine::IdleState>(m_rDataContext, m_Fsm));

    m_Fsm.AddState(StateMachine::StateId::CalibrationState,
                   std::make_unique<StateMachine::CalibrationState>(
                       m_rDataContext, m_Fsm));
    m_Fsm.AddState(
        StateMachine::StateId::BufferringState,
        std::make_unique<StateMachine::BufferringState>(m_rDataContext, m_Fsm));

    m_Fsm.AddState(
        StateMachine::StateId::RecordingState,
        std::make_unique<StateMachine::RecordingState>(m_rDataContext, m_Fsm));
    m_Fsm.AddState(
        StateMachine::StateId::ProcessingState,
        std::make_unique<StateMachine::ProcessingState>(m_rDataContext, m_Fsm));
    m_Fsm.Start(StateMachine::StateId::IdleState);
}

void W8BandServiceManager::StartApplication() {}

void W8BandServiceManager::Update()
{
    if(v_WakeUpDetected)
    {
        // Serial.println(ISRcount);
        m_rDataContext.m_LiftOffDetected = true;
        v_WakeUpDetected = false;
    }
    m_Fsm.Update();
}

void W8BandServiceManager::AttachWakeUptInterrupt(uint16_t interruptPin)
{
    attachInterrupt(digitalPinToInterrupt(interruptPin), WakeUpISR1, RISING);
    ISRcount += 1;
}

void W8BandServiceManager::WakeUpISR1() { v_WakeUpDetected = true; }
} // namespace w8band
