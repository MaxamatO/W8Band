#include "W8BandServiceManager.hpp"
#include "BleServiceManager.hpp"
#include "DataTypes.hpp"
#include <Arduino.h>

namespace w8band
{
namespace
{
Hardware::BleTypes::BleDeviceStatus ToBleStatus(StateMachine::StateId state)
{
    switch(state)
    {
    case StateMachine::StateId::IdleState:
        return Hardware::BleTypes::BleDeviceStatus::Idle;
    case StateMachine::StateId::CalibrationState:
        return Hardware::BleTypes::BleDeviceStatus::Calibrating;
    case StateMachine::StateId::BufferringState:
        return Hardware::BleTypes::BleDeviceStatus::Buffering;
    case StateMachine::StateId::ArmedState:
        return Hardware::BleTypes::BleDeviceStatus::Armed;
    case StateMachine::StateId::RecordingState:
        return Hardware::BleTypes::BleDeviceStatus::Recording;
    case StateMachine::StateId::ProcessingState:
        return Hardware::BleTypes::BleDeviceStatus::Processing;
    case StateMachine::StateId::SendingState:
        return Hardware::BleTypes::BleDeviceStatus::ResultReady;
    }

    return Hardware::BleTypes::BleDeviceStatus::Error;
}
} // namespace

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
    m_LastPublishedState = StateMachine::StateId::IdleState;
    m_rBleServiceManager.SetStatus(Hardware::BleTypes::BleDeviceStatus::Idle);
}

void W8BandServiceManager::StartApplication() {}

void W8BandServiceManager::HandleBleCommand(
    const Hardware::BleTypes::BleCommand &rCommand)
{
    switch(rCommand.command)
    {
    case Hardware::BleTypes::BleCommandType::Calibrate:
        if(m_Fsm.GetCurrentStateId() == StateMachine::StateId::IdleState)
        {
            m_Fsm.RequestTransition(StateMachine::StateId::CalibrationState);
            m_rBleServiceManager.SendCommandResponse(
                rCommand.command, Hardware::BleTypes::CommandResult::Ok);
        } else
        {
            m_rBleServiceManager.SendCommandResponse(
                rCommand.command,
                Hardware::BleTypes::CommandResult::InvalidState);
        }
        break;
    default:
        m_rBleServiceManager.SendCommandResponse(
            rCommand.command, Hardware::BleTypes::CommandResult::InvalidCommand);
        break;
    }
}

void W8BandServiceManager::PublishFsmStatus()
{
    const StateMachine::StateId currentState = m_Fsm.GetCurrentStateId();
    if(currentState == m_LastPublishedState)
    {
        return;
    }

    m_LastPublishedState = currentState;
    m_rBleServiceManager.SetStatus(ToBleStatus(currentState));
}

void W8BandServiceManager::HandleBleSendResultData()
{
    if(!m_rDataContext.m_ResultReady)
    {
        return;
    }
    if(m_rBleServiceManager.QueueResult(m_rDataContext.m_ProcessingResult))
    {
        m_rDataContext.m_ResultReady = false;
    }
}

void W8BandServiceManager::Update()
{
    HandleBleSendResultData();

    Hardware::BleTypes::BleCommand command;

    while(m_rBleServiceManager.TryPopCommand(command))
    {
        HandleBleCommand(command);
    }

    if(v_WakeUpDetected)
    {
        m_rDataContext.m_LiftOffDetected = true;
        v_WakeUpDetected = false;
    }
    m_Fsm.Update();
    PublishFsmStatus();

    m_rBleServiceManager.SendBLEData();
}

void W8BandServiceManager::AttachWakeUptInterrupt(uint16_t interruptPin)
{ attachInterrupt(digitalPinToInterrupt(interruptPin), WakeUpISR1, RISING); }

void W8BandServiceManager::WakeUpISR1()
{
    v_WakeUpDetected = true;
    ISRcount += 1;
}
} // namespace w8band
