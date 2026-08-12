#include "BufferringState.hpp"

#define MOTION_START_TIMEOUT_MS 5000

namespace w8band::StateMachine
{

namespace
{
constexpr float ACCEL_VAR_THS = 13.0f;
constexpr float GYRO_VAR_THS = 50.0f;

constexpr float HOLD_STILL_ACCEL_VAR_THS = 100.0f;
constexpr float MOTION_START_ACCEL_VAR_THS = 450.0f;

constexpr uint32_t HOLD_STILL_DWELL_MS = 100;

constexpr uint32_t MOTION_START_DWELL_MS = 80;
}

BufferringState::BufferringState(DataContext &rDataCtx, W8BandFsm &rFsm)
    : IState<DataContext, StateId>(rDataCtx), m_rFsm(rFsm)
{}

/// @brief @see IState::OnEnter()
void BufferringState::OnEnter()
{
    m_Window.Reset();
    m_rContext.m_PreIndex = 0;
    m_rContext.m_PreCount = 0;
    m_rContext.m_LiftOffDetected = false;
    m_rContext.m_MotionStartDetected = false;

    m_InMotionStartDwell = false;
    m_InMotionTimeMs = 0;
    m_InStillnessStartDwell = false;
    m_InStillnessTimeMs = 0;
    m_Phase = BufferringPhase::WaitLiftOff;
}

/// @brief @see IState::OnExit()
void BufferringState::OnExit() {}

/// @brief @see IState::Update()
void BufferringState::Update()
{
    data::SamplePacket packet;
    if(!m_rContext.m_rLsmServiceManager.ObtainData(packet))
    {
        return;
    }
    m_rContext.PushToPreBuffer(packet);

    switch(m_Phase)
    {
    case BufferringPhase::WaitLiftOff: HandleWaitLiftOff(); break;
    case BufferringPhase::WaitStillness: HandleWaitStillness(packet); break;
    case BufferringPhase::WaitMotion: HandleWaitMotion(packet); break;
    default: break;
    }
}

void BufferringState::HandleWaitLiftOff()
{
    if(!m_rContext.m_LiftOffDetected)
    {
        return;
    }
    Serial.println("LiftOff detected.");
    m_rContext.m_LiftOffDetected = false;
    m_Phase = BufferringPhase::WaitStillness;
    m_LiftOffBeganMs = millis();
}

void BufferringState::HandleWaitStillness(data::SamplePacket &rPacket)
{
    float gyroMag;
    float accelMag;

    DataContext::CalculateMagnitude(rPacket, accelMag, gyroMag);
    m_Window.Push(accelMag, gyroMag);
    bool stillnessDetected
        = m_Window.isStill(HOLD_STILL_ACCEL_VAR_THS, GYRO_VAR_THS);
    Serial.println("Accel variance: ");
    Serial.print(m_Window.GetVarAccel());

    // Wykrycie ciszy -> dwell start. Nastepna iteracja -> Dalej cisza -> dwell
    // start -> update time Nastepna iteracja -> RUCH -> dwell = false -> Nie
    // iterujemy dalej
    if(!stillnessDetected)
    {
        m_InStillnessStartDwell = false;
        return;
    }

    if(!m_InStillnessStartDwell)
    {
        m_InStillnessStartDwell = true;
        m_InStillnessTimeMs = millis();
        return;
    }

    if(millis() - m_InStillnessTimeMs > HOLD_STILL_DWELL_MS)
    {
        Serial.println("Barbell is still. Transition to WAIT MOTION.");
        m_Phase = BufferringPhase::WaitMotion;
        return;
    }
}

void BufferringState::HandleWaitMotion(data::SamplePacket &rPacket)
{
    float gyroMag;
    float accelMag;
    DataContext::CalculateMagnitude(rPacket, accelMag, gyroMag);
    m_Window.Push(accelMag, gyroMag);
    Serial.println("Accel variance inside Wait Motion: ");
    Serial.print(m_Window.GetVarAccel());

    bool stillnessDetected
        = m_Window.isStill(MOTION_START_ACCEL_VAR_THS, GYRO_VAR_THS);

    if(stillnessDetected)
    {
        m_InMotionStartDwell = false;
        return;
    }

    if(!m_InMotionStartDwell)
    {
        m_InMotionStartDwell = true;
        m_InMotionTimeMs = millis();
        return;
    }

    if(millis() - m_InMotionTimeMs > MOTION_START_DWELL_MS)
    {
        Serial.println("Barbell started moving. Starting recording.");
        m_rFsm.RequestTransition(StateId::RecordingState);
    }
}

StateId BufferringState::GetStateId() const
{ return StateId::BufferringState; }
std::string BufferringState::GetStateName() const
{ return "Bufferring State"; }
} // namespace
