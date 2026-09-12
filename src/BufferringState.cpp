#include "BufferringState.hpp"

namespace w8band::StateMachine
{

namespace
{
constexpr float HOLD_STILL_ACCEL_VAR_THS = 100.0f;
constexpr float MOTION_START_ACCEL_VAR_THS = 450.0f;

constexpr uint32_t HOLD_STILL_DWELL_MS = 100;

constexpr uint32_t MOTION_START_DWELL_MS = 80;

bool HasSensorTimeElapsed(uint32_t startedTicks, uint32_t nowTicks,
                          uint32_t durationMs)
{
    const uint32_t elapsedTicks = nowTicks - startedTicks;
    const float elapsedMs
        = elapsedTicks * LSM_TIMESTAMP_TICK_SECONDS * 1000.0f;
    return elapsedMs >= static_cast<float>(durationMs);
}
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
    m_rContext.m_Data.clear();
    m_rContext.m_Data.reserve(PRE_RECORD_SAMPLES + 120U * 8U);
    m_rContext.m_LiftOffDetected = false;
    m_rContext.m_MotionStartDetected = false;

    m_InMotionStartDwell = false;
    m_MotionStartedTicks = 0;
    m_InStillnessStartDwell = false;
    m_StillnessStartedTicks = 0;
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
}

void BufferringState::HandleWaitStillness(data::SamplePacket &rPacket)
{
    const float accelMag = DataContext::CalculateAccelMagnitude(rPacket);
    m_Window.Push(accelMag);
    bool stillnessDetected
        = m_Window.IsStill(HOLD_STILL_ACCEL_VAR_THS);

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
        m_StillnessStartedTicks = rPacket.timestamp_ticks;
        return;
    }

    if(HasSensorTimeElapsed(m_StillnessStartedTicks, rPacket.timestamp_ticks,
                            HOLD_STILL_DWELL_MS))
    {
        Serial.println("Barbell is still. Transition to WAIT MOTION.");
        m_Phase = BufferringPhase::WaitMotion;
        m_InMotionStartDwell = false;
        return;
    }
}

void BufferringState::HandleWaitMotion(data::SamplePacket &rPacket)
{
    const float accelMag = DataContext::CalculateAccelMagnitude(rPacket);
    m_Window.Push(accelMag);

    bool stillnessDetected
        = m_Window.IsStill(MOTION_START_ACCEL_VAR_THS);

    if(stillnessDetected)
    {
        m_InMotionStartDwell = false;
        return;
    }

    if(!m_InMotionStartDwell)
    {
        m_InMotionStartDwell = true;
        m_MotionStartedTicks = rPacket.timestamp_ticks;
        return;
    }

    if(HasSensorTimeElapsed(m_MotionStartedTicks, rPacket.timestamp_ticks,
                            MOTION_START_DWELL_MS))
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
