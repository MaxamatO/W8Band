#include "CalibrationState.hpp"

#define SAMPLES_TO_DISCARD 30

namespace w8band::StateMachine
{
namespace
{
constexpr float ACCEL_VAR_THS = 10.0f;
constexpr float GYRO_VAR_THS = 10.0f;
}
CalibrationState::CalibrationState(DataContext &rDataCtx, W8BandFsm &rFsm)
    : fsm::IState<DataContext, StateId>(rDataCtx), m_rFsm(rFsm)
{}

static void CalculateMagnitude(data::SamplePacket &rPacket, float &rAccelMag,
                               float &rGyroMag)
{
    float ax = rPacket.a[0];
    float ay = rPacket.a[1];
    float az = rPacket.a[2];

    float qx = rPacket.q[1];
    float qy = rPacket.q[2];
    float qz = rPacket.q[3];

    rAccelMag = std::sqrt(ax * ax + ay * ay + az * az);
    rGyroMag = std::sqrt(qx * qx + qy * qy + qz * qz);
}

void CalibrationState::OnEnter()
{
    m_rContext.m_rLsmServiceManager.ResetFIFO();
    m_DiscardCount = SAMPLES_TO_DISCARD;
    m_Phase = CalibrationPhase::WaitingForStillness;
    m_StartedAtMs = millis();
}

void CalibrationState::OnExit() {}

void CalibrationState::Update()
{
    data::SamplePacket packet;
    m_rContext.m_rLsmServiceManager.ObtainData(packet);

    if(m_DiscardCount > 0)
    {
        m_DiscardCount--;
        return;
    }

    switch(m_Phase)
    {
    case CalibrationPhase::WaitingForStillness:
        StillnessWaitHandler(packet);
        break;
    case CalibrationPhase::Accumulating: AccumulatingHandler(packet); break;
    case CalibrationPhase::Done: break;
    case CalibrationPhase::Failed: break;
    default: break;
    }
}

void CalibrationState::AccumulatingHandler(data::SamplePacket &rPacket)
{
    float accelMag;
    float gyroMag;
    CalculateMagnitude(rPacket, accelMag, gyroMag);

    m_Window.Push(accelMag, gyroMag);

    if(!m_Window.isStill(ACCEL_VAR_THS, GYRO_VAR_THS))
    {
        Serial.println("Not Still");
        m_Phase = CalibrationPhase::WaitingForStillness;
        return;
    }
}

void CalibrationState::StillnessWaitHandler(data::SamplePacket &rPacket)
{
    float accelMag;
    float gyroMag;
    CalculateMagnitude(rPacket, accelMag, gyroMag);
    m_Window.Push(accelMag, gyroMag);
    if(m_Window.isStill(ACCEL_VAR_THS, GYRO_VAR_THS))
    {
        Serial.println("Still");
        m_Phase = CalibrationPhase::Accumulating;
        return;
    }
}

StateId CalibrationState::GetStateId() const
{ return StateId::CalibrationState; }
std::string CalibrationState::GetStateName() { return "CalibrationState"; }
} // namespace w8band::StateMachine
