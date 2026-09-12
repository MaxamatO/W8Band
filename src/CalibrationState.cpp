#include "CalibrationState.hpp"

#define SAMPLES_TO_DISCARD 30
#define CALIBRATION_TIMEOUT_MS 6000

namespace w8band::StateMachine
{
namespace
{
// Threshold for determining stillness.
constexpr float STILLNESS_ACCEL_VAR_THS = 120.0f;
}
CalibrationState::CalibrationState(DataContext &rDataCtx, W8BandFsm &rFsm)
    : fsm::IState<DataContext, StateId>(rDataCtx), m_rFsm(rFsm)
{}

void CalibrationState::OnEnter()
{
    m_rContext.m_rLsmServiceManager.ResetFIFO();
    m_Window.Reset();
    m_BiasAccumulator.Reset();
    m_rContext.m_CalibrationValid = false;
    m_DiscardCount = SAMPLES_TO_DISCARD;
    m_Phase = CalibrationPhase::WaitingForStillness;
    m_StartedAtMs = millis();
}

void CalibrationState::OnExit() {}

void CalibrationState::Update()
{
    if(millis() - m_StartedAtMs > CALIBRATION_TIMEOUT_MS
       && m_Phase != CalibrationPhase::Done)
    {
        m_Phase = CalibrationPhase::Failed;
    }

    data::SamplePacket packet;
    if(!m_rContext.m_rLsmServiceManager.ObtainData(packet))
    {
        return;
    }

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
    case CalibrationPhase::Done: CalibrationDoneHandler(); break;
    case CalibrationPhase::Failed: CalibratingErrorHandler(); break;
    default: break;
    }
}

void CalibrationState::AccumulatingHandler(data::SamplePacket &rPacket)
{
    const float accelMag = DataContext::CalculateAccelMagnitude(rPacket);
    m_Window.Push(accelMag);

    if(!m_Window.IsStill(STILLNESS_ACCEL_VAR_THS))
    {
        Serial.println("Not Still");
        m_BiasAccumulator.Reset();
        m_Phase = CalibrationPhase::WaitingForStillness;
        return;
    }
    m_BiasAccumulator.Add(rPacket);

    if(m_BiasAccumulator.IsFull(CALIBRATION_DATA_COUNT))
    {
        m_Phase = CalibrationPhase::Done;
    }
}

void CalibrationState::CalibrationDoneHandler()
{
    m_rContext.m_AccelBias.axBias = static_cast<int32_t>(
        m_BiasAccumulator.GetMeanAx() - m_BiasAccumulator.GetMeanGx());
    m_rContext.m_AccelBias.ayBias = static_cast<int32_t>(
        m_BiasAccumulator.GetMeanAy() - m_BiasAccumulator.GetMeanGy());
    m_rContext.m_AccelBias.azBias = static_cast<int32_t>(
        m_BiasAccumulator.GetMeanAz() - m_BiasAccumulator.GetMeanGz());
    m_rContext.m_CalibrationValid = true;
    m_rFsm.RequestTransition(StateId::BufferringState);
}

void CalibrationState::StillnessWaitHandler(data::SamplePacket &rPacket)
{
    const float accelMag = DataContext::CalculateAccelMagnitude(rPacket);
    m_Window.Push(accelMag);
    if(!m_Window.IsStill(STILLNESS_ACCEL_VAR_THS))
    {
        return;
    }

    m_BiasAccumulator.Reset();
    m_BiasAccumulator.Add(rPacket);
    m_Phase = CalibrationPhase::Accumulating;
}

void CalibrationState::CalibratingErrorHandler()
{
    Serial.println("Entered error state");
    m_rContext.m_CalibrationValid = false;
}

StateId CalibrationState::GetStateId() const
{ return StateId::CalibrationState; }
std::string CalibrationState::GetStateName() const
{ return "CalibrationState"; }
} // namespace w8band::StateMachine
