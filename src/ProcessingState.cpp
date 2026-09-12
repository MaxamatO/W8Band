#include "ProcessingState.hpp"
#include "motion-processor/MotionProcessor.hpp"

#ifndef W8BAND_DUMP_RAW_CSV
#define W8BAND_DUMP_RAW_CSV 0
#endif

namespace w8band::StateMachine
{
namespace
{
#if W8BAND_DUMP_RAW_CSV
void PrintRawCapture(const DataContext &rContext)
{
    Serial.println(
        "timestamp_ticks,qw,qx,qy,qz,ax,ay,az,gvx,gvy,gvz,biasx,biasy,biasz");
    for(const data::SamplePacket &rPacket : rContext.m_Data)
    {
        Serial.printf("%lu,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%ld,%ld,%ld\n",
                      static_cast<unsigned long>(rPacket.timestamp_ticks),
                      rPacket.q[0], rPacket.q[1], rPacket.q[2], rPacket.q[3],
                      rPacket.a[0], rPacket.a[1], rPacket.a[2], rPacket.gv[0],
                      rPacket.gv[1], rPacket.gv[2],
                      static_cast<long>(rContext.m_AccelBias.axBias),
                      static_cast<long>(rContext.m_AccelBias.ayBias),
                      static_cast<long>(rContext.m_AccelBias.azBias));
    }
}
#endif

void PrintProcessingResult(const Motion::ProcessingResult &rResult)
{
    Serial.printf("Motion processing: %s\n",
                  Motion::MotionProcessor::StatusToString(rResult.status));
    if(!rResult.valid)
    {
        Serial.printf("Raw samples: %u\n", rResult.rawSampleCount);
        return;
    }

    Serial.printf("Samples: %u, duration: %lu ms, turnaround: %u\n",
                  rResult.rawSampleCount,
                  static_cast<unsigned long>(rResult.durationMs),
                  rResult.turnaroundIndex);
    Serial.printf("ROM: %d mm, vmax: %d mm/s, vmean: %d mm/s\n",
                  rResult.verticalRomMm, rResult.maxVelocityMmPerSec,
                  rResult.meanConcentricVelocityMmPerSec);
    Serial.printf("Eccentric: %lu ms, concentric: %lu ms, gravity std: %u mg\n",
                  static_cast<unsigned long>(rResult.eccentricDurationMs),
                  static_cast<unsigned long>(rResult.concentricDurationMs),
                  rResult.maxWorldGravityStdMg);
    if(rResult.powerValid)
    {
        Serial.printf("Power max: %ld mW, mean: %ld mW\n",
                      static_cast<long>(rResult.maxPowerMilliwatts),
                      static_cast<long>(rResult.meanConcentricPowerMilliwatts));
    } else
    {
        Serial.println("Power unavailable: barbell mass is not configured.");
    }

    Serial.println("trajectory_index,horizontal_mm,vertical_mm,turnaround");
    for(uint16_t i = 0; i < rResult.trajectoryPointCount; ++i)
    {
        const Motion::TrajectoryPoint &rPoint = rResult.trajectory[i];
        Serial.printf("%u,%d,%d,%u\n", i, rPoint.horizontalMm,
                      rPoint.verticalMm,
                      i == rResult.turnaroundTrajectoryIndex ? 1U : 0U);
    }
}
} // namespace

ProcessingState::ProcessingState(DataContext &rDataCtx, W8BandFsm &rFsm)
    : fsm::IState<DataContext, StateId>(rDataCtx), m_rFsm(rFsm)
{}

void ProcessingState::OnEnter()
{
    m_rContext.m_ProcessingResult = Motion::MotionProcessor::Process(
        m_rContext.m_Data, m_rContext.m_AccelBias,
        m_rContext.m_ProcessingConfig);
    PrintProcessingResult(m_rContext.m_ProcessingResult);

#if W8BAND_DUMP_RAW_CSV
    PrintRawCapture(m_rContext);
#endif

    // SendingState is not registered yet. Keep the device usable for the next
    // repetition; once BLE transport exists, this target becomes SendingState.
    m_rFsm.RequestTransition(StateId::BufferringState);
}

void ProcessingState::OnExit() {}

void ProcessingState::Update() {}

StateId ProcessingState::GetStateId() const
{ return StateId::ProcessingState; }

std::string ProcessingState::GetStateName() const { return "ProcessingState"; }
} // namespace w8band::StateMachine
