#include "RecordingState.hpp"
#include <cmath>

namespace w8band::StateMachine
{
namespace
{
constexpr float MG_TO_MS2 = 9.80665f / 1000.0f;

// Phase gates. They deliberately do not attempt to mark the exact turnaround.
constexpr float ECCENTRIC_VELOCITY_MPS = -0.10f;
constexpr float CONCENTRIC_VELOCITY_MPS = 0.05f;
constexpr uint32_t DIRECTION_DWELL_MS = 80;

// End-of-repetition gates, calibrated against dataset.csv and dataset2.csv.
constexpr float MIN_PEAK_CONCENTRIC_VELOCITY_MPS = 0.20f;
constexpr float TERMINAL_DECELERATION_MPS = 0.10f;
constexpr float END_VELOCITY_MPS = 0.12f;
constexpr float END_ACCEL_VARIANCE_MG2 = 250.0f;
constexpr float END_VERTICAL_ACCEL_RMS_MS2 = 0.30f;

// Wider exit thresholds prevent a candidate from flickering at the boundary.
constexpr float END_EXIT_VELOCITY_MPS = 0.20f;
constexpr float END_EXIT_ACCEL_VARIANCE_MG2 = 450.0f;
constexpr float END_EXIT_VERTICAL_ACCEL_RMS_MS2 = 0.50f;

constexpr uint32_t END_CANDIDATE_DWELL_MS = 120;
constexpr uint32_t POST_STOP_CAPTURE_MS = 300;
constexpr uint32_t RECORDING_TIMEOUT_MS = 8000;
constexpr std::size_t MAX_RECORDING_SAMPLES = PRE_RECORD_SAMPLES + 120U * 8U;
constexpr float MAX_INTEGRATION_DT_SECONDS = 0.10f;

bool HasSensorTimeElapsed(uint32_t startedTicks, uint32_t nowTicks,
                          uint32_t durationMs)
{
    const uint32_t elapsedTicks = nowTicks - startedTicks;
    const float elapsedMs
        = elapsedTicks * LSM_TIMESTAMP_TICK_SECONDS * 1000.0f;
    return elapsedMs >= static_cast<float>(durationMs);
}

const char *PhaseName(RecordingPhase phase)
{
    switch(phase)
    {
    case RecordingPhase::AwaitingEccentric: return "awaiting eccentric";
    case RecordingPhase::Eccentric: return "eccentric";
    case RecordingPhase::Concentric: return "concentric";
    case RecordingPhase::StopCandidate: return "stop candidate";
    case RecordingPhase::PostStopCapture: return "post-stop capture";
    default: return "unknown";
    }
}
} // namespace

RecordingState::RecordingState(DataContext &rDataCtx, W8BandFsm &rFsm)
    : IState<DataContext, StateId>(rDataCtx), m_rFsm(rFsm)
{}

void RecordingState::OnEnter()
{
    m_rContext.DropPreBufferToEndData();

    m_EndDetector.Reset();
    m_VerticalAccelFilter.Reset();
    m_PreviousQuat = {1.0f, 0.0f, 0.0f, 0.0f};
    m_Phase = RecordingPhase::AwaitingEccentric;
    m_HavePrevSample = false;
    m_PrevFilteredWorldAz = 0.0f;
    m_PrevTimestampTicks = 0;
    m_CurrVelocityZ = 0.0f;
    m_PeakVelocityZ = 0.0f;
    m_TerminalDecelerationSeen = false;
    m_DirectionalDwellActive = false;
    m_DirectionalDwellStartedTicks = 0;
    m_StopCandidateStartedTicks = 0;
    m_PostStopStartedTicks = 0;

    // Feed the rolling pre-buffer through the same causal path. This gives
    // both the filter and the online integrator a real stationary baseline.
    for(const data::SamplePacket &packet : m_rContext.m_Data)
    {
        ProcessSample(packet);
    }

    m_rContext.m_RecordingStartedMs = millis();
}

void RecordingState::OnExit() {}

void RecordingState::Update()
{
    if(millis() - m_rContext.m_RecordingStartedMs >= RECORDING_TIMEOUT_MS)
    {
        Serial.println("Recording timeout; processing captured data.");
        m_rFsm.RequestTransition(StateId::ProcessingState);
        return;
    }

    if(m_rContext.m_Data.size() >= MAX_RECORDING_SAMPLES)
    {
        Serial.println("Recording buffer limit reached; processing data.");
        m_rFsm.RequestTransition(StateId::ProcessingState);
        return;
    }

    data::SamplePacket packet;
    if(!m_rContext.m_rLsmServiceManager.ObtainData(packet))
    {
        return;
    }

    m_rContext.m_Data.push_back(packet);
    ProcessSample(packet);
}

void RecordingState::ProcessSample(const data::SamplePacket &rPacket)
{
    float linAxMg;
    float linAyMg;
    float linAzMg;
    m_rContext.GetLinearAccel(rPacket, linAxMg, linAyMg, linAzMg);

    const helpers::Processing::Quat q
        = helpers::Processing::DecodeQuaternion(rPacket, m_PreviousQuat);
    float worldAxMg;
    float worldAyMg;
    float worldAzMg;
    helpers::Processing::RotateBodyToWorld(q.w, q.x, q.y, q.z, linAxMg,
                                           linAyMg, linAzMg, worldAxMg,
                                           worldAyMg, worldAzMg);

    const float filteredWorldAz
        = m_VerticalAccelFilter.Process(worldAzMg * MG_TO_MS2);
    m_EndDetector.Push(DataContext::CalculateAccelMagnitude(rPacket),
                       filteredWorldAz);

    if(!m_HavePrevSample)
    {
        m_PrevFilteredWorldAz = filteredWorldAz;
        m_PrevTimestampTicks = rPacket.timestamp_ticks;
        m_HavePrevSample = true;
        UpdatePhase(rPacket.timestamp_ticks);
        return;
    }

    const uint32_t deltaTicks
        = rPacket.timestamp_ticks - m_PrevTimestampTicks;
    const float dtSeconds
        = static_cast<float>(deltaTicks) * LSM_TIMESTAMP_TICK_SECONDS;

    if(dtSeconds > 0.0f && dtSeconds <= MAX_INTEGRATION_DT_SECONDS)
    {
        m_CurrVelocityZ
            += (m_PrevFilteredWorldAz + filteredWorldAz) * 0.5f * dtSeconds;
    }

    m_PrevFilteredWorldAz = filteredWorldAz;
    m_PrevTimestampTicks = rPacket.timestamp_ticks;
    UpdatePhase(rPacket.timestamp_ticks);
}

void RecordingState::UpdatePhase(uint32_t timestampTicks)
{
    switch(m_Phase)
    {
    case RecordingPhase::AwaitingEccentric:
        if(UpdateDirectionalDwell(m_CurrVelocityZ <= ECCENTRIC_VELOCITY_MPS,
                                  timestampTicks, DIRECTION_DWELL_MS))
        {
            SetPhase(RecordingPhase::Eccentric, timestampTicks);
        }
        break;

    case RecordingPhase::Eccentric:
        if(UpdateDirectionalDwell(m_CurrVelocityZ >= CONCENTRIC_VELOCITY_MPS,
                                  timestampTicks, DIRECTION_DWELL_MS))
        {
            m_PeakVelocityZ = m_CurrVelocityZ;
            SetPhase(RecordingPhase::Concentric, timestampTicks);
        }
        break;

    case RecordingPhase::Concentric:
        if(m_CurrVelocityZ > m_PeakVelocityZ)
        {
            m_PeakVelocityZ = m_CurrVelocityZ;
        }
        if(m_PeakVelocityZ >= MIN_PEAK_CONCENTRIC_VELOCITY_MPS
           && m_CurrVelocityZ
                  <= m_PeakVelocityZ - TERMINAL_DECELERATION_MPS)
        {
            m_TerminalDecelerationSeen = true;
        }
        if(m_TerminalDecelerationSeen && IsStrictEndCandidate())
        {
            SetPhase(RecordingPhase::StopCandidate, timestampTicks);
        }
        break;

    case RecordingPhase::StopCandidate:
        if(!IsRelaxedEndCandidate())
        {
            SetPhase(RecordingPhase::Concentric, timestampTicks);
        } else if(HasSensorTimeElapsed(m_StopCandidateStartedTicks,
                                       timestampTicks,
                                       END_CANDIDATE_DWELL_MS))
        {
            SetPhase(RecordingPhase::PostStopCapture, timestampTicks);
        }
        break;

    case RecordingPhase::PostStopCapture:
        if(HasSensorTimeElapsed(m_PostStopStartedTicks, timestampTicks,
                                POST_STOP_CAPTURE_MS))
        {
            m_rFsm.RequestTransition(StateId::ProcessingState);
        }
        break;
    }
}

void RecordingState::SetPhase(RecordingPhase phase, uint32_t timestampTicks)
{
    m_Phase = phase;
    m_DirectionalDwellActive = false;
    if(phase == RecordingPhase::StopCandidate)
    {
        m_StopCandidateStartedTicks = timestampTicks;
    } else if(phase == RecordingPhase::PostStopCapture)
    {
        m_PostStopStartedTicks = timestampTicks;
    }
    Serial.printf("Recording phase: %s, v_z=%.3f m/s\n", PhaseName(phase),
                  m_CurrVelocityZ);
}

bool RecordingState::UpdateDirectionalDwell(bool condition,
                                             uint32_t timestampTicks,
                                             uint32_t dwellMs)
{
    if(!condition)
    {
        m_DirectionalDwellActive = false;
        return false;
    }
    if(!m_DirectionalDwellActive)
    {
        m_DirectionalDwellActive = true;
        m_DirectionalDwellStartedTicks = timestampTicks;
        return false;
    }
    return HasSensorTimeElapsed(m_DirectionalDwellStartedTicks, timestampTicks,
                                dwellMs);
}

bool RecordingState::IsStrictEndCandidate() const
{
    return std::fabs(m_CurrVelocityZ) <= END_VELOCITY_MPS
           && m_EndDetector.IsStill(END_ACCEL_VARIANCE_MG2,
                                    END_VERTICAL_ACCEL_RMS_MS2);
}

bool RecordingState::IsRelaxedEndCandidate() const
{
    return std::fabs(m_CurrVelocityZ) <= END_EXIT_VELOCITY_MPS
           && m_EndDetector.IsStill(END_EXIT_ACCEL_VARIANCE_MG2,
                                    END_EXIT_VERTICAL_ACCEL_RMS_MS2);
}

StateId RecordingState::GetStateId() const { return StateId::RecordingState; }

std::string RecordingState::GetStateName() const { return "Recording State"; }
} // namespace w8band::StateMachine
