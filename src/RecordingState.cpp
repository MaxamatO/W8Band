#include "RecordingState.hpp"
#include "helpers/ProcessingHelpers.hpp"

namespace w8band::StateMachine
{
namespace
{
constexpr float MG_TO_MS2 = 9.80665f / 1000.0f; // G in MG = 0.00980665g
constexpr float kUpThreshold = 0.14f;
constexpr float kDownThreshold = -0.14f;
constexpr float kHysteresis = 0.02f;
}
RecordingState::RecordingState(DataContext &rDataCtx, W8BandFsm &rFsm)
    : IState<DataContext, StateId>(rDataCtx), m_rFsm(rFsm)
{}

void RecordingState::OnEnter()
{
    m_rContext.DropPreBufferToEndData();

    m_HavePrevSample = false;
    m_CurrVelocityZ = 0.0f;
    m_PrevWorldAz = 0.0f;
    m_PrevTimestampMs = 0;

    m_Direction = MotionDirection::Unknown;
    m_HasSeenGoingUp = false;
    m_IsNearZeroDwell = false;
    m_NearZeroTimeMs = 0;

    m_rContext.m_RecordingStartedMs = millis();
}
void RecordingState::OnExit() {}
void RecordingState::Update()
{
    data::SamplePacket packet;
    if(!m_rContext.m_rLsmServiceManager.ObtainData(packet))
    {
        return;
    }
    m_rContext.m_Data.push_back(packet);

    IntegrateVelocityRT(packet);
    UpdateDirection();
    CheckForEndOfMotion();
    if(millis() - m_rContext.m_RecordingStartedMs > RECORDING_TIMEOUT_MS)
    {
        Serial.println("Recording timeout reached.");
        m_rFsm.RequestTransition(StateId::ProcessingState);
    }
}

void RecordingState::IntegrateVelocityRT(data::SamplePacket &rPacket)
{
    float linAx;
    float linAy;
    float linAz;

    // Linear acceleration of IMU in device frame
    m_rContext.GetLinearAccel(rPacket, linAx, linAy, linAz);

    float worldAx_mg;
    float worldAy_mg;
    float worldAz_mg;

    float qw = rPacket.q[0] / Q14_SCALE;
    float qx = rPacket.q[1] / Q14_SCALE;
    float qy = rPacket.q[2] / Q14_SCALE;
    float qz = rPacket.q[3] / Q14_SCALE;

    helpers::Processing::RotateBodyToWorld(qw, qx, qy, qz, linAx, linAy, linAz,
                                           worldAx_mg, worldAy_mg, worldAz_mg);

    if(!m_HavePrevSample)
    {
        m_PrevWorldAz = worldAz_mg * MG_TO_MS2;
        m_PrevTimestampMs = rPacket.timestamp_ms;
        m_HavePrevSample = true;
        return;
    }

    // TODO: Przeanalizowac blad nieortogonalnosc osi, przyrost predkosci na lezacym
    // urzadzeniu wahal sie od 0.01/s, do 1.00/s w zaleznosci od obrotu wzgledem OZ.

    // Integrate in order to obtain current velocity.
    float dtSeconds = (rPacket.timestamp_ms - m_PrevTimestampMs) / 1000.0f;
    float worldAz_ms2 = worldAz_mg * MG_TO_MS2;
    m_CurrVelocityZ += (m_PrevWorldAz + worldAz_ms2) * 0.5f * dtSeconds;
    m_PrevWorldAz = worldAz_ms2;
    m_PrevTimestampMs = rPacket.timestamp_ms;
}

void RecordingState::CheckForEndOfMotion()
{
    if(!m_HasSeenGoingUp)
    {
        m_IsNearZeroDwell = false;
        return;
    }

    bool nearZero = (m_Direction == MotionDirection::NearZero);

    if(!nearZero)
    {
        m_IsNearZeroDwell = false;
        return;
    }

    // If nearZero is true, and device has been moving up, we skip these ifs
    // above, and get into this one with pre-set near zero dwell, and begin
    // checking time of near zero in order to confidently state that the device
    // has stopped moving, disregarding noise
    if(!m_IsNearZeroDwell)
    {
        m_IsNearZeroDwell = true;
        m_NearZeroTimeMs = millis();
        return;
    }

    Serial.println("Velocity: ");
    Serial.print(m_CurrVelocityZ);

    if(millis() - m_NearZeroTimeMs > END_OF_MOTION_DWELL_MS)
    {
        Serial.println("End of motion detected.");
        m_rFsm.RequestTransition(StateId::ProcessingState);
    }
}

void RecordingState::UpdateDirection()
{
    switch(m_Direction)
    {
    case MotionDirection::GoingUp:
        // Serial.println("up.");
        if(m_CurrVelocityZ < kUpThreshold - kHysteresis)
        {
            m_Direction = MotionDirection::NearZero;
        }
        break;
    case MotionDirection::GoingDown:
        Serial.println("down.");
        if(m_CurrVelocityZ > kDownThreshold + kHysteresis)
        {
            m_Direction = MotionDirection::NearZero;
        }
        break;
    case MotionDirection::NearZero: Serial.println("zero");
    case MotionDirection::Unknown:
        if(m_CurrVelocityZ > kUpThreshold)
        {
            m_Direction = MotionDirection::GoingUp;
        } else if(m_CurrVelocityZ < kDownThreshold)
        {
            m_Direction = MotionDirection::GoingDown;
        }
        break;
    default: break;
    }
    if(m_Direction == MotionDirection::GoingUp)
    {
        m_HasSeenGoingUp = true;
    }
}

StateId RecordingState::GetStateId() const { return StateId::RecordingState; }
std::string RecordingState::GetStateName() const { return "Recording State"; }
} // namespace w8band::StateMachine
