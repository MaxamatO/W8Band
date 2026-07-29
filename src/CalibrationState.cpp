#include "CalibrationState.hpp"

#define SAMPLES_TO_DISCARD 30
#define CALIBRATION_TIMEOUT_MS 6000

namespace w8band::StateMachine
{
namespace
{
constexpr float ACCEL_VAR_THS = 15.0f;
constexpr float GYRO_VAR_THS = 15.0f;
void RotateWorldToBody(float qw, float qx, float qy, float qz, float vx,
                       float vy, float vz, float &rOutX, float &rOutY,
                       float &rOutZ)
{
    qx = -qx;
    qy = -qy;
    qz = -qz;

    float cx = qy * vz - qz * vy;
    float cy = qz * vx - qx * vz;
    float cz = qx * vy - qy * vx;

    float ccx = qy * cz - qz * cy;
    float ccy = qz * cx - qx * cz;
    float ccz = qx * cy - qy * cx;

    rOutX = vx + 2.0f * qw * cx + 2.0f * ccx;
    rOutY = vy + 2.0f * qw * cy + 2.0f * ccy;
    rOutZ = vz + 2.0f * qw * cz + 2.0f * ccz;
}
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

static void NormaliseQuaternion() {}

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
    // if(millis() - m_StartedAtMs > CALIBRATION_TIMEOUT_MS
    //    && m_Phase != CalibrationPhase::Done)
    // {
    //     m_Phase = CalibrationPhase::Failed;
    // }

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
    case CalibrationPhase::Done: CalibrationDoneHandler(); break;
    case CalibrationPhase::Failed: CalibratingErrorHandler(); break;
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
    float qw = m_BiasAccumulator.GetMeanQw() / data::Q14_SCALE;
    float qx = m_BiasAccumulator.GetMeanQx() / data::Q14_SCALE;
    float qy = m_BiasAccumulator.GetMeanQy() / data::Q14_SCALE;
    float qz = m_BiasAccumulator.GetMeanQz() / data::Q14_SCALE;

    float norm = std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
    qw /= norm;
    qx /= norm;
    qy /= norm;
    qz /= norm;

    float expectedGx;
    float expectedGy;
    float expectedGz;

    RotateWorldToBody(qw, qx, qy, qz, 0.0f, 0.0f, data::GRAVITY_MG, expectedGx,
                      expectedGy, expectedGz);

    float meanAx = m_BiasAccumulator.GetMeanAx();
    float meanAy = m_BiasAccumulator.GetMeanAy();
    float meanAz = m_BiasAccumulator.GetMeanAz();

    m_rContext.m_AccelBias.axBias = static_cast<int32_t>(meanAx - expectedGx);
    m_rContext.m_AccelBias.ayBias = static_cast<int32_t>(meanAy - expectedGy);
    m_rContext.m_AccelBias.azBias = static_cast<int32_t>(meanAz - expectedGz);
    m_rContext.m_CalibrationValid = true;
    m_rFsm.RequestTransition(StateId::BufferringState);
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

    m_BiasAccumulator.Add(rPacket);

    if(m_BiasAccumulator.IsFull(CALIBRATION_DATA_COUNT))
    {
        m_Phase = CalibrationPhase::Done;
        return;
    }
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
