#include "W8BandServiceManager.hpp"

namespace w8band
{

static W8BandServiceManager *instance = nullptr;

volatile bool v_WakeUpDetected = false;

void WakeUpISR1();

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
        StateMachine::StateId::ArmedState,
        std::make_unique<StateMachine::ArmedState>(m_rDataContext, m_Fsm));
    m_Fsm.Start(StateMachine::StateId::IdleState);
}

void W8BandServiceManager::StartApplication() {}

void W8BandServiceManager::Update() { m_Fsm.Update(); }

void W8BandServiceManager::RequestStateChange(StateMachine::StateId stateId)
{ m_Fsm.RequestTransition(stateId); }

// -------------------------------------------------------------------

// -------------------------------------------------------------------
// void W8Band::Update()
// {
//     switch(m_CurrentState)
//     {
//     case(StateMachine::BUFFERRING):
//         if(v_WakeUpDetected)
//         {
//             m_CurrentState = StateMachine::ARMED;
//             return;
//         }
//         if(m_Imu.FIFO_Get_Num_Samples(&fifo_samples) != LSM6DSV16X_OK)
//             return;
//         if(fifo_samples == 0)
//             return;
//         for(uint16_t i = 0; i < fifo_samples; i++)
//         {
//             if(m_Imu.FIFO_Get_Tag(&m_Tag) != LSM6DSV16X_OK)
//                 break;

//             if(m_Tag == TAG_GAME_ROTATION_VECTOR)
//             {
//                 // Library returns float[4]: i(X), j(Y), k(Z), r(W)
//                 float tmp[4];
//                 m_Imu.FIFO_Get_Rotation_Vector(tmp);
//                 m_LatestQuat[0] = tmp[3]; // Qw = r
//                 m_LatestQuat[1] = tmp[0]; // Qx = i
//                 m_LatestQuat[2] = tmp[1]; // Qy = j
//                 m_LatestQuat[3] = tmp[2]; // Qz = k
//                 m_HaveFreshQuat = true;
//             } else if(m_Tag == TAG_ACCELEROMETER)
//             {
//                 m_Imu.FIFO_Get_X_Axes(m_LatestAccel); // int32 [mg]
//                 m_HaveFreshAccel = true;
//             } else
//             {
//                 int32_t dummy[3];
//                 m_Imu.FIFO_Get_G_Axes(dummy);
//                 continue;
//             }

//             if(m_HaveFreshQuat && m_HaveFreshAccel)
//             {
//                 SamplePacket s;
//                 s.q[0] = toQ14(m_LatestQuat[0]);
//                 s.q[1] = toQ14(m_LatestQuat[1]);
//                 s.q[2] = toQ14(m_LatestQuat[2]);
//                 s.q[3] = toQ14(m_LatestQuat[3]);
//                 s.a[0] = (int16_t)m_LatestAccel[0];
//                 s.a[1] = (int16_t)m_LatestAccel[1];
//                 s.a[2] = (int16_t)m_LatestAccel[2];
//                 s.seq = m_Seq++;
//                 s.timestamp_ms = millis();
//                 m_HaveFreshQuat = false;
//                 m_HaveFreshAccel = false;

//                 m_PreBuffer[m_PreIndex] = s;
//                 m_PreIndex = (m_PreIndex + 1) % PRE_RECORD_SAMPLES;
//                 if(m_PreCount < PRE_RECORD_SAMPLES)
//                 {
//                     m_PreCount++;
//                 }
//             }
//         }
//         return;
//     case(StateMachine::ARMED):

//         Serial.print(
//             "Wakeup detected - saving pre buffer to m_Data. Saving: ");
//         Serial.println(m_PreCount);
//         for(int i = 0; i < m_PreCount; i++)
//         {
//             uint16_t idx = (m_PreIndex - m_PreCount + i + PRE_RECORD_SAMPLES)
//                            % PRE_RECORD_SAMPLES;
//             m_Data.push_back(m_PreBuffer[idx]);
//         }
//         m_CurrentState = StateMachine::RECORDING;
//         m_IsRecording = true;
//         m_RecordingStart = millis();
//         instance->m_Imu.FIFO_Set_Mode(LSM6DSV16X_BYPASS_MODE);
//         delay(10);
//         instance->m_Imu.FIFO_Set_Mode(LSM6DSV16X_STREAM_MODE);
//         delay(10);
//         return;

//     case(StateMachine::RECORDING):
//         if(millis() - m_RecordingStart >= RECORDING_TIME_MS)
//         {
//             m_IsRecording = false;
//             Serial.print("Recording done, samples: ");
//             Serial.println(m_Data.size());
//             SendBLEData();
//             return;
//         }
//         if(m_Imu.FIFO_Get_Num_Samples(&fifo_samples) != LSM6DSV16X_OK)
//             return;
//         if(fifo_samples == 0)
//             return;
//         for(uint16_t i = 0; i < fifo_samples; i++)
//         {
//             if(m_Imu.FIFO_Get_Tag(&m_Tag) != LSM6DSV16X_OK)
//                 break;

//             if(m_Tag == TAG_GAME_ROTATION_VECTOR)
//             {
//                 // Library returns float[4]: i(X), j(Y), k(Z), r(W)
//                 float tmp[4];
//                 m_Imu.FIFO_Get_Rotation_Vector(tmp);
//                 m_LatestQuat[0] = tmp[3]; // Qw = r
//                 m_LatestQuat[1] = tmp[0]; // Qx = i
//                 m_LatestQuat[2] = tmp[1]; // Qy = j
//                 m_LatestQuat[3] = tmp[2]; // Qz = k
//                 m_HaveFreshQuat = true;
//             } else if(m_Tag == TAG_ACCELEROMETER)
//             {
//                 m_Imu.FIFO_Get_X_Axes(m_LatestAccel); // int32 [mg]
//                 m_HaveFreshAccel = true;
//             } else
//             {
//                 int32_t dummy[3];
//                 m_Imu.FIFO_Get_G_Axes(dummy);
//                 continue;
//             }

//             if(m_HaveFreshQuat && m_HaveFreshAccel)
//             {
//                 SamplePacket s;
//                 s.q[0] = toQ14(m_LatestQuat[0]);
//                 s.q[1] = toQ14(m_LatestQuat[1]);
//                 s.q[2] = toQ14(m_LatestQuat[2]);
//                 s.q[3] = toQ14(m_LatestQuat[3]);
//                 s.a[0] = (int16_t)m_LatestAccel[0];
//                 s.a[1] = (int16_t)m_LatestAccel[1];
//                 s.a[2] = (int16_t)m_LatestAccel[2];
//                 s.seq = m_Seq++;
//                 s.timestamp_ms = millis();
//                 m_Data.push_back(s);
//                 m_HaveFreshQuat = false;
//                 m_HaveFreshAccel = false;
//             }
//         }
//     case(StateMachine::IDLE): break;
//     default: return;
//     }
// }

void WakeUpISR1() { v_WakeUpDetected = true; }
} // namespace w8band
