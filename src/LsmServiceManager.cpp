#include "LsmServiceManager.hpp"

#define IMU_FREQ 120.0f
#define WU_THS 10

#define TAG_GAME_ROTATION_VECTOR 0x13u
#define TAG_ACCELEROMETER 0x02u

#define ALGO_FREQ 120U

#define ALGO_PERIOD_MS (1000U / ALGO_FREQ)

namespace w8band::Hardware
{
namespace
{
unsigned long startTime, elapsedTime;
}

// Q1.14 scale: multiply float [-1,1] by 16384 and clamp to int16
static inline int16_t toQ14(float v)
{
    int32_t x = (int32_t)(v * 16384.0f);
    if(x > 32767)
        x = 32767;
    if(x < -32768)
        x = -32768;
    return (int16_t)x;
}

LsmServiceManager::LsmServiceManager(TwoWire &rWire, uint8_t i2cAddress)
    : m_Imu{&rWire, i2cAddress}
{}

bool LsmServiceManager::InitLsm()
{
    uint8_t status = 0;

    status |= m_Imu.begin();
    status |= m_Imu.Device_Reset();
    delay(50);

    status |= m_Imu.Enable_X();
    status |= m_Imu.Set_X_FS(4);
    status |= m_Imu.Set_X_ODR(IMU_FREQ);

    status |= m_Imu.Enable_G();
    status |= m_Imu.Set_G_FS(2000);
    status |= m_Imu.Set_G_ODR(IMU_FREQ);
    delay(50);

    status |= m_Imu.Set_SFLP_ODR(IMU_FREQ);
    status |= m_Imu.Enable_Rotation_Vector();
    delay(20);

    status |= m_Imu.FIFO_Set_Mode(LSM6DSV16X_BYPASS_MODE);
    delay(20);
    status |= m_Imu.Set_SFLP_Batch(true, false, false);
    status |= m_Imu.FIFO_Set_X_BDR(IMU_FREQ);
    status |= m_Imu.FIFO_Set_Mode(LSM6DSV16X_STREAM_MODE);
    delay(20);

    // InitWakeup(status);
    return (status == LSM6DSV16X_OK);
}

bool LsmServiceManager::InitWakeup(uint8_t &rStatus)
{
    rStatus |= m_Imu.Enable_Wake_Up_Detection(LSM6DSV16X_INT1_PIN);
    rStatus |= m_Imu.Set_Wake_Up_Threshold(WU_THS);
    return (rStatus == LSM6DSV16X_OK);
}

void LsmServiceManager::ResetFIFO()
{
    m_Imu.FIFO_Set_Mode(LSM6DSV16X_BYPASS_MODE);
    delay(20);
    m_Imu.FIFO_Set_Mode(LSM6DSV16X_STREAM_MODE);
    delay(20);
}

void LsmServiceManager::FillBufferData() {}

void LsmServiceManager::ObtainData(data::SamplePacket &rPacketOut)
{
    startTime = millis();
    if(m_Imu.FIFO_Get_Num_Samples(&fifo_samples) != LSM6DSV16X_OK)
    {
        Serial.println("Failed to get number of samples inside FIFO");
        return;
    }
    if(fifo_samples == 0)
    {
        return;
    }
    for(int i = 0; i < fifo_samples; i++)
    {
        if(m_Imu.FIFO_Get_Tag(&m_Tag) != LSM6DSV16X_OK)
        {
            Serial.println("FIFO tag read error");
            return;
        }
        if(m_Tag == TAG_GAME_ROTATION_VECTOR)
        {
            if(m_Imu.FIFO_Get_Rotation_Vector(&m_LatestQuat[0]) != LSM6DSV16X_OK)
            {
                Serial.println("Error in FIFO GET ROTATION VECTOR");
                return;
            }
            if(isnan(m_LatestQuat[0]) || isnan(m_LatestQuat[1])
               || isnan(m_LatestQuat[2]) || isnan(m_LatestQuat[3]))
            {
                Serial.println("Discarding NAN quaternion values");
                continue;
            }

            // Serial.print("Quaternion wewnatrz obtain data: ");
            // Serial.print(m_LatestQuat[3], 4);
            // Serial.print(", ");
            // Serial.print(m_LatestQuat[0], 4);
            // Serial.print(", ");
            // Serial.print(m_LatestQuat[1], 4);
            // Serial.print(", ");
            // Serial.println(m_LatestQuat[2], 4);

            m_HaveFreshQuat = true;

            if((long)(ALGO_PERIOD_MS - elapsedTime) > 0)
            {
                delay(ALGO_PERIOD_MS - elapsedTime);
            }
        } else if(m_Tag == TAG_ACCELEROMETER)
        {
            m_Imu.FIFO_Get_X_Axes(m_LatestAccel);
            m_HaveFreshAccel = true;
        } else
        {
            int32_t dummy[3];
            m_Imu.FIFO_Get_G_Axes(dummy);
            continue;
        }

        if(m_HaveFreshAccel && m_HaveFreshQuat)
        {
            rPacketOut.q[0] = toQ14(m_LatestQuat[0]);
            rPacketOut.q[1] = toQ14(m_LatestQuat[1]);
            rPacketOut.q[2] = toQ14(m_LatestQuat[2]);
            rPacketOut.q[3] = toQ14(m_LatestQuat[3]);

            rPacketOut.a[0] = (int16_t)m_LatestAccel[0];
            rPacketOut.a[1] = (int16_t)m_LatestAccel[1];
            rPacketOut.a[2] = (int16_t)m_LatestAccel[2];

            elapsedTime = millis() - startTime;
        }
    }
}

} // namespace w8band::lsm_service_manager
