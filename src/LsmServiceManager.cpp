#include "LsmServiceManager.hpp"
#include "DataTypes.hpp"

#define IMU_FREQ 120.0f
/*
    WAK_UP_THS (0x5Bh) holds 6 bit number (WK_THS_[5:0]).
    INACTIVITY_DUR (0x54h) holds 3 bit number (WU_INACT_THS_W_[2:0]). It's a
   resolution, that we multiply WAKE UP THS value by, to get our THS in mg. so
   REAL_THS_MG = WAKE_UP_THS * INACTIVITY_DUR Where resolution can be: 000
   - 7.8125 mg/LSB - default
    .
    .   Multiply previous value by 2.
    .
    100
    101/110/111 - 250 mg/LSB
    so in our case REAL_THS_MG = 20 * 001
*/
#define WU_THS 6

#define TAG_GAME_ROTATION_VECTOR 0x13u
#define TAG_GRAVITY_VECTOR 0x17u
#define TAG_ACCELEROMETER 0x02u
#define TAG_TIMESTAMP 0x04u

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
    int32_t x = (int32_t)(v * Q14_SCALE);
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
    Serial.println("dupa2");
    uint8_t status = 0;

    status |= m_Imu.begin();
    Serial.println("dupa3");
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
    Serial.println("Before gvec");
    Serial.flush();
    delay(10);
    status |= m_Imu.Enable_Gravity_Vector();
    delay(20);
    Serial.println("after gvec 1");
    Serial.flush();
    delay(10);

    status |= m_Imu.FIFO_Set_Mode(LSM6DSV16X_BYPASS_MODE);
    delay(20);
    status |= m_Imu.Set_SFLP_Batch(true, true, false);
    status |= m_Imu.FIFO_Enable_Timestamp();
    status |= m_Imu.FIFO_Set_Timestamp_Decimation(
        LSM6DSV16X_TMSTMP_DEC_1);
    Serial.println("after gvec 2");
    delay(10);
    status |= m_Imu.FIFO_Set_X_BDR(IMU_FREQ);
    status |= m_Imu.FIFO_Set_Mode(LSM6DSV16X_STREAM_MODE);
    delay(20);

    InitWakeup(status);
    return (status == LSM6DSV16X_OK);
}

bool LsmServiceManager::InitWakeup(uint8_t &rStatus)
{
    rStatus |= m_Imu.Enable_Wake_Up_Detection(LSM6DSV16X_INT1_PIN);

    // Enables High pass filter mode, so we can transition smoothly, instead of needing a jitter.
    rStatus |= m_Imu.Set_X_Filter_Mode(1, 4);

    // According do documentation, we also need to enable HPF in register
    // TAP_CFG (56h), more specifically, value SLOPE_FDS of that register.
    // SLOPE_FDS - 0: slope filter applied, 1: HPF applied
    uint8_t tapCfg0 = 0;

    // Set 4th bit of TAP_CFG0 to 1 (SLOPE_FDS)
    m_Imu.Read_Reg(LSM6DSV16X_TAP_CFG0, &tapCfg0);
    tapCfg0 |= (1 << 4);
    m_Imu.Write_Reg(LSM6DSV16X_TAP_CFG0, tapCfg0);

    // Enable Hihh pass filter again...
    // uint8_t ctrl9Reg = 0;
    // m_Imu.Read_Reg(LSM6DSV16X_CTRL9, &ctrl9Reg);
    // ctrl9Reg |= (1 << 4); // HP_SLOPE_XL_EN = 1
    // m_Imu.Write_Reg(LSM6DSV16X_CTRL9, ctrl9Reg);

    // Set rosolution
    uint8_t inactDurReg = 0;
    m_Imu.Read_Reg(LSM6DSV16X_INACTIVITY_DUR, &inactDurReg);

    inactDurReg |= (0b010 << 4);
    m_Imu.Write_Reg(LSM6DSV16X_INACTIVITY_DUR, inactDurReg);

    m_Imu.Set_Wake_Up_Threshold(WU_THS);

    return (rStatus == LSM6DSV16X_OK);
}

void LsmServiceManager::ResetFIFO()
{
    m_Imu.FIFO_Set_Mode(LSM6DSV16X_BYPASS_MODE);
    delay(20);
    m_Imu.FIFO_Set_Mode(LSM6DSV16X_STREAM_MODE);
    delay(20);

    // Do not let data from before the reset become part of the first frame.
    m_HaveFreshAccel = false;
    m_HaveFreshQuat = false;
    m_HaveFreshGV = false;
    m_HaveFreshTimestamp = false;
}

void LsmServiceManager::FillBufferData() {}

bool LsmServiceManager::ObtainData(data::SamplePacket &rPacketOut)
{
    startTime = millis();
    if(m_Imu.FIFO_Get_Num_Samples(&fifo_samples) != LSM6DSV16X_OK)
    {
        Serial.println("Failed to get number of samples inside FIFO");
        return false;
    }
    if(fifo_samples == 0)
    {
        return false;
    }
    for(int i = 0; i < fifo_samples; i++)
    {
        if(m_Imu.FIFO_Get_Tag(&m_Tag) != LSM6DSV16X_OK)
        {
            Serial.println("FIFO tag read error");
            return false;
        }
        if(m_Tag == TAG_GAME_ROTATION_VECTOR)
        {
            if(m_Imu.FIFO_Get_Rotation_Vector(&m_LatestQuat[0]) != LSM6DSV16X_OK)
            {
                Serial.println("Error in FIFO GET ROTATION VECTOR");
                return false;
            }
            if(isnan(m_LatestQuat[0]) || isnan(m_LatestQuat[1])
               || isnan(m_LatestQuat[2]) || isnan(m_LatestQuat[3]))
            {
                Serial.println("Discarding NAN quaternion values");
                continue;
            }

            m_HaveFreshQuat = true;

        } else if(m_Tag == TAG_ACCELEROMETER)
        {
            m_Imu.FIFO_Get_X_Axes(m_LatestAccel);
            m_HaveFreshAccel = true;
        } else if(m_Tag == TAG_GRAVITY_VECTOR)
        {
            m_Imu.FIFO_Get_Gravity_Vector(m_LatestGravityVector);
            m_HaveFreshGV = true;
        } else if(m_Tag == TAG_TIMESTAMP)
        {
            if(m_Imu.FIFO_Get_Timestamp(&m_LatestTimestampTicks)
               != LSM6DSV16X_OK)
            {
                Serial.println("Error in FIFO GET TIMESTAMP");
                return false;
            }
            m_HaveFreshTimestamp = true;
        } else
        {
            uint8_t dummy[6];
            m_Imu.FIFO_Get_Data(dummy);
            continue;
        }

        if(m_HaveFreshAccel && m_HaveFreshQuat && m_HaveFreshGV
           && m_HaveFreshTimestamp)
        {
            elapsedTime = millis() - startTime;

            rPacketOut.q[0] = toQ14(m_LatestQuat[3]); // W
            rPacketOut.q[1] = toQ14(m_LatestQuat[0]); // X
            rPacketOut.q[2] = toQ14(m_LatestQuat[1]); // Y
            rPacketOut.q[3] = toQ14(m_LatestQuat[2]); // Z

            rPacketOut.a[0] = (int16_t)m_LatestAccel[0]; // X
            rPacketOut.a[1] = (int16_t)m_LatestAccel[1]; // Y
            rPacketOut.a[2] = (int16_t)m_LatestAccel[2]; // Z

            rPacketOut.gv[0] = (int16_t)m_LatestGravityVector[0]; // X
            rPacketOut.gv[1] = (int16_t)m_LatestGravityVector[1]; // Y
            rPacketOut.gv[2] = (int16_t)m_LatestGravityVector[2]; // Z

            rPacketOut.timestamp_ticks = m_LatestTimestampTicks;
            m_HaveFreshAccel = false;
            m_HaveFreshGV = false;
            m_HaveFreshQuat = false;
            m_HaveFreshTimestamp = false;

            return true;
        }
    }
    return false;
}

} // namespace w8band::lsm_service_manager
