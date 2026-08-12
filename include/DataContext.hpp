#pragma once
#include "BleServiceManager.hpp"
#include "DataTypes.hpp"
#include "LsmServiceManager.hpp"
#include <memory>
#include <vector>

/// @brief Value for number of samples to constantly buffer, in order to
/// calibrate for gravity.
#define PRE_RECORD_SAMPLES 60

namespace w8band
{

struct DataContext
{
public:
    DataContext(Hardware::BleServiceManager &m_rBle,
                Hardware::LsmServiceManager &m_rLsm);

    /// @brief Reference to BleServiceManager for handling BLE communication
    Hardware::BleServiceManager &m_rBleServiceManager;

    /// @brief Reference to LsmServiceManager for handling LSM6DSV16X
    Hardware::LsmServiceManager &m_rLsmServiceManager;

    /// @brief Buffer used for calibration - calculating gravity from stored data
    data::SamplePacket m_PreBuffer[PRE_RECORD_SAMPLES];

    /// @brief Helpers for inserting m_PreBuffer values into m_Data
    uint16_t m_PreIndex = 0;
    uint16_t m_PreCount = 0;

    /// @brief Vector of result data
    std::vector<data::SamplePacket> m_Data;

    /// @brief Set by ISR in order to determine lift off
    bool m_LiftOffDetected;

    /// @brief Set by StillnessDetector in order to begin RecordingState
    bool m_MotionStartDetected;

    /// @brief Time in ms in which the recording RecordingState::OnEnter has
    /// been called. Taken from millis()
    uint32_t m_RecordingStartedMs;

    /// @brief Accelerometer bias computer by CalibrationState. Valid when
    /// m_CalibrationValid == true
    data::AccBiasVec3 m_AccelBias{};

    /// @brief True once CalibrationState has produced a bias.
    bool m_CalibrationValid = false;

    /// @brief Helper method to populate rolling buffer to have continous data.
    /// @param[in] rPacket Reference to packet to push to buffer
    void PushToPreBuffer(data::SamplePacket &rPacket);

    /// @brief Helper method for dropping all data in order from m_PreBuffer into m_Data.
    void DropPreBufferToEndData();

    /// @brief Helper method to calculate magniture
    /// @param[in] rPacket Reference for data to calculate magnitude based on
    /// @param[out] rAccelMag Output Accelerometer Magnitude
    /// @param[out] rGyroMag Output Gyroscope Magnitude
    static void CalculateMagnitude(data::SamplePacket &rPacket,
                                   float &rAccelMag, float &rGyroMag);

    /// @brief Helper method to calculate linear acceleration
    /// @param[in] rPacket Reference to packet for data
    /// @param [out] rLinAx Reference to linear x axis acceleration
    /// @param [out] rLinAy Reference to linear y axis acceleration
    /// @param [out] rLinAz Reference to linear z axis acceleration
    void GetLinearAccel(data::SamplePacket &rPacket, float &rLinAx,
                        float &rLinAy, float &rLinAz);

    // void CalculateVelocityRealTime(data::SamplePacket &rPacket, )
};

} // namespace w8band::DataContext
