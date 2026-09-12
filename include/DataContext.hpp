#pragma once
#include "BleServiceManager.hpp"
#include "DataTypes.hpp"
#include "LsmServiceManager.hpp"
#include "motion-processor/MotionProcessor.hpp"
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
    /// @brief Constructs shared state data using hardware service managers.
    /// @param[in,out] m_rBle BLE service manager shared by application states.
    /// @param[in,out] m_rLsm IMU service manager shared by application states.
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

    /// @brief Runtime configuration used by batch motion processing.
    Motion::ProcessingConfig m_ProcessingConfig{};

    /// @brief Most recently calculated trajectory and repetition metrics.
    Motion::ProcessingResult m_ProcessingResult{};

    /// @brief Helper method to populate rolling buffer to have continous data.
    /// @param[in] rPacket Reference to packet to push to buffer
    void PushToPreBuffer(data::SamplePacket &rPacket);

    /// @brief Appends buffered packets to m_Data in chronological order.
    void DropPreBufferToEndData();

    /// @brief Calculates raw accelerometer-vector magnitude in mg.
    /// @param[in] rPacket Packet containing raw accelerometer samples.
    /// @return Euclidean acceleration magnitude in mg.
    static float CalculateAccelMagnitude(const data::SamplePacket &rPacket);

    /// @brief Removes calibrated bias and SFLP gravity in the sensor frame.
    /// @param[in] rPacket Packet containing acceleration and gravity vectors.
    /// @param[out] rLinAx Linear X-axis acceleration in mg.
    /// @param[out] rLinAy Linear Y-axis acceleration in mg.
    /// @param[out] rLinAz Linear Z-axis acceleration in mg.
    void GetLinearAccel(const data::SamplePacket &rPacket, float &rLinAx,
                        float &rLinAy, float &rLinAz);

    // void CalculateVelocityRealTime(data::SamplePacket &rPacket, )
};

} // namespace w8band::DataContext
