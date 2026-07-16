#pragma once
#include "BleServiceManager.hpp"
#include "DataTypes.hpp"
#include "LsmServiceManager.hpp"
#include <memory>
#include <vector>

///  @brief Value for number of samples to constantly buffer, in order to
/// calibrate for gravity.
#define PRE_RECORD_SAMPLES 120

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

    /// @brief Set by ISR in order to start recording data
    bool m_WakeUpDetected;
};

} // namespace w8band::DataContext
