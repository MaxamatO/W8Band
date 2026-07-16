#pragma once
#include "DataTypes.hpp"
#include <LSM6DSV16XSensor.h>

#define IMU_I2C_ADDRESS LSM6DSV16X_I2C_ADD_L

namespace w8band::Hardware
{
/// @brief Class responsible for initializing, configuring and managing
/// LSM6DSV16X sensor
class LsmServiceManager
{
public:
    /// @brief Default constructor
    LsmServiceManager(TwoWire &rWire, uint8_t i2cAddress = IMU_I2C_ADDRESS);

    /// @brief Initialize LSM6DSV16X, calls InitWakeup() at the end.
    /// @return True if initialization was correct, false otherwise.
    bool InitLsm();

    /// @brief Method used for obtaining quaternions and accelerometer data
    /// from lsm's FIFO. And saving it into provided struct.
    /// @param[out] rPacketOut Struct passed by reference for storing loaded
    /// data. For more infromation about struct @see SamplePacket in
    /// DataTypes.hpp
    void ObtainData(data::SamplePacket &rPacketOut);

    /// @brief Method used to fill provided buffer data for calibration.
    void FillBufferData();

    /// @brief Method used for reseting FIFO by setting it into BYPASS mode,
    /// and into STREAM mode
    void ResetFIFO();

private:
    /// @brief Private LSM instance
    LSM6DSV16XSensor m_Imu;

    /// @brief Initialize LSM wakeup interrupt detection.
    /// @param[in] rStatus status passed by InitLsm to check for correct
    /// initialization.
    /// @return True if initalization was correct, false otherwise.
    bool InitWakeup(uint8_t &rStatus);

    /// @brief Used to determine type of data from LSM FIFO
    uint8_t m_Tag = 0;

    /// @brief Amount of data inside lsm's FIFO
    uint16_t fifo_samples = 0;

    /// @brief Latest quaternions obtained from LSM6DSV16X
    float m_LatestQuat[4] = {};

    /// @brief Latest acceleration data obtained from LSM6DSV16X
    int32_t m_LatestAccel[3] = {};

    /// @brief Indicator if obtained quaternions are recent
    bool m_HaveFreshQuat = false;

    /// @brief Indicator if obtained acceleration values are recent
    bool m_HaveFreshAccel = false;
};

} // namespace w8band::lsm_service_manager
