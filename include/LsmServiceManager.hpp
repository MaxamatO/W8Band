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
    /// @brief Constructs an LSM6DSV16X service on the selected I2C bus.
    /// @param[in,out] rWire I2C bus used to communicate with the sensor.
    /// @param[in] i2cAddress LSM6DSV16X I2C address.
    LsmServiceManager(TwoWire &rWire, uint8_t i2cAddress = IMU_I2C_ADDRESS);

    /// @brief Initialize LSM6DSV16X, calls InitWakeup() at the end.
    /// @return True if initialization was correct, false otherwise.
    bool InitLsm();

    /// @brief Method used for obtaining quaternions and accelerometer data
    /// from lsm's FIFO. And saving it into provided struct.
    /// @param[out] rPacketOut Struct passed by reference for storing loaded
    /// data. For more infromation about struct @see SamplePacket in
    /// DataTypes.hpp
    /// @return True if obtained data consists of q, a, gv and a sensor
    /// timestamp. False if a complete frame is not available yet.
    bool ObtainData(data::SamplePacket &rPacketOut);

    /// @brief Method used to fill provided buffer data for calibration.
    void FillBufferData();

    /// @brief Method used for reseting FIFO by setting it into BYPASS mode,
    /// and into STREAM mode
    void ResetFIFO();

private:
    /// @brief Private LSM instance
    LSM6DSV16XSensor m_Imu;

    /// @brief Initialize LSM wakeup interrupt detection.
    /// @param[in,out] rStatus Accumulated initialization status updated with
    /// wake-up configuration results.
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

    /// @brief Latest gravity vector data obtained from LSM6DSV16X
    float m_LatestGravityVector[3] = {};

    /// @brief Latest raw timestamp obtained from the sensor FIFO.
    uint32_t m_LatestTimestampTicks = 0;

    /// @brief Indicator if obtained quaternions are recent
    bool m_HaveFreshQuat = false;

    /// @brief Indicator if obtained acceleration values are recent
    bool m_HaveFreshAccel = false;

    /// @brief Indicator if obtained Gravity Vector values are recent
    bool m_HaveFreshGV = false;

    /// @brief Indicator if a fresh sensor timestamp is available.
    bool m_HaveFreshTimestamp = false;
};

} // namespace w8band::lsm_service_manager
