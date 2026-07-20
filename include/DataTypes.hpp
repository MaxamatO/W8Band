#pragma once
#include <memory>
namespace data
{
/// @brief Struct describing single packet in m_Data, thats sent over with BLE.
/// It needs to be packed
struct __attribute__((packed)) SamplePacket
{
    /// @brief Quaternions data obtained from IMU using built in SFLP
    /// Qw Qx Qy Qz  Q1.14 format
    /// 8 bytes
    int16_t q[4];

    /// @brief Acceleration data obtained from IMU in mg
    /// Ax, Ay, Az
    /// 6 bytes
    int16_t a[3];

    /// @brief Number of sample
    /// 2 bytes
    uint16_t seq;

    /// @brief Timestamp of uploaded data
    /// 4 bytes
    uint32_t timestamp_ms;
};
static_assert(sizeof(SamplePacket) == 20, "SamplePacket must be 20 bytes");

/// @brief Packed struct Vec3 used to return calculated accelerometer bias in x,y,z axes
struct __attribute__((packed)) AccBiasVec3
{
    /// @brief ax bias
    int32_t axBias;

    /// @brief ay bias
    int32_t ayBias;

    /// @brief az bias
    int32_t azBias;
};
static_assert(sizeof(AccBiasVec3) == 12, "AccBiasVec3 must be 12 bytes");

} // namespace data
