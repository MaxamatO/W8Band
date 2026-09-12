#pragma once

#include "DataTypes.hpp"

namespace helpers::Processing
{

/// @brief Normalized quaternion in scalar-first order.
struct Quat
{
    /// @brief Real component.
    float w;
    /// @brief X imaginary component.
    float x;
    /// @brief Y imaginary component.
    float y;
    /// @brief Z imaginary component.
    float z;
};

/// Causal 2nd-order low-pass filter tuned for 120 Hz input and a 10 Hz cutoff.
/// Reset-on-first-sample initialization avoids a synthetic startup transient.
class LowPassFilter
{
public:
    /// @brief Clears filter history and enables steady-state initialization.
    void Reset();

    /// @brief Filters one input sample using the causal biquad.
    /// @param[in] value Current input sample.
    /// @return Current filtered output sample.
    float Process(float value);

private:
    /// @brief True after filter history has been initialized.
    bool m_Initialized = false;

    /// @brief Input sample delayed by one step.
    float m_X1 = 0.0f;

    /// @brief Input sample delayed by two steps.
    float m_X2 = 0.0f;

    /// @brief Output sample delayed by one step.
    float m_Y1 = 0.0f;

    /// @brief Output sample delayed by two steps.
    float m_Y2 = 0.0f;
};

/// @brief Rotates a vector from sensor body frame to world frame.
/// @param[in] qw Quaternion real component.
/// @param[in] qx Quaternion X imaginary component.
/// @param[in] qy Quaternion Y imaginary component.
/// @param[in] qz Quaternion Z imaginary component.
/// @param[in] vx Body-frame vector X component.
/// @param[in] vy Body-frame vector Y component.
/// @param[in] vz Body-frame vector Z component.
/// @param[out] rOutX World-frame vector X component.
/// @param[out] rOutY World-frame vector Y component.
/// @param[out] rOutZ World-frame vector Z component.
void RotateBodyToWorld(float qw, float qx, float qy, float qz, float vx,
                       float vy, float vz, float &rOutX, float &rOutY,
                       float &rOutZ);

/// @brief Decodes, normalizes and sign-aligns a Q1.14 SFLP quaternion.
/// @param[in] rPacket Packet containing the raw scalar-first quaternion.
/// @param[in,out] rPrev Previous normalized quaternion, replaced by result.
/// @return Normalized quaternion aligned with rPrev.
Quat DecodeQuaternion(const data::SamplePacket &rPacket, Quat &rPrev);

} // namespace helpers::Processing
