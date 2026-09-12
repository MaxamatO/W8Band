#include "ProcessingHelpers.hpp"
#include <math.h>
namespace helpers::Processing
{
constexpr float LPF_B0 = 0.0494899563f;
constexpr float LPF_B1 = 0.0989799125f;
constexpr float LPF_B2 = 0.0494899563f;
constexpr float LPF_A1 = -1.2796324250f;
constexpr float LPF_A2 = 0.4775922501f;

void LowPassFilter::Reset()
{
    m_Initialized = false;
    m_X1 = 0.0f;
    m_X2 = 0.0f;
    m_Y1 = 0.0f;
    m_Y2 = 0.0f;
}

float LowPassFilter::Process(float value)
{
    if(!m_Initialized)
    {
        m_Initialized = true;
        m_X1 = value;
        m_X2 = value;
        m_Y1 = value;
        m_Y2 = value;
        return value;
    }

    const float filtered = LPF_B0 * value + LPF_B1 * m_X1 + LPF_B2 * m_X2
                           - LPF_A1 * m_Y1 - LPF_A2 * m_Y2;
    m_X2 = m_X1;
    m_X1 = value;
    m_Y2 = m_Y1;
    m_Y1 = filtered;
    return filtered;
}

/// @brief Rotates body from to world frame
/// @param[in] qw Quaternion real part
/// @param[in] qx Quaternion x
/// @param[in] qy Quaternion y
/// @param[in] qz Quaternion z
/// @param[in] vx
/// @param[in] vy
/// @param[in] vz
/// @param[out] rOutX Rotated X axis pointing to world's X
/// @param[out] rOutY Rotated Y axis pointing to world's Y
/// @param[out] rOutZ Rotated Z axis pointing to world's Z
void RotateBodyToWorld(float qw, float qx, float qy, float qz, float vx,
                       float vy, float vz, float &rOutX, float &rOutY,
                       float &rOutZ)
{
    float cx = qy * vz - qz * vy;
    float cy = qz * vx - qx * vz;
    float cz = qx * vy - qy * vx;

    float ccx = qy * cz - qz * cy;
    float ccy = qz * cx - qx * cz;
    float ccz = qx * cy - qy * cx;

    rOutX = vx + 2.0f * qw * cx + 2.0f * ccx;
    rOutY = vy + 2.0f * qw * cy + 2.0f * ccy;
    rOutZ = vz + 2.0f * qw * cz + 2.0f * ccz;
}

Quat DecodeQuaternion(const data::SamplePacket &rPacket, Quat &rPrev)
{
    Quat q{rPacket.q[0] / Q14_SCALE, rPacket.q[1] / Q14_SCALE,
           rPacket.q[2] / Q14_SCALE, rPacket.q[3] / Q14_SCALE};

    float norm = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if(norm > 1e-6f)
    {
        q.w /= norm;
        q.x /= norm;
        q.y /= norm;
        q.z /= norm;
    }

    float dot = q.w * rPrev.w + q.x * rPrev.x + q.y * rPrev.y + q.z * rPrev.z;
    if(dot < 0.0f)
    {
        q.w = -q.w;
        q.x = -q.x;
        q.y = -q.y;
        q.z = -q.z;
    }

    rPrev = q;
    return q;
}

}
