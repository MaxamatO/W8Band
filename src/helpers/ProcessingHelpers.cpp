#include "ProcessingHelpers.hpp"
namespace helpers::Processing
{

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

}
