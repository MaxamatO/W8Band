#pragma once

namespace helpers::Processing
{
void RotateBodyToWorld(float qw, float qx, float qy, float qz, float vx,
                       float vy, float vz, float &rOutX, float &rOutY,
                       float &rOutZ);
}