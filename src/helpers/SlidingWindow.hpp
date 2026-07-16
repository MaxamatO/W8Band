#pragma once
#include <memory>
namespace helpers
{
template <std::size_t WindowSize> class StillnessDetector
{
public:
    void Reset()
    {
        m_Count = 0;
        m_Index = 0;
        m_SumAccel = m_SumSqAccel = 0;
        m_SumGyro = m_SumSqGyro = 0;
    };
    void Push(float accelMag, float gyroMag)
    {
        if(m_Count == WindowSize)
        {
            float oldA = m_AccelBuffer[m_Index];
            float oldG = m_GyroBuffer[m_Index];
            m_SumAccel -= oldA;
            m_SumSqAccel -= oldA * oldA;
            m_SumGyro -= oldG;
            m_SumSqGyro -= oldG * oldG;
        } else
        {
            m_Count++;
        }

        m_AccelBuffer[m_Index] = accelMag;
        m_GyroBuffer[m_Index] = gyroMag;

        m_SumAccel += accelMag;
        m_SumSqAccel += accelMag * accelMag;

        m_SumGyro += gyroMag;
        m_SumGyro += gyroMag * gyroMag;

        m_Index = (m_Index + 1) % WindowSize;
    }

    bool IsFull() const { return m_Count == WindowSize; }

    bool isStill(float accelValThresh, float gyroValThresh) const
    {
        if(!IsFull())
        {
            return false;
        }

        float n = static_cast<float>(WindowSize);
        float accelVar
            = (m_SumSqAccel / n) - ((m_SumAccel / n) * (m_SumAccel / n));
        float gyroVar
            = (m_SumSqGyro / n) - ((m_SumGyro / n) * (m_SumGyro / n));
        return accelVar < accelValThresh && gyroVar < gyroValThresh;
    }

private:
    std::array<float, WindowSize> m_AccelBuffer{};
    std::array<float, WindowSize> m_GyroBuffer{};

    float m_SumAccel = 0;
    float m_SumSqAccel = 0;

    float m_SumGyro = 0;
    float m_SumSqGyro = 0;

    std::size_t m_Count = 0;
    std::size_t m_Index = 0;
};

} // namespace helpers
