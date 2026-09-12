#pragma once
#include <array>
#include <cstddef>

namespace helpers
{
/// @brief Detects stillness from acceleration-magnitude variance.
/// @tparam WindowSize Number of samples stored in the rolling window.
template <std::size_t WindowSize> class StillnessDetector
{
public:
    /// @brief Clears samples and accumulated rolling statistics.
    void Reset()
    {
        m_Count = 0;
        m_Index = 0;
        m_SumAccel = m_SumSqAccel = 0;
    }

    /// @brief Appends one acceleration-magnitude sample to the window.
    /// @param[in] accelMag Raw acceleration-vector magnitude in mg.
    void Push(float accelMag)
    {
        if(m_Count == WindowSize)
        {
            float oldA = m_AccelBuffer[m_Index];
            m_SumAccel -= oldA;
            m_SumSqAccel -= oldA * oldA;
        } else
        {
            m_Count++;
        }

        m_AccelBuffer[m_Index] = accelMag;
        m_SumAccel += accelMag;
        m_SumSqAccel += accelMag * accelMag;

        m_Index = (m_Index + 1) % WindowSize;
    }

    /// @brief Checks whether the window contains WindowSize samples.
    /// @return True when rolling statistics cover the complete window.
    bool IsFull() const { return m_Count == WindowSize; }

    /// @brief Compares acceleration variance with the stillness threshold.
    /// @param[in] accelVarThreshold Maximum variance in mg^2.
    /// @return True when the window is full and variance is below threshold.
    bool IsStill(float accelVarThreshold) const
    {
        if(!IsFull())
        {
            return false;
        }
        return GetAccelVariance() < accelVarThreshold;
    }

    /// @brief Calculates acceleration-magnitude variance over the window.
    /// @return Variance in mg^2, or -1 when the window is not full.
    float GetAccelVariance() const
    {
        if(!IsFull())
        {
            return -1.0f;
        }
        float n = static_cast<float>(WindowSize);
        const float mean = m_SumAccel / n;
        const float variance = m_SumSqAccel / n - mean * mean;
        return variance > 0.0f ? variance : 0.0f;
    }

private:
    /// @brief Rolling acceleration-magnitude samples in mg.
    std::array<float, WindowSize> m_AccelBuffer{};

    /// @brief Sum of acceleration magnitudes in the current window.
    float m_SumAccel = 0;

    /// @brief Sum of squared acceleration magnitudes in the current window.
    float m_SumSqAccel = 0;

    /// @brief Number of valid samples currently stored.
    std::size_t m_Count = 0;

    /// @brief Index at which the next sample will be written.
    std::size_t m_Index = 0;
};

} // namespace helpers
