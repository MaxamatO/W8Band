#pragma once

#include <array>
#include <cmath>
#include <cstddef>

namespace helpers
{
/// Rolling signal-quality detector used together with velocity and phase
/// history. Acceleration magnitude rejects general movement, while world-Z
/// RMS rejects vertical vibration around a drifting integrated velocity.
/// @tparam WindowSize Number of samples stored in the rolling window.
template <std::size_t WindowSize> class EndMotionDetector
{
public:
    /// @brief Clears samples and accumulated rolling statistics.
    void Reset()
    {
        m_Count = 0;
        m_Index = 0;
        m_SumAccel = 0.0f;
        m_SumSqAccel = 0.0f;
        m_SumSqVerticalAccel = 0.0f;
    }

    /// @brief Appends one sample and removes the oldest sample when full.
    /// @param[in] accelMagnitudeMg Raw acceleration-vector magnitude in mg.
    /// @param[in] verticalAccelMs2 Filtered world-frame Z acceleration in m/s^2.
    void Push(float accelMagnitudeMg, float verticalAccelMs2)
    {
        if(m_Count == WindowSize)
        {
            const float oldAccel = m_AccelMagnitude[m_Index];
            const float oldVerticalAccel = m_VerticalAccel[m_Index];
            m_SumAccel -= oldAccel;
            m_SumSqAccel -= oldAccel * oldAccel;
            m_SumSqVerticalAccel
                -= oldVerticalAccel * oldVerticalAccel;
        } else
        {
            ++m_Count;
        }

        m_AccelMagnitude[m_Index] = accelMagnitudeMg;
        m_VerticalAccel[m_Index] = verticalAccelMs2;
        m_SumAccel += accelMagnitudeMg;
        m_SumSqAccel += accelMagnitudeMg * accelMagnitudeMg;
        m_SumSqVerticalAccel += verticalAccelMs2 * verticalAccelMs2;
        m_Index = (m_Index + 1) % WindowSize;
    }

    /// @brief Checks whether the window contains WindowSize samples.
    /// @return True when rolling statistics cover the complete window.
    bool IsFull() const { return m_Count == WindowSize; }

    /// @brief Calculates variance of acceleration-vector magnitude.
    /// @return Variance in mg^2, or -1 when the window is not full.
    float GetAccelVariance() const
    {
        if(!IsFull())
        {
            return -1.0f;
        }
        const float n = static_cast<float>(WindowSize);
        const float mean = m_SumAccel / n;
        const float variance = m_SumSqAccel / n - mean * mean;
        return variance > 0.0f ? variance : 0.0f;
    }

    /// @brief Calculates RMS of filtered vertical acceleration.
    /// @return RMS in m/s^2, or -1 when the window is not full.
    float GetVerticalAccelRms() const
    {
        if(!IsFull())
        {
            return -1.0f;
        }
        return std::sqrt(m_SumSqVerticalAccel
                         / static_cast<float>(WindowSize));
    }

    /// @brief Checks both stillness metrics against supplied limits.
    /// @param[in] maxAccelVariance Maximum acceleration variance in mg^2.
    /// @param[in] maxVerticalAccelRms Maximum vertical RMS in m/s^2.
    /// @return True when the window is full and both limits are satisfied.
    bool IsStill(float maxAccelVariance, float maxVerticalAccelRms) const
    {
        return IsFull() && GetAccelVariance() <= maxAccelVariance
               && GetVerticalAccelRms() <= maxVerticalAccelRms;
    }

private:
    /// @brief Rolling raw acceleration-magnitude samples in mg.
    std::array<float, WindowSize> m_AccelMagnitude{};

    /// @brief Rolling filtered vertical-acceleration samples in m/s^2.
    std::array<float, WindowSize> m_VerticalAccel{};

    /// @brief Sum of acceleration magnitudes in the current window.
    float m_SumAccel = 0.0f;

    /// @brief Sum of squared acceleration magnitudes in the current window.
    float m_SumSqAccel = 0.0f;

    /// @brief Sum of squared vertical accelerations in the current window.
    float m_SumSqVerticalAccel = 0.0f;

    /// @brief Number of valid samples currently stored.
    std::size_t m_Count = 0;

    /// @brief Index at which the next sample will be written.
    std::size_t m_Index = 0;
};
} // namespace helpers
