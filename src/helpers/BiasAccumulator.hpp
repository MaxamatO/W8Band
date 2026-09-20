#pragma once
#include "DataTypes.hpp"

/// @brief Accumulates sensor samples and calculates their mean values.
struct BiasAccumulator
{
    /// @brief Clears all sums and the sample counter.
    void Reset() { *this = BiasAccumulator{}; }

    /// @brief Adds one synchronized sensor sample to every accumulator.
    /// @param[in] rPacket Sample containing acceleration, gravity and quaternion.
    void Add(const data::SamplePacket &rPacket)
    {
        m_SumAx += rPacket.a[0];
        m_SumAy += rPacket.a[1];
        m_SumAz += rPacket.a[2];

        m_SumQw += rPacket.q[0];
        m_SumQx += rPacket.q[1];
        m_SumQy += rPacket.q[2];
        m_SumQz += rPacket.q[3];

        m_SumGx += rPacket.gv[0];
        m_SumGy += rPacket.gv[1];
        m_SumGz += rPacket.gv[2];

        m_Count++;
    }

    /// @brief Checks whether the requested number of samples was collected.
    /// @param[in] target Required number of samples.
    /// @return True when at least target samples were added.
    bool IsFull(std::size_t target) const { return m_Count >= target; }

    /// @brief Returns the number of collected samples.
    /// @return Number of samples added since the last reset.
    std::size_t GetCount() const { return m_Count; }

    /// @brief Calculates mean accelerometer X value.
    /// @return Mean X-axis acceleration in raw sensor units.
    float GetMeanAx() const { return m_SumAx / static_cast<float>(m_Count); }

    /// @brief Calculates mean accelerometer Y value.
    /// @return Mean Y-axis acceleration in raw sensor units.
    float GetMeanAy() const { return m_SumAy / static_cast<float>(m_Count); }

    /// @brief Calculates mean accelerometer Z value.
    /// @return Mean Z-axis acceleration in raw sensor units.
    float GetMeanAz() const { return m_SumAz / static_cast<float>(m_Count); }

    /// @brief Calculates mean gravity-vector X value.
    /// @return Mean X-axis gravity value in raw sensor units.
    float GetMeanGx() const { return m_SumGx / static_cast<float>(m_Count); }

    /// @brief Calculates mean gravity-vector Y value.
    /// @return Mean Y-axis gravity value in raw sensor units.
    float GetMeanGy() const { return m_SumGy / static_cast<float>(m_Count); }

    /// @brief Calculates mean gravity-vector Z value.
    /// @return Mean Z-axis gravity value in raw sensor units.
    float GetMeanGz() const { return m_SumGz / static_cast<float>(m_Count); }

    /// @brief Calculates mean quaternion real component.
    /// @return Mean Q1.14 quaternion real component.
    float GetMeanQw() const { return m_SumQw / static_cast<float>(m_Count); }

    /// @brief Calculates mean quaternion X component.
    /// @return Mean Q1.14 quaternion X component.
    float GetMeanQx() const { return m_SumQx / static_cast<float>(m_Count); }

    /// @brief Calculates mean quaternion Y component.
    /// @return Mean Q1.14 quaternion Y component.
    float GetMeanQy() const { return m_SumQy / static_cast<float>(m_Count); }

    /// @brief Calculates mean quaternion Z component.
    /// @return Mean Q1.14 quaternion Z component.
    float GetMeanQz() const { return m_SumQz / static_cast<float>(m_Count); }

private:
    /// @brief Sum of accelerometer X values.
    float m_SumAx = 0;

    /// @brief Sum of accelerometer Y values.
    float m_SumAy = 0;

    /// @brief Sum of accelerometer Z values.
    float m_SumAz = 0;

    /// @brief Sum of gravity-vector X values.
    float m_SumGx = 0;

    /// @brief Sum of gravity-vector Y values.
    float m_SumGy = 0;

    /// @brief Sum of gravity-vector Z values.
    float m_SumGz = 0;

    /// @brief Sum of quaternion real components.
    float m_SumQw = 0;

    /// @brief Sum of quaternion X components.
    float m_SumQx = 0;

    /// @brief Sum of quaternion Y components.
    float m_SumQy = 0;

    /// @brief Sum of quaternion Z components.
    float m_SumQz = 0;

    /// @brief Number of samples included in the sums.
    std::size_t m_Count = 0;
};
