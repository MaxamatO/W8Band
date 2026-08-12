#pragma once
#include "DataTypes.hpp"

struct BiasAccumulator
{
    void Reset() { *this = BiasAccumulator{}; }

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

    bool IsFull(std::size_t target) const { return m_Count >= target; }

    std::size_t GetCount() const { return m_Count; }

    float GetMeanAx() const { return m_SumAx / static_cast<float>(m_Count); }
    float GetMeanAy() const { return m_SumAy / static_cast<float>(m_Count); }
    float GetMeanAz() const { return m_SumAz / static_cast<float>(m_Count); }

    float GetMeanGx() const { return m_SumGx / static_cast<float>(m_Count); }
    float GetMeanGy() const { return m_SumGy / static_cast<float>(m_Count); }
    float GetMeanGz() const { return m_SumGz / static_cast<float>(m_Count); }

    float GetMeanQw() const { return m_SumQw / static_cast<float>(m_Count); }
    float GetMeanQx() const { return m_SumQx / static_cast<float>(m_Count); }
    float GetMeanQy() const { return m_SumQy / static_cast<float>(m_Count); }
    float GetMeanQz() const { return m_SumQz / static_cast<float>(m_Count); }

private:
    float m_SumAx = 0, m_SumAy = 0, m_SumAz = 0;
    float m_SumGx = 0, m_SumGy = 0, m_SumGz = 0;
    float m_SumQw = 0, m_SumQx = 0, m_SumQy = 0, m_SumQz = 0;

    std::size_t m_Count = 0;
};