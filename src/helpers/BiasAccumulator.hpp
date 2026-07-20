#include "DataTypes.hpp"

struct BiasAccumulator
{
    void Reset() { *this = BiasAccumulator{}; }

    void Add(const data::SamplePacket &rPacket)
    {
        m_SumAx += rPacket.a[0];
        m_SumAy += rPacket.a[1];
        m_SumAz += rPacket.a[2];

        m_Count++;
    }

    bool IsFull(std::size_t target) const { return m_Count >= target; }

    std::size_t GetCount() const { return m_Count; }

    float GetMeanAx() const { return m_SumAx / static_cast<float>(m_Count); }
    float GetMeanAy() const { return m_SumAy / static_cast<float>(m_Count); }
    float GetMeanAz() const { return m_SumAz / static_cast<float>(m_Count); }

private:
    float m_SumAx = 0, m_SumAy = 0, m_SumAz = 0;
    float m_SumGx = 0, m_SumGy = 0, m_SumGz = 0;
    float m_SumQw = 0, m_SumQx = 0, m_SumQy = 0, m_SumQz = 0;

    std::size_t m_Count = 0;
};