#include "DataContext.hpp"
#include "BleServiceManager.hpp"
#include "LsmServiceManager.hpp"
namespace w8band
{
DataContext::DataContext(Hardware::BleServiceManager &m_rBle,
                         Hardware::LsmServiceManager &rLsm)
    : m_rBleServiceManager(m_rBle), m_rLsmServiceManager(rLsm)
{}

void DataContext::PushToPreBuffer(data::SamplePacket &rPacket)
{
    m_PreBuffer[m_PreIndex] = rPacket;
    m_PreIndex = (m_PreIndex + 1) % PRE_RECORD_SAMPLES;
    if(m_PreCount < PRE_RECORD_SAMPLES)
    {
        m_PreCount++;
    }
}

void DataContext::DropPreBufferToEndData()
{
    for(int i = 0; i < m_PreCount; i++)
    {
        uint16_t idx = (m_PreIndex - m_PreCount + i + PRE_RECORD_SAMPLES)
                       % PRE_RECORD_SAMPLES;
        m_Data.push_back(m_PreBuffer[idx]);
    }
}

float DataContext::CalculateAccelMagnitude(const data::SamplePacket &rPacket)
{
    float ax = rPacket.a[0];
    float ay = rPacket.a[1];
    float az = rPacket.a[2];
    return std::sqrt(ax * ax + ay * ay + az * az);
}

void DataContext::GetLinearAccel(const data::SamplePacket &rPacket,
                                 float &rLinAx, float &rLinAy, float &rLinAz)
{
    rLinAx = (float)rPacket.a[0] - (float)m_AccelBias.axBias
             - (float)rPacket.gv[0];
    rLinAy = (float)rPacket.a[1] - (float)m_AccelBias.ayBias
             - (float)rPacket.gv[1];
    rLinAz = (float)rPacket.a[2] - (float)m_AccelBias.azBias
             - (float)rPacket.gv[2];
}

}
