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
}