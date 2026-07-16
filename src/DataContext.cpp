#include "DataContext.hpp"
#include "BleServiceManager.hpp"
#include "LsmServiceManager.hpp"
namespace w8band
{
DataContext::DataContext(Hardware::BleServiceManager &m_rBle,
                         Hardware::LsmServiceManager &rLsm)
    : m_rBleServiceManager(m_rBle), m_rLsmServiceManager(rLsm)
{}
}