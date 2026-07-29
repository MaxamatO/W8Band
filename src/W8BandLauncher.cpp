#include "W8BandLauncher.hpp"
#include <Wire.h>
#include <memory>

namespace w8band
{
W8BandLauncher::W8BandLauncher()
    : m_ErrorMessage(""), m_Lsm(Wire), m_DataContext{m_Ble, m_Lsm}
{}

bool W8BandLauncher::Initialize()
{
    m_ErrorMessage = "";
    pinMode(WAKEUP_INT1, INPUT_PULLDOWN);
    if(!m_Lsm.InitLsm())
    {
        m_ErrorMessage = "Lsm init failed";
        return false;
    }
    if(!m_Ble.InitBle())
    {
        m_ErrorMessage = "Ble init failed";
        return false;
    }
    m_W8BandServiceManager.emplace(m_Ble, m_Lsm, m_DataContext);
    if(!m_W8BandServiceManager.has_value())
    {
        m_ErrorMessage = "Error creating w8band service manager.";
        return false;
    }
    m_W8BandServiceManager->Init();
    m_W8BandServiceManager->AttachWakeUptInterrupt(WAKEUP_INT1);
    return true;
}

void W8BandLauncher::LaunchW8Band()
{
    if(m_W8BandServiceManager.has_value())
    {
        m_W8BandServiceManager->StartApplication();
    }
}

void W8BandLauncher::Update()
{
    if(m_W8BandServiceManager.has_value())
    {
        m_W8BandServiceManager->Update();
    }
}

std::string W8BandLauncher::GetErrorMessage() { return m_ErrorMessage; }
} // namespace w8band
