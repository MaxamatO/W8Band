#pragma once
#include "BleServiceManager.hpp"
#include "LsmServiceManager.hpp"
#include "W8BandServiceManager.hpp"
#include <Arduino.h>
#include <optional>
#include <string>
#define WAKEUP_INT1 PIN_A3
#define WAKEUP_INT2 PIN_A1
#define CS_PIN A2
namespace w8band
{
class W8BandLauncher
{
public:
    W8BandLauncher();

    /// @brief Method used by the user to setup w8band application.
    ///        Sets m_ErrorMessage to debug information when launch was not
    ///        correct.
    /// @return True if launch was correct, false otherwise.
    bool Initialize();
    std::string GetErrorMessage();

    /// @brief Method used to launch W8BandServiceManager and give it control.
    void LaunchW8Band();

    /// @brief Drives the FSM
    void Update();

private:
    std::string m_ErrorMessage;
    std::optional<W8BandServiceManager> m_W8BandServiceManager;
    Hardware::LsmServiceManager m_Lsm;
    Hardware::BleServiceManager m_Ble;
    DataContext m_DataContext;
};
}