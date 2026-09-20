#pragma once
#include "BleServiceManager.hpp"
#include "LsmServiceManager.hpp"
#include "W8BandServiceManager.hpp"
#include <Arduino.h>
#include <optional>
#include <string>

/// @brief GPIO connected to the first LSM6DSV16X interrupt output.
#define WAKEUP_INT1 PIN_A3

/// @brief GPIO connected to the second LSM6DSV16X interrupt output.
#define WAKEUP_INT2 PIN_A1

/// @brief GPIO used as the inactive SPI chip-select line.
#define CS_PIN A2

namespace w8band
{
/// @brief Creates hardware services and starts the W8Band application.
class W8BandLauncher
{
public:
    /// @brief Constructs hardware managers and their shared data context.
    W8BandLauncher();

    /// @brief Method used by the user to setup w8band application.
    ///        Sets m_ErrorMessage to debug information when launch was not
    ///        correct.
    /// @return True if launch was correct, false otherwise.
    bool Initialize();

    /// @brief Returns the latest initialization error message.
    /// @return Empty string when no initialization error was recorded.
    std::string GetErrorMessage();

    /// @brief Method used to launch W8BandServiceManager and give it control.
    void LaunchW8Band();

    /// @brief Drives the FSM
    void Update();

private:
    /// @brief Description of the latest initialization error.
    std::string m_ErrorMessage;

    /// @brief Main application manager created after hardware initialization.
    std::optional<W8BandServiceManager> m_W8BandServiceManager;

    /// @brief IMU service used by the application.
    Hardware::LsmServiceManager m_Lsm;

    /// @brief BLE service used by the application.
    Hardware::BleServiceManager m_Ble;

    /// @brief Data shared by all application states.
    DataContext m_DataContext;
};
} // namespace w8band
