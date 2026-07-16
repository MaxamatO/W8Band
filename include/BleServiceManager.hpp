#pragma once
#include <Arduino.h>
#include <bluefruit.h>
#include <cstdint>
namespace w8band::Hardware
{

class BleServiceManager
{
public:
    /// @brief Default constructor for launcher.
    BleServiceManager();

    /// @brief Initialize BLE with default values provided by creator.
    /// @return True if initialization was successful, false otherwise.
    bool InitBle();
    void SendBLEData();

    static void ConnectCallback(uint16_t connHandle);
    static void DisconnectCallback(uint16_t connHandle, uint8_t reason);
    static void WriteCallback(uint16_t conn_hdl, BLECharacteristic *chr,
                              uint8_t *data, uint16_t len);

    BLEService m_BLEService;
    BLECharacteristic m_ControlCharacteristic;
    BLECharacteristic m_DataCharacteristic;
};

} // namespace w8band::BleServiceManager
