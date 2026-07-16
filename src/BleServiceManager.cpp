#include "BleServiceManager.hpp"
#include "DataContext.hpp"

namespace w8band::Hardware
{

BleServiceManager::BleServiceManager()
    : m_BLEService("19B10000-E8F2-537E-4F6C-D104768A1214"),
      m_ControlCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214",
                              BLERead | BLEWrite),
      m_DataCharacteristic("19B10002-E8F2-537E-4F6C-D104768A1214")
{}

bool BleServiceManager::InitBle()
{
    // Bluefruit.begin();
    // Bluefruit.setTxPower(4);
    // Bluefruit.setName("W8Band");
    // Bluefruit.Periph.setConnectCallback(ConnectCallback);
    // Bluefruit.Periph.setDisconnectCallback(DisconnectCallback);

    // m_BLEService.begin();

    // // Control – 1 byte R/W
    // m_ControlCharacteristic.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE);
    // m_ControlCharacteristic.setPermission(SECMODE_OPEN, SECMODE_OPEN);
    // m_ControlCharacteristic.setFixedLen(1);
    // m_ControlCharacteristic.begin();
    // m_ControlCharacteristic.write8(0);
    // m_ControlCharacteristic.setWriteCallback(WriteCallback);

    // // Data – NOTIFY, exactly 20 bytes per packet (one sample)
    // // Works with default MTU=23 (ATT overhead=3, payload=20)
    // m_DataCharacteristic.setProperties(CHR_PROPS_NOTIFY);
    // m_DataCharacteristic.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
    // m_DataCharacteristic.setFixedLen(sizeof(DataContext::SamplePacket)); //
    // 20 m_DataCharacteristic.begin();

    // Bluefruit.Advertising.addFlags(
    //     BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
    // Bluefruit.Advertising.addTxPower();
    // Bluefruit.Advertising.addService(m_BLEService);
    // Bluefruit.Advertising.addName();
    // Bluefruit.Advertising.restartOnDisconnect(true);
    // Bluefruit.Advertising.start(0);

    // Serial.println("BLE: OK");
}

void BleServiceManager::SendBLEData()
{
    // m_CurrentState = StateMachine::IDLE;
    // if(m_Data.empty())
    //     return;

    // const int total = (int)m_Data.size();
    // int sent = 0;

    // while(sent < total)
    // {
    //     int wait = 200;
    //     while(!Bluefruit.connected() && wait-- > 0)
    //         delay(10);
    //     if(!Bluefruit.connected())
    //     {
    //         Serial.println("BLE lost");
    //         break;
    //     }

    //     bool ok = m_DataCharacteristic.notify(
    //         reinterpret_cast<const uint8_t *>(&m_Data[sent]),
    //         sizeof(SamplePacket));

    //     if(ok)
    //     {
    //         sent++;
    //         if(sent % 8 == 0)
    //             delay(1);
    //     } else
    //     {
    //         delay(2);
    //     }
    // }
    // SamplePacket eof;
    // memset(&eof, 0, sizeof(eof));
    // eof.seq = 0xFFFF;
    // eof.timestamp_ms = (uint32_t)total;

    // for(int i = 0; i < 5; i++)
    // {
    //     int wait = 100;
    //     while(!Bluefruit.connected() && wait-- > 0)
    //         delay(10);
    //     if(!Bluefruit.connected())
    //         break;
    //     m_DataCharacteristic.notify(reinterpret_cast<const uint8_t *>(&eof),
    //                                 sizeof(eof));
    //     delay(20);
    // }

    // Serial.print("Sent: ");
    // Serial.print(sent);
    // Serial.print(" / ");
    // Serial.println(total);

    // m_ControlCharacteristic.write8(0);
}

void BleServiceManager::WriteCallback(uint16_t, BLECharacteristic *,
                                      uint8_t *data, uint16_t len)
{
    // if(len < 1 || data[0] != 1)
    //     return;
    // if(instance->m_IsRecording)
    //     return;

    // Serial.println("START - waiting 1s before ARMED due to wakeup");

    // instance->m_Imu.FIFO_Set_Mode(LSM6DSV16X_BYPASS_MODE);
    // delay(10);
    // instance->m_Imu.FIFO_Set_Mode(LSM6DSV16X_STREAM_MODE);
    // delay(10);

    // instance->m_PreIndex = 0;
    // instance->m_PreCount = 0;

    // instance->m_Data.clear();
    // instance->m_Data.reserve(1200);
    // instance->m_HaveFreshQuat = false;
    // instance->m_HaveFreshAccel = false;
    // instance->m_Seq = 0;

    // v_WakeUpDetected = false;
    // instance->m_CurrentState = StateMachine::BUFFERRING;
}

void BleServiceManager::ConnectCallback(uint16_t conn_hdl)
{
    Serial.println("BLE connected");

    BLEConnection *conn = Bluefruit.Connection(conn_hdl);
    if(conn)
        conn->requestMtuExchange(247);
}

void BleServiceManager::DisconnectCallback(uint16_t, uint8_t)
{ Serial.println("BLE disconnected"); }
} // namespace w8band::BleServiceManager
