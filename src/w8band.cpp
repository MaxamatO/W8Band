#include "w8band.hpp"

#include <memory>

static uint16_t fifo_samples;
namespace w8band
{

W8Band *instance = nullptr;

W8Band::W8Band(TwoWire &wireBus, uint8_t i2cAddress)
    : m_Imu(&wireBus, i2cAddress),
      m_BLEService("19B10000-E8F2-537E-4F6C-D104768A1214"),
      m_IsLSMEnabledCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214",
                                   BLERead | BLEWrite),
      m_ConnectionHandle(BLE_CONN_HANDLE_INVALID),
      m_DataCharacteristic("19B10002-E8F2-537E-4F6C-D104768A1214")
{
    m_Data.clear();
    instance = this;
}

void W8Band::Init()
{
    InitLsm();
    InitBle();
}

bool W8Band::InitLsm()
{
    uint8_t status = 0;
    status |= m_Imu.begin();

    status |= m_Imu.Device_Reset();
    delay(10);

    status |= m_Imu.Enable_X();
    status |= m_Imu.Enable_G();
    status |= m_Imu.Set_X_FS(4);
    status |= m_Imu.Set_G_FS(2000);

    status |= m_Imu.Set_X_ODR(120.0f);
    status |= m_Imu.Set_G_ODR(120.0f);
    delay(50);

    status |= m_Imu.Set_SFLP_ODR(120.0f);
    status |= m_Imu.Enable_Rotation_Vector();
    status |= m_Imu.FIFO_Set_Mode(LSM6DSV16X_BYPASS_MODE);
    delay(10);

    status |= m_Imu.Set_SFLP_Batch(true, false, false);

    status |= m_Imu.FIFO_Set_X_BDR(120.0f);
    status |= m_Imu.FIFO_Set_Mode(LSM6DSV16X_STREAM_MODE);
    m_Tag = 0;

    return status == LSM6DSV16X_OK;
}

void W8Band::InitBle()
{
    Bluefruit.begin();
    Bluefruit.setTxPower(4);
    Bluefruit.setName("W8Band");

    Bluefruit.Periph.setConnectCallback(ConnectCallback);
    Bluefruit.Periph.setDisconnectCallback(DisconnectCallback);

    m_BLEService.begin();

    m_IsLSMEnabledCharacteristic.setProperties(CHR_PROPS_READ
                                               | CHR_PROPS_WRITE);
    m_IsLSMEnabledCharacteristic.setPermission(SECMODE_OPEN, SECMODE_OPEN);
    m_IsLSMEnabledCharacteristic.setFixedLen(1);
    m_IsLSMEnabledCharacteristic.begin();
    m_IsLSMEnabledCharacteristic.write8(0);
    m_IsLSMEnabledCharacteristic.setWriteCallback(WriteCallback);

    m_DataCharacteristic.setProperties(CHR_PROPS_NOTIFY);
    m_DataCharacteristic.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
    m_DataCharacteristic.setMaxLen(244);
    m_DataCharacteristic.begin();

    Bluefruit.Advertising.addFlags(
        BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
    Bluefruit.Advertising.addTxPower();
    Bluefruit.Advertising.addService(m_BLEService);
    Bluefruit.Advertising.addName();
    Bluefruit.Advertising.restartOnDisconnect(true);

    Bluefruit.Advertising.start(0);
}

void W8Band::SendBLEData()
{
    if(m_Data.empty())
        return;

    const int SAMPLES_PER_PACKET = 8;
    const int SAMPLE_SIZE = sizeof(SystemData);
    uint8_t buffer[SAMPLES_PER_PACKET * SAMPLE_SIZE];

    int totalSamples = m_Data.size();
    int samplesSent = 0;

    while(samplesSent < totalSamples)
    {
        int samplesInThisPacket
            = min(SAMPLES_PER_PACKET, totalSamples - samplesSent);
        int bytesToSend = samplesInThisPacket * SAMPLE_SIZE;

        memcpy(buffer, &m_Data[samplesSent], bytesToSend);

        m_DataCharacteristic.notify(buffer, bytesToSend);
        samplesSent += samplesInThisPacket;

        delay(5);
    }
    Serial.println(samplesSent);
}

void W8Band::ConnectCallback(uint16_t connHandle)
{
    Serial.println("Telefon połączony!");
    // Tutaj możesz np. zapalić diodę LED na płytce
}

void W8Band::DisconnectCallback(uint16_t connHandle, uint8_t reason)
{
    Serial.print("Rozłączono. Powód: ");
    Serial.println(reason);
}

// void W8Band::Update()
// {
//     // if(millis() - m_RecordingStartTime >= RECORDING_TIME)
//     // {
//     //     m_IsRecording = false;
//     //     Serial.println("Zakonczono nagrywanie.");
//     //     SendBLEData();
//     //     return;
//     // }

//     SystemData systemData;

//     if(m_Imu.FIFO_Get_Num_Samples(&fifo_samples) != LSM6DSV16X_OK)
//     {
//         Serial.println("LSM6DSV16X Sensor failed to get number of samples "
//                        "inside FIFO");
//         return;
//     }

//     if(fifo_samples > 0)
//     {
//         for(int i = 0; i < fifo_samples; i++)
//         {
//             m_Imu.FIFO_Get_Tag(&m_Tag);
//             if(m_Tag == TAG_GAME_ROTATION_VECTOR)
//             {
//                 m_Imu.FIFO_Get_Rotation_Vector(&systemData.m_Quaternions[0]);
//             } else if(m_Tag == TAG_ACCELEROMETER)
//             {
//                 m_Imu.FIFO_Get_X_Axes(systemData.m_Accelerometer);
//                 m_Data.push_back(systemData);
//             }
//         }
//     }
// }

void W8Band::WriteCallback(uint16_t conn_hdl, BLECharacteristic *chr,
                           uint8_t *data, uint16_t len)
{
    // if(data[0] == 1)
    // {
    if(instance->m_IsRecording)
        return;

    Serial.println("Ustawiam flage nagrywania na true");
    instance->m_Imu.FIFO_Set_Mode(LSM6DSV16X_BYPASS_MODE);
    delay(5);
    instance->m_Imu.FIFO_Set_Mode(LSM6DSV16X_STREAM_MODE);
    instance->m_Data.clear();
    instance->m_Data.reserve(600);
    // instance->m_RecordingStartTime = millis();
    instance->m_IsRecording = true;
    // } else
    // {
    //     // Serial.println("Ustawiam flage nagrywania na false");
    //     // instance->m_IsRecording = false;
    // }
}

void W8Band::Update()
{
    if(m_Imu.FIFO_Get_Num_Samples(&fifo_samples) != LSM6DSV16X_OK)
    {
        // Zakomentowane, żeby nie śmieciło na porcie szeregowym, gdy Python
        // nasłuchuje liczb Serial.println("LSM6DSV16X Sensor failed to get
        // number of samples inside FIFO");
        return;
    }

    if(fifo_samples > 0)
    {
        for(int i = 0; i < fifo_samples; i++)
        {
            m_Imu.FIFO_Get_Tag(&m_Tag);

            if(m_Tag == TAG_GAME_ROTATION_VECTOR)
            {
                // Aktualizujemy kwaternion w naszej "trwałej" ramce
                m_Imu.FIFO_Get_Rotation_Vector(
                    &m_CurrentFrame.m_Quaternions[0]);
            } else if(m_Tag == TAG_ACCELEROMETER)
            {
                // Aktualizujemy akcelerometr
                m_Imu.FIFO_Get_X_Axes(m_CurrentFrame.m_Accelerometer);

                // --- STREAMING NA ŻYWO (Format CSV: Qx,Qy,Qz,Qw,Ax,Ay,Az) ---
                Serial.print(m_CurrentFrame.m_Quaternions[0], 4);
                Serial.print(",");
                Serial.print(m_CurrentFrame.m_Quaternions[1], 4);
                Serial.print(",");
                Serial.print(m_CurrentFrame.m_Quaternions[2], 4);
                Serial.print(",");
                Serial.print(m_CurrentFrame.m_Quaternions[3], 4);
                Serial.print(",");
                Serial.print(m_CurrentFrame.m_Accelerometer[0]);
                Serial.print(",");
                Serial.print(m_CurrentFrame.m_Accelerometer[1]);
                Serial.print(",");
                Serial.println(
                    m_CurrentFrame.m_Accelerometer[2]); // println na końcu!
            } else
            {
                // Bardzo ważne: opróżniamy FIFO z innych tagów (np. żyroskopu,
                // temperatury), żeby nie zapchać pamięci układu scalonego.
                int32_t dummy[3];
                m_Imu.FIFO_Get_G_Axes(dummy);
            }
        }
    }
}

void W8Band::AccelerometerHandle() {}

void W8Band::QuaternionHandle() {}

} // namespace w8band