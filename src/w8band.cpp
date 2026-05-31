#include "w8band.hpp"

namespace w8band
{

static W8Band *instance = nullptr;
static uint16_t fifo_samples = 0;

// Q1.14 scale: multiply float [-1,1] by 16384 and clamp to int16
static inline int16_t toQ14(float v)
{
    int32_t x = (int32_t)(v * 16384.0f);
    if(x > 32767)
        x = 32767;
    if(x < -32768)
        x = -32768;
    return (int16_t)x;
}

// -------------------------------------------------------------------
W8Band::W8Band(TwoWire &wireBus, uint8_t i2cAddress)
    : m_Imu(&wireBus, i2cAddress),
      m_BLEService("19B10000-E8F2-537E-4F6C-D104768A1214"),
      m_ControlCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214",
                              BLERead | BLEWrite),
      m_DataCharacteristic("19B10002-E8F2-537E-4F6C-D104768A1214")
{ instance = this; }

// -------------------------------------------------------------------
void W8Band::Init()
{
    Serial.begin(115200);
    InitLsm();
    // InitBle();
}

// -------------------------------------------------------------------
bool W8Band::InitLsm()
{
    uint8_t status = 0;

    status |= m_Imu.begin();
    status |= m_Imu.Device_Reset();
    delay(50);

    status |= m_Imu.Enable_X();
    status |= m_Imu.Set_X_FS(4);
    status |= m_Imu.Set_X_ODR(IMU_FREQ);

    status |= m_Imu.Enable_G();
    status |= m_Imu.Set_G_FS(2000);
    status |= m_Imu.Set_G_ODR(IMU_FREQ);
    delay(50);

    status |= m_Imu.Set_SFLP_ODR(IMU_FREQ);
    status |= m_Imu.Enable_Rotation_Vector();
    delay(20);

    status |= m_Imu.FIFO_Set_Mode(LSM6DSV16X_BYPASS_MODE);
    delay(20);
    status |= m_Imu.Set_SFLP_Batch(true, false, false);
    status |= m_Imu.FIFO_Set_X_BDR(IMU_FREQ);
    status |= m_Imu.FIFO_Set_Mode(LSM6DSV16X_STREAM_MODE);
    delay(20);

    Serial.print("IMU: ");
    Serial.println(status == LSM6DSV16X_OK ? "OK" : "ERROR");
    return (status == LSM6DSV16X_OK);
}

// -------------------------------------------------------------------
// void W8Band::InitBle()
// {
//     Bluefruit.begin();
//     Bluefruit.setTxPower(4);
//     Bluefruit.setName("W8Band");
//     Bluefruit.Periph.setConnectCallback(ConnectCallback);
//     Bluefruit.Periph.setDisconnectCallback(DisconnectCallback);

//     m_BLEService.begin();

//     // Control – 1 byte R/W
//     m_ControlCharacteristic.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE);
//     m_ControlCharacteristic.setPermission(SECMODE_OPEN, SECMODE_OPEN);
//     m_ControlCharacteristic.setFixedLen(1);
//     m_ControlCharacteristic.begin();
//     m_ControlCharacteristic.write8(0);
//     m_ControlCharacteristic.setWriteCallback(WriteCallback);

//     // Data – NOTIFY, exactly 20 bytes per packet (one sample)
//     // Works with default MTU=23 (ATT overhead=3, payload=20)
//     m_DataCharacteristic.setProperties(CHR_PROPS_NOTIFY);
//     m_DataCharacteristic.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
//     m_DataCharacteristic.setFixedLen(sizeof(SamplePacket)); // 20
//     m_DataCharacteristic.begin();

//     Bluefruit.Advertising.addFlags(
//         BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
//     Bluefruit.Advertising.addTxPower();
//     Bluefruit.Advertising.addService(m_BLEService);
//     Bluefruit.Advertising.addName();
//     Bluefruit.Advertising.restartOnDisconnect(true);
//     Bluefruit.Advertising.start(0);

//     Serial.println("BLE: OK");
// }

// -------------------------------------------------------------------
void W8Band::Update()
{
    if(m_Imu.FIFO_Get_Num_Samples(&fifo_samples) != LSM6DSV16X_OK)
        return;
    if(fifo_samples == 0)
        return;

    for(uint16_t i = 0; i < fifo_samples; i++)
    {
        if(m_Imu.FIFO_Get_Tag(&m_Tag) != LSM6DSV16X_OK)
            break;

        if(m_Tag == TAG_GAME_ROTATION_VECTOR)
        {
            // Library returns float[4]: i(X), j(Y), k(Z), r(W)
            float tmp[4];
            m_Imu.FIFO_Get_Rotation_Vector(tmp);
            m_LatestQuat[0] = tmp[3]; // Qw = r
            m_LatestQuat[1] = tmp[0]; // Qx = i
            m_LatestQuat[2] = tmp[1]; // Qy = j
            m_LatestQuat[3] = tmp[2]; // Qz = k
            m_HaveFreshQuat = true;
        } else if(m_Tag == TAG_ACCELEROMETER)
        {
            m_Imu.FIFO_Get_X_Axes(m_LatestAccel); // int32 [mg]
            m_HaveFreshAccel = true;
        } else
        {
            int32_t dummy[3];
            m_Imu.FIFO_Get_G_Axes(dummy);
            continue;
        }

        if(m_HaveFreshQuat && m_HaveFreshAccel)
        {
            SamplePacket s;
            s.q[0] = toQ14(m_LatestQuat[0]);
            s.q[1] = toQ14(m_LatestQuat[1]);
            s.q[2] = toQ14(m_LatestQuat[2]);
            s.q[3] = toQ14(m_LatestQuat[3]);
            s.a[0] = (int16_t)m_LatestAccel[0];
            s.a[1] = (int16_t)m_LatestAccel[1];
            s.a[2] = (int16_t)m_LatestAccel[2];
            s.seq = m_Seq++;
            s.timestamp_ms = millis();
            m_Data.push_back(s);
            m_HaveFreshQuat = false;
            m_HaveFreshAccel = false;
            Serial.println("Quats: ");
            Serial.print("r: ");
            Serial.println(s.q[0]);
            Serial.print("i: ");
            Serial.println(s.q[1]);
            Serial.print("j: ");
            Serial.println(s.q[2]);
            Serial.print("k: ");
            Serial.println(s.q[3]);
            Serial.println("Accel: ");
            Serial.print("x: ");
            Serial.println(s.a[0]);
            Serial.print("y: ");
            Serial.println(s.a[1]);
            Serial.print("z: ");
            Serial.println(s.a[2]);
        }
    }
}

// -------------------------------------------------------------------
// Send one sample per notify packet (20 B = default MTU payload)
// At 120Hz × 8s = ~960 samples × 20B = 19.2 kB
// At 15ms connection interval: 960 × 15ms = 14.4s transfer time
// → receiver should wait at least 15s after recording ends
// -------------------------------------------------------------------
// void W8Band::SendBLEData()
// {
//     if(m_Data.empty())
//         return;

//     const int total = (int)m_Data.size();
//     int sent = 0;

//     while(sent < total)
//     {
//         int wait = 200;
//         while(!Bluefruit.connected() && wait-- > 0)
//             delay(10);
//         if(!Bluefruit.connected())
//         {
//             Serial.println("BLE lost");
//             break;
//         }

//         bool ok = m_DataCharacteristic.notify(
//             reinterpret_cast<const uint8_t *>(&m_Data[sent]),
//             sizeof(SamplePacket));

//         if(ok)
//         {
//             sent++;
//             // Yield every 8 packets so the SoftDevice stack can breathe
//             if(sent % 8 == 0)
//                 delay(1);
//         } else
//         {
//             delay(2); // stack busy, retry same packet
//         }
//     }
//     SamplePacket eof;
//     memset(&eof, 0, sizeof(eof));
//     eof.seq = 0xFFFF;
//     eof.timestamp_ms = (uint32_t)total;

//     for(int i = 0; i < 5; i++)
//     {
//         int wait = 100;
//         while(!Bluefruit.connected() && wait-- > 0)
//             delay(10);
//         if(!Bluefruit.connected())
//             break;
//         m_DataCharacteristic.notify(reinterpret_cast<const uint8_t *>(&eof),
//                                     sizeof(eof));
//         delay(20);
//     }

//     Serial.print("Sent: ");
//     Serial.print(sent);
//     Serial.print(" / ");
//     Serial.println(total);

//     m_ControlCharacteristic.write8(0);
// }

// // -------------------------------------------------------------------
// void W8Band::WriteCallback(uint16_t, BLECharacteristic *, uint8_t *data,
//                            uint16_t len)
// {
//     if(len < 1 || data[0] != 1)
//         return;
//     if(instance->m_IsRecording)
//         return;

//     Serial.println("START");

//     instance->m_Data.clear();
//     instance->m_Data.reserve(1200);
//     instance->m_HaveFreshQuat = false;
//     instance->m_HaveFreshAccel = false;
//     instance->m_Seq = 0;

//     instance->m_Imu.FIFO_Set_Mode(LSM6DSV16X_BYPASS_MODE);
//     delay(10);
//     instance->m_Imu.FIFO_Set_Mode(LSM6DSV16X_STREAM_MODE);
//     delay(10);

//     instance->m_RecordingStart = millis();
//     instance->m_IsRecording = true;
// }

// void W8Band::ConnectCallback(uint16_t conn_hdl)
// {
//     Serial.println("BLE connected");
//     // Request larger MTU – central may or may not honour it.
//     // If it does, Bleak will see mtu_size > 23 and we could batch packets.
//     // If not, 20-byte single-sample packets still work fine.
//     BLEConnection *conn = Bluefruit.Connection(conn_hdl);
//     if(conn)
//         conn->requestMtuExchange(247);
// }

// void W8Band::DisconnectCallback(uint16_t, uint8_t)
// { Serial.println("BLE disconnected"); }

} // namespace w8band