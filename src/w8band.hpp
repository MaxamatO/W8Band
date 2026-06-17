#pragma once
#include <Arduino.h>
#include <LSM6DSV16XSensor.h>
#include <Wire.h>
#include <bluefruit.h>
#include <vector>

#define IMU_I2C_ADDRESS LSM6DSV16X_I2C_ADD_L
#define TAG_GAME_ROTATION_VECTOR 0x13u
#define TAG_ACCELEROMETER 0x02u

#define WAKEUP_INT1 PIN_A3
#define WAKEUP_INT2 PIN_A1
#define CS_PIN A2

#define WU_THS 10

#define IMU_FREQ 120.0f
#define RECORDING_TIME_MS 4000

#define PRE_RECORD_SAMPLES 120

// -------------------------------------------------------------------
// Wire format – 20 bytes, little-endian, no MTU negotiation needed:
//
//   int16  Qw Qx Qy Qz   [8 B]   Q1.14 fixed-point  (×16384 → int16)
//   int16  Ax Ay Az       [6 B]   milligravity [mg]
//   uint16 seq            [2 B]   sequence counter (detect lost packets)
//   uint32 timestamp_ms   [4 B]   millis() on nRF
//
// Total: 20 bytes  →  Python struct '<4h3hHI'
// 20 samples per BLE notify (20×20 = 400 B, but stack sends up to
// floor(notify_max/20) samples; we cap at 1 to be safe with MTU=23)
// -------------------------------------------------------------------
namespace w8band
{

enum class StateMachine : uint8_t
{
    IDLE,
    BUFFERRING,
    ARMED,
    RECORDING
};

struct __attribute__((packed)) SamplePacket
{
    int16_t q[4];          // Qw Qx Qy Qz  Q1.14 8 bytes
    int16_t a[3];          // mg                 6 bytes
    uint16_t seq;          //                    2 bytes
    uint32_t timestamp_ms; //                    4 bytes
};
static_assert(sizeof(SamplePacket) == 20, "SamplePacket must be 20 bytes");

class W8Band
{
public:
    W8Band(TwoWire &wireBus, uint8_t i2cAddress = IMU_I2C_ADDRESS);
    void Init();
    void Update();

private:
    bool InitLsm();
    void InitBle();
    void SendBLEData();
    void InitWakeup();

    static void ConnectCallback(uint16_t connHandle);
    static void DisconnectCallback(uint16_t connHandle, uint8_t reason);
    static void WriteCallback(uint16_t conn_hdl, BLECharacteristic *chr,
                              uint8_t *data, uint16_t len);

    LSM6DSV16XSensor m_Imu;
    BLEService m_BLEService;
    BLECharacteristic m_ControlCharacteristic;
    BLECharacteristic m_DataCharacteristic;

    bool m_IsRecording = false;
    unsigned long m_RecordingStart = 0;
    uint8_t m_Tag = 0;
    uint16_t m_Seq = 0;

    std::vector<SamplePacket> m_Data;
    SamplePacket m_PreBuffer[PRE_RECORD_SAMPLES];
    uint16_t m_PreIndex = 0;
    uint16_t m_PreCount = 0;

    float m_LatestQuat[4] = {};    // Qw Qx Qy Qz  (float, converted on store)
    int32_t m_LatestAccel[3] = {}; // mg
    bool m_HaveFreshQuat = false;
    bool m_HaveFreshAccel = false;

    StateMachine m_CurrentState;
};

} // namespace w8band