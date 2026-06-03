#pragma once
#include <Arduino.h>
#include <LSM6DSV16XSensor.h>
#include <Wire.h>
#include <bluefruit.h>
#include <vector>

#define IMU_I2C_ADDRESS LSM6DSV16X_I2C_ADD_L
#define TAG_GAME_ROTATION_VECTOR 0x13u
#define TAG_ACCELEROMETER 0x02u

#define IMU_FREQ 120.0f

// ── Piny przerwań
// INT1 → Significant Motion (START nagrywania)
// INT2 → Sleep/Inactivity    (STOP
#define PIN_INT1 PIN_A2
#define PIN_INT2 PIN_A3

// ── Inactivity ───────────────────────────────────────────────────────────────
// Próg wake-up / inactivity [LSB], 1 LSB = 31.25 mg (przy ±4 g, HP mode)
// 3 LSB ≈ 94 mg — sztanga musi być naprawdę nieruchoma
#define INACT_THRESHOLD_LSB 3

// Czas bezruchu zanim uzna koniec serii: SLEEP_DUR × (512 / ODR_XL)
// SLEEP_DUR=4 → 4 × 512/120 ≈ 17 s  (zwiększ jeśli kończy za wcześnie)
#define SLEEP_DUR_VAL 1

// ── Protokół BLE ─────────────────────────────────────────────────────────────
// Wire format – 20 bytes little-endian:
//   int16  Qw Qx Qy Qz  [8 B]  Q1.14
//   int16  Ax Ay Az      [6 B]  mg
//   uint16 seq           [2 B]
//   uint32 timestamp_ms  [4 B]
namespace w8band
{

struct __attribute__((packed)) SamplePacket
{
    int16_t q[4]; // Qw Qx Qy Qz  Q1.14
    int16_t a[3]; // mg
    uint16_t seq;
    uint32_t timestamp_ms;
};
static_assert(sizeof(SamplePacket) == 20, "SamplePacket must be 20 bytes");

// Stan maszyny stanów urządzenia
enum class State : uint8_t
{
    IDLE,         // czeka na komendę BLE 0x01
    CALIBRATING,  // IMU calibruje się, czeka na SMD
    RECORDING,    // zbiera próbki
    TRANSFERRING, // wysyła dane przez BLE
};

class W8Band
{
public:
    W8Band(TwoWire &wireBus, uint8_t i2cAddress = IMU_I2C_ADDRESS);
    void Init();
    void Update();

    // Wywoływane z ISR — volatile flag, obsługa w Update()
    static void IsrInt1(); // SMD  → START nagrywania
    static void IsrInt2(); // SLEEP → STOP nagrywania

private:
    bool InitLsm();
    void InitBle();
    void ConfigureWakeInact();
    void StartCalibration();
    void StartRecording();
    void StopRecording();
    void SendBLEData();

    static void ConnectCallback(uint16_t connHandle);
    static void DisconnectCallback(uint16_t connHandle, uint8_t reason);
    static void WriteCallback(uint16_t conn_hdl, BLECharacteristic *chr,
                              uint8_t *data, uint16_t len);

    LSM6DSV16XSensor m_Imu;
    BLEService m_BLEService;
    BLECharacteristic m_ControlCharacteristic;
    BLECharacteristic m_DataCharacteristic;

    State m_State = State::IDLE;
    uint16_t m_Seq = 0;
    uint8_t m_Tag = 0;

    std::vector<SamplePacket> m_Data;

    float m_LatestQuat[4] = {};
    int32_t m_LatestAccel[3] = {};
    bool m_HaveFreshQuat = false;
    bool m_HaveFreshAccel = false;

    // Flagi ustawiane przez ISR, czyszczone w Update()
    static volatile bool s_SmdFired;
    static volatile bool s_InactFired;
};

} // namespace w8band