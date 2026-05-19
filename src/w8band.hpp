#pragma once

#include <Arduino.h>
#include <LSM6DSV16XSensor.h>
#include <Wire.h>
#include <bluefruit.h>
#include <memory>
#include <vector>

#define INT1_pin D3
#define IMU_I2C_ADDRESS LSM6DSV16X_I2C_ADD_L

#define TAG_GAME_ROTATION_VECTOR 0x13u
#define TAG_ACCELEROMETER 0x02u

#define ALGO_FREQ 120U
#define ALGO_PERIOD (1000U / ALGO_FREQ)

#define RECORDING_TIME 8000

namespace w8band
{

struct SystemData
{
    float m_Quaternions[4];
    int32_t m_Accelerometer[3];
};

class W8Band
{
public:
    W8Band(TwoWire &wireBus, uint8_t i2cAddress = IMU_I2C_ADDRESS);
    void Update();
    void Init();

    bool IsRecording() { return m_IsRecording; }
    void SetRecording(bool state) { m_IsRecording = state; }

private:
    bool InitLsm();
    void InitBle();
    void InitTimer();

    void QuaternionHandle();
    void AccelerometerHandle();
    void SendBLEData();

    static void ConnectCallback(uint16_t connHandle);
    static void DisconnectCallback(uint16_t connHandle, uint8_t reason);
    static void WriteCallback(uint16_t conn_hdl, BLECharacteristic *chr,
                              uint8_t *data, uint16_t len);
    static void TimerHandler1();

    LSM6DSV16XSensor m_Imu;

    BLEService m_BLEService;
    BLECharacteristic m_IsLSMEnabledCharacteristic;
    BLECharacteristic m_DataCharacteristic;

    uint16_t m_ConnectionHandle;

    unsigned long m_RecordingStartTime;

    std::vector<SystemData> m_Data;

    SystemData m_CurrentFrame;

    uint8_t m_Tag;

    bool m_IsRecording;
};

}