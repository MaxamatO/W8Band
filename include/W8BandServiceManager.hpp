#pragma once
#include "BleServiceManager.hpp"
#include "BufferringState.hpp"
#include "CalibrationState.hpp"
#include "DataContext.hpp"
#include "IdleState.hpp"
#include "LsmServiceManager.hpp"
#include "ProcessingState.hpp"
#include "RecordingState.hpp"
#include <Arduino.h>
#include <memory>
#include <vector>

/// @brief Configured recording duration in milliseconds.
#define RECORDING_TIME_MS 4000

namespace w8band
{

/// @brief Coordinates BLE commands, sensor interrupts and the application FSM.
class W8BandServiceManager
{
public:
    /// @brief Constructs the main application service manager.
    /// @param[in,out] rBleSM BLE service used for commands and results.
    /// @param[in,out] rLsmSM IMU service used by acquisition states.
    /// @param[in,out] rDataContext Data shared by all states.
    W8BandServiceManager(Hardware::BleServiceManager &rBleSM,
                         Hardware::LsmServiceManager &rLsmSM,
                         DataContext &rDataContext);

    /// @brief Registers application states, starts the FSM and publishes Idle.
    void Init();

    /// @brief Application start hook. Currently performs no extra work.
    void StartApplication();

    /// @brief Used to drive the FSM, called once per loop()
    void Update();

    /// @brief Attaches the lift-off interrupt handler to a GPIO.
    /// @param[in] interruptPin GPIO connected to the IMU interrupt output.
    void AttachWakeUptInterrupt(uint16_t interruptPin);

private:
    /// @brief True while manager-level recording is active.
    bool m_IsRecording = false;

    /// @brief Manager-level recording start time in milliseconds.
    unsigned long m_RecordingStart = 0;

    /// @brief Sequence number reserved for transmitted data.
    uint16_t m_Seq = 0;

    /// @brief Handles one command removed from the BLE command queue.
    /// @param[in] rCommand Command received from the phone.
    void HandleBleCommand(const Hardware::BleTypes::BleCommand &rCommand);

    /// @brief Moves a finished processing result to the BLE manager.
    void HandleBleSendResultData();

    /// @brief Publishes the current FSM state when it changes.
    void PublishFsmStatus();

    /// @brief Records that the IMU wake-up interrupt occurred.
    static void WakeUpISR1();

    /// @brief Data shared by the application states.
    DataContext &m_rDataContext;

    /// @brief BLE service used to receive commands and send status or results.
    Hardware::BleServiceManager &m_rBleServiceManager;

    /// @brief IMU service used to obtain synchronized sensor samples.
    Hardware::LsmServiceManager &m_rLsmServiceManager;

    /// @brief State machine controlling one measurement cycle.
    StateMachine::W8BandFsm m_Fsm;

    /// @brief Last FSM state published through the Status characteristic.
    StateMachine::StateId m_LastPublishedState
        = StateMachine::StateId::IdleState;
};

} // namespace w8band
