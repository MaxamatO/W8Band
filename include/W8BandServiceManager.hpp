#pragma once
#include "ArmedState.hpp"
#include "BleServiceManager.hpp"
#include "BufferringState.hpp"
#include "CalibrationState.hpp"
#include "DataContext.hpp"
#include "IdleState.hpp"
#include "LsmServiceManager.hpp"
#include <Arduino.h>
#include <memory>
#include <vector>

#define RECORDING_TIME_MS 4000

namespace w8band
{

class W8BandServiceManager
{
public:
    W8BandServiceManager(Hardware::BleServiceManager &rBleSM,
                         Hardware::LsmServiceManager &rLsmSM,
                         DataContext &rDataContext);

    /// @brief Sets up pin modes, interrupts, serial and the FSM
    void Init();

    /// @brief Starts the application with default state and values.
    void StartApplication();

    /// @brief Used to drive the FSM, called once per loop()
    void Update();

    /// @brief Requests FSM transition to provided rStateId state
    /// @param stateId State to which transition will occurr.
    void RequestStateChange(StateMachine::StateId stateId);

    void AttachWakeUptInterrupt(uint16_t interruptPin);

private:
    bool m_IsRecording = false;
    unsigned long m_RecordingStart = 0;
    uint16_t m_Seq = 0;

    static void WakeUpISR1();

    DataContext &m_rDataContext;
    Hardware::BleServiceManager &m_rBleServiceManager;
    Hardware::LsmServiceManager &m_rLsmServiceManager;

    /// FSM
    StateMachine::W8BandFsm m_Fsm;
};

} // namespace w8band