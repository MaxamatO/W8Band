#pragma once
#include "BleDataTypes.hpp"
#include "motion-processor/MotionProcessor.hpp"
#include <Arduino.h>
#include <array>
#include <atomic>
#include <bluefruit.h>
#include <cstdint>
namespace w8band::Hardware
{
static_assert(sizeof(BleTypes::CommandResponse) == 2,
              "CommandResponse format must be exactly two bytes");

/// @brief Class responsible for handling all BLE related functionalities:
/// advertising, handling connections, sending/recieving data
class BleServiceManager
{
public:
    /// @brief Default constructor for launcher.
    BleServiceManager();

    /// @brief Initialize BLE with default values provided by creator.
    /// @return True if initialization was successful, false otherwise.
    bool InitBle();

    /// @brief Method called for sendling BLE Data.
    void SendBLEData();

    /// @brief Removes the oldest command from the receive queue.
    /// @param[out] rCommand Receives the removed command.
    /// @return True when a command was returned, false when the queue is empty.
    bool TryPopCommand(BleTypes::BleCommand &rCommand);

    /// @brief Sends an application-level response through Control indications.
    /// @param[in] command Command value copied into the response.
    /// @param[in] result Result of handling the command.
    /// @return True when a subscribed client confirmed the indication.
    bool SendCommandResponse(BleTypes::BleCommandType command,
                             BleTypes::CommandResult result);

    /// @brief Updates the readable status and notifies a subscribed client.
    /// @param[in] status New device status.
    /// @return True when the GATT value was updated successfully.
    bool SetStatus(BleTypes::BleDeviceStatus status);

    /// @brief Copies a processing result into the BLE transmission buffer.
    /// @param[in] rResult Complete result of one repetition.
    /// @return True when the result was queued, false when another result is
    /// already waiting for transmission.
    bool QueueResult(const Motion::ProcessingResult &rResult);

    /// @brief Custom GATT service exposed by the device.
    BLEService m_BLEService;

    /// @brief Receives commands and sends command responses.
    BLECharacteristic m_ControlCharacteristic;

    /// @brief Stores and notifies the current device status.
    BLECharacteristic m_StatusCharacteristic;

    /// @brief Sends processed result frames and trajectory chunks.
    BLECharacteristic m_DataCharacteristic;

private:
    /// @brief Active manager instance used by static BLE callbacks.
    static BleServiceManager *s_pInstance;

    /// @brief Called on BLE connect
    /// @param[in] connHandle Handle of the connected client.
    static void ConnectCallback(uint16_t connHandle);

    /// @brief Called on BLE disconnect
    /// @param[in] connHandle Handle of the disconnected client.
    /// @param[in] reason BLE disconnect reason.
    static void DisconnectCallback(uint16_t connHandle, uint8_t reason);

    /// @brief Called when a client writes to the Control Point characteristic.
    /// @param[in] connHandle Handle of the client that performed the write.
    /// @param[in] pCharacteristic Characteristic that received the data.
    /// @param[in] pData Pointer to the received bytes.
    /// @param[in] len Number of received bytes.
    static void
    WriteCallback(uint16_t connHandle, BLECharacteristic *pCharacteristic,
                  uint8_t *pData, uint16_t len);

    /// @brief Maximum ATT notification payload supported by the local buffer.
    static constexpr size_t RESULT_MAX_LENGTH = 244;

    /// @brief Decodes and queues bytes received through the Control Point.
    /// @param[in] pData Pointer to the received bytes.
    /// @param[in] length Number of received bytes.
    void HandleWrite(const uint8_t *pData, uint16_t length);

    /// @brief Calculates the usable notification payload for the connection.
    /// @return Payload size in bytes after subtracting the ATT header.
    uint16_t GetNotificationPayloadSize();

    // BLE phase sending handlers

    /// @brief Sends Begin frame with corresponding bytes:
    /*
        0       - protocol version
        1       - messageType = Begin
        2:3     - resultId (unique for each rep)
        4       - processingStatus
        5       - flags (powerValid, Valid)
        6:7     - trajectoryPointCount (should be 100)
        8:9     - rawSampleCount (amount of raw samples that were obtained)
        10:11   - turnaroundIndex (from raw samples array)
        12:13   - turnaroundTrajectoryIndex (index inside final array of
       trajectioryPointCount points)

    */
    /// @return True if notifies correctly, false otherwise
    bool SendResultBegin();

    /// @brief Sends timing frame with corresponding bytes:
    /*
        0       - protocol version
        1       - messageType = Timing
        2:3     - resultId (unique for each rep)
        4:7     - durationMs
        8:11     - eccentricDurationMs
        12:15    - concentricDurationMs
    */
    /// @return True if notifies correctly, false otherwise
    bool SendResultTiming();

    /// @brief Sends motion metrics frame with corresponding bytes:
    /*
        0       - protocol version
        1       - messageType = Motion
        2:3     - resultId (unique for each rep)
        4:5     - verticalRomMm
        6:7     - maxVelocityMmPerSec
        8:9     - meanConcentricVelocityMmPerSec
        10:11   - maxWorldGravityStdMg
    */
    /// @return True if notifies correctly, false otherwise
    bool SendResultMotionMetrics();

    /// @brief Sends power metrics frame with corresponding bytes:
    /*
        0       - protocol version
        1       - messageType = Power
        2:3     - resultId (unique for each rep)
        4:5     - maxPowerMilliwatts
        6:7     - meanConventricPowerMilliwats
    */
    /// @return True if notification was queued, false otherwise.
    bool SendResultPower();

    /// @brief Stream data dynamically, dependant on negotiated MTU size
    /*
        0       - protocol version
        1       - messageType = Trajectory
        2:3     - resultId
        4       - startIndex
        5...    - chunks containing 4B of data (2B horizontalMm, 2B verticalMm)
    */
    /// @return True if notifies correctly, false otherwise
    bool SendResultTrajectory();

    /// @brief Send end frame to the device
    /*
        0       - protocol version
        1       - messageType = End
        2:3     - resultId
        4:5     - number of sent points in Trajectory frames
    */
    /// @return True if notifies correctly, false otherwise
    bool SendResultEnd();

    /// @brief Internal ring-buffer size, including one unused control slot.
    static constexpr uint8_t COMMAND_QUEUE_STORAGE_SIZE = 5;

    /// @brief Ring buffer containing commands received from BLE callbacks.
    std::array<BleTypes::BleCommand, COMMAND_QUEUE_STORAGE_SIZE> m_CommandQueue{};

    /// @brief Index at which the BLE callback writes the next command.
    std::atomic<uint8_t> m_CommandWriteIndex{0};

    /// @brief Index from which the application reads the next command.
    std::atomic<uint8_t> m_CommandReadIndex{0};

    /// @brief Copy of the result currently waiting for transmission.
    Motion::ProcessingResult m_PendingResultData{};

    /// @brief Index of the next trajectory point to send.
    uint16_t m_ResultOffset = 0;

    /// @brief True while a result is waiting or being transmitted.
    bool m_ResultPending = false;

    /// @brief Identifier assigned to the current result.
    uint16_t m_PendingResultId = 0;

    /// @brief Reusable buffer for a trajectory notification.
    std::array<uint8_t, RESULT_MAX_LENGTH> m_TxBuffer{};

    /// @brief Current phase of the result transmission.
    BleTypes::ResultTxPhase m_TxPhase;
};

} // namespace w8band::Hardware
