#include "BleServiceManager.hpp"
#include "DataContext.hpp"

namespace w8band::Hardware
{
namespace
{
constexpr uint8_t PROTOCOL_VERSION = 1;

constexpr uint8_t RESULT_FLAG_VALID = 1 << 0;
constexpr uint8_t RESULT_FLAG_POWER_VALID = 1 << 1;

constexpr char DEVICE_NAME[] = "W8Band";

constexpr char SERVICE_UUID[] = "19B10000-E8F2-537E-4F6C-D104768A1214";
constexpr char CONTROL_UUID[]
    = "19B10001-E8F2-537E-4F6C-D104768A1214"; // Control point characteristic
constexpr char RESULT_UUID[]
    = "19B10002-E8F2-537E-4F6C-D104768A1214"; // ResultData characterisitc
constexpr char STATUS_UUID[]
    = "19B10003-E8F2-537E-4F6C-D104768A1214"; // Status characteristic

constexpr uint16_t CONTROL_MAX_LENGTH = 20;
constexpr uint16_t STATUS_MAX_LENGTH = 20;
constexpr uint16_t REQUESTED_ATT_MTU = 247;

void WriteUint16LittleEndian(uint8_t *pOutput, uint16_t value)
{
    pOutput[0] = static_cast<uint8_t>(value & 0xFF);
    pOutput[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
}

void WriteUint32LittleEndian(uint8_t *pOutput, uint32_t value)
{
    pOutput[0] = static_cast<uint8_t>(value & 0xFF);
    pOutput[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
    pOutput[2] = static_cast<uint8_t>((value >> 16) & 0xFF);
    pOutput[3] = static_cast<uint8_t>((value >> 24) & 0xFF);
}

} // namespace

BleServiceManager *BleServiceManager::s_pInstance = nullptr;

BleServiceManager::BleServiceManager()
    : m_BLEService(SERVICE_UUID), m_ControlCharacteristic(CONTROL_UUID),
      m_StatusCharacteristic(STATUS_UUID), m_DataCharacteristic(RESULT_UUID)
{ s_pInstance = this; }

bool BleServiceManager::InitBle()
{
    // This must be configured before Bluefruit.begin(). It allows an ATT MTU
    // up to 247 while the protocol remains compatible with the default MTU 23.
    Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
    if(!Bluefruit.begin(1, 0))
    {
        Serial.println("BLE: stack initialization failed");
        return false;
    }

    Bluefruit.autoConnLed(true);
    if(!Bluefruit.setTxPower(4))
    {
        Serial.println("BLE: TX power configuration failed");
        return false;
    }
    Bluefruit.setName(DEVICE_NAME);
    Bluefruit.Periph.setConnectCallback(ConnectCallback);
    Bluefruit.Periph.setDisconnectCallback(DisconnectCallback);

    // A service must be started before its characteristics. Bluefruit assigns
    // every subsequently started characteristic to the last started service.
    if(m_BLEService.begin() != ERROR_NONE)
    {
        Serial.println("BLE: service initialization failed");
        return false;
    }

    // Commands are written by the phone. Indications will carry command
    // responses once the application protocol is implemented.
    m_ControlCharacteristic.setProperties(CHR_PROPS_WRITE | CHR_PROPS_INDICATE);
    // Bluefruit derives the CCCD write permission from the value's read
    // permission. It must therefore be open for a client to subscribe to
    // indications. The missing CHR_PROPS_READ still prevents value reads.
    m_ControlCharacteristic.setPermission(SECMODE_OPEN, SECMODE_OPEN);
    m_ControlCharacteristic.setMaxLen(CONTROL_MAX_LENGTH);
    const err_t controlError = m_ControlCharacteristic.begin();
    if(controlError != ERROR_NONE)
    {
        Serial.printf("BLE: control characteristic init failed: %lu\n",
                      static_cast<unsigned long>(controlError));
        return false;
    }
    m_ControlCharacteristic.setWriteCallback(WriteCallback);

    // Status is readable after reconnect and can also report state changes.
    m_StatusCharacteristic.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
    m_StatusCharacteristic.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
    m_StatusCharacteristic.setMaxLen(STATUS_MAX_LENGTH);
    if(m_StatusCharacteristic.begin() != ERROR_NONE)
    {
        Serial.println("BLE: status characteristic initialization failed");
        return false;
    }
    if(m_StatusCharacteristic.write8(
           static_cast<uint8_t>(BleTypes::BleDeviceStatus::Initializing))
       != sizeof(uint8_t))
    {
        Serial.println("BLE: initial status write failed");
        return false;
    }

    // Result payloads are notifications. The actual chunk size will later be
    // selected from the negotiated MTU and must not assume 244-byte payloads.
    m_DataCharacteristic.setProperties(CHR_PROPS_NOTIFY);
    m_DataCharacteristic.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
    m_DataCharacteristic.setMaxLen(RESULT_MAX_LENGTH);
    if(m_DataCharacteristic.begin() != ERROR_NONE)
    {
        Serial.println("BLE: result characteristic initialization failed");
        return false;
    }

    Bluefruit.Advertising.clearData();
    Bluefruit.ScanResponse.clearData();
    if(!Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE)
       || !Bluefruit.Advertising.addTxPower()
       || !Bluefruit.Advertising.addService(m_BLEService)
       || !Bluefruit.ScanResponse.addName())
    {
        Serial.println("BLE: advertising payload configuration failed");
        return false;
    }

    Bluefruit.Advertising.restartOnDisconnect(true);
    Bluefruit.Advertising.setInterval(32, 244); // 20 ms, then 152.5 ms.
    Bluefruit.Advertising.setFastTimeout(30);
    if(!Bluefruit.Advertising.start(0))
    {
        Serial.println("BLE: advertising start failed");
        return false;
    }

    Serial.println("BLE: advertising as W8Band");
    return true;
}

void BleServiceManager::HandleWrite(const uint8_t *pData, uint16_t length)
{
    BleTypes::BleCommand command{};
    if(length == 1)
    {
        command.command = static_cast<BleTypes::BleCommandType>(pData[0]);
    }

    const uint8_t writeIndex
        = m_CommandWriteIndex.load(std::memory_order_relaxed);
    const uint8_t nextWriteIndex
        = static_cast<uint8_t>((writeIndex + 1) % COMMAND_QUEUE_STORAGE_SIZE);

    if(nextWriteIndex == m_CommandReadIndex.load(std::memory_order_acquire))
    {
        Serial.println("BLE: command queue full");
        return;
    }

    m_CommandQueue[writeIndex] = command;
    m_CommandWriteIndex.store(nextWriteIndex, std::memory_order_release);
}

bool BleServiceManager::SendCommandResponse(BleTypes::BleCommandType command,
                                            BleTypes::CommandResult result)
{
    const BleTypes::CommandResponse response{static_cast<uint8_t>(command),
                                             static_cast<uint8_t>(result)};

    if(!Bluefruit.connected() || !m_ControlCharacteristic.indicateEnabled())
    {
        Serial.println("BLE: command response indication is not enabled");
        return false;
    }

    return m_ControlCharacteristic.indicate(&response, sizeof(response));
}

bool BleServiceManager::SetStatus(BleTypes::BleDeviceStatus status)
{
    const uint8_t value = static_cast<uint8_t>(status);
    if(m_StatusCharacteristic.write8(value) != sizeof(value))
    {
        Serial.println("BLE: status update failed");
        return false;
    }

    if(Bluefruit.connected() && m_StatusCharacteristic.notifyEnabled())
    {
        if(!m_StatusCharacteristic.notify8(value))
        {
            Serial.println("BLE: status notification failed");
        }
    }

    return true;
}

bool BleServiceManager::TryPopCommand(BleTypes::BleCommand &rCommand)
{
    const uint8_t readIndex = m_CommandReadIndex.load(std::memory_order_relaxed);
    if(readIndex == m_CommandWriteIndex.load(std::memory_order_acquire))
    {
        return false;
    }

    rCommand = m_CommandQueue[readIndex];
    m_CommandReadIndex.store(
        static_cast<uint8_t>((readIndex + 1) % COMMAND_QUEUE_STORAGE_SIZE),
        std::memory_order_release);
    return true;
}

bool BleServiceManager::QueueResult(const Motion::ProcessingResult &rResult)
{
    if(m_ResultPending)
    {
        return false;
    }

    // All data have been copied into m_PendingResultData, so it doesn't get
    // overwritten, and can be sent on every Update tick independently
    m_TxBuffer = {};
    m_ResultPending = true;
    m_ResultOffset = 0;
    m_PendingResultId++;
    m_PendingResultData = rResult;
    m_TxPhase = BleTypes::ResultTxPhase::Begin;
    return true;
}

void BleServiceManager::SendBLEData()
{
    if(!m_ResultPending || !Bluefruit.connected()
       || !m_DataCharacteristic.notifyEnabled())
    {
        return;
    }

    using namespace w8band::Hardware::BleTypes;
    switch(m_TxPhase)
    {
    case ResultTxPhase::Idle: break;
    case ResultTxPhase::Begin:
        if(SendResultBegin())
        {
            if(!m_PendingResultData.valid)
            {
                m_TxPhase = ResultTxPhase::End;
            } else
            {
                m_TxPhase = ResultTxPhase::Timing;
            }
        }
        break;
    case ResultTxPhase::Timing:
        if(SendResultTiming())
        {
            m_TxPhase = ResultTxPhase::Motion;
        }
        break;
    case ResultTxPhase::Motion:
        if(SendResultMotionMetrics())
        {
            m_TxPhase = ResultTxPhase::Power;
        }
        break;
    case ResultTxPhase::Power:
        if(!m_PendingResultData.powerValid)
        {
            m_TxPhase = ResultTxPhase::Trajectory;
            break;
        }
        if(SendResultPower())
        {
            m_TxPhase = ResultTxPhase::Trajectory;
        }
        break;
    case ResultTxPhase::Trajectory:
        if(m_ResultOffset >= m_PendingResultData.trajectoryPointCount)
        {
            m_TxPhase = ResultTxPhase::End;
            break;
        }
        if(SendResultTrajectory())
        {
            if(m_ResultOffset >= m_PendingResultData.trajectoryPointCount)
            {
                m_TxPhase = ResultTxPhase::End;
            }
        }
        break;
    case ResultTxPhase::End:
        if(SendResultEnd())
        {
            m_ResultPending = false;
            m_ResultOffset = 0;
            m_TxPhase = ResultTxPhase::Idle;
            Serial.println("Transmission completed.");
        }
        break;
    }
}

bool BleServiceManager::SendResultBegin()
{
    // Build 14 byte frame for Begin Transmition
    uint8_t frame[14]{};

    frame[0] = PROTOCOL_VERSION;
    frame[1] = static_cast<uint8_t>(BleTypes::ResultMessageType::Begin);
    WriteUint16LittleEndian(&frame[2], m_PendingResultId);
    frame[4] = static_cast<uint8_t>(m_PendingResultData.status);
    frame[5] = (m_PendingResultData.valid ? RESULT_FLAG_VALID : 0)
               | (m_PendingResultData.powerValid ? RESULT_FLAG_POWER_VALID : 0);
    WriteUint16LittleEndian(&frame[6], m_PendingResultData.trajectoryPointCount);
    WriteUint16LittleEndian(&frame[8], m_PendingResultData.rawSampleCount);
    WriteUint16LittleEndian(&frame[10], m_PendingResultData.turnaroundIndex);
    WriteUint16LittleEndian(&frame[12],
                            m_PendingResultData.turnaroundTrajectoryIndex);

    return Bluefruit.connected() && m_DataCharacteristic.notifyEnabled()
           && m_DataCharacteristic.notify(frame, sizeof(frame));
}

bool BleServiceManager::SendResultTiming()
{
    uint8_t frame[16]{};

    frame[0] = PROTOCOL_VERSION;
    frame[1] = static_cast<uint8_t>(BleTypes::ResultMessageType::Timing);
    WriteUint16LittleEndian(&frame[2], m_PendingResultId);
    WriteUint32LittleEndian(&frame[4], m_PendingResultData.durationMs);
    WriteUint32LittleEndian(&frame[8], m_PendingResultData.eccentricDurationMs);
    WriteUint32LittleEndian(&frame[12],
                            m_PendingResultData.concentricDurationMs);

    return Bluefruit.connected() && m_DataCharacteristic.notifyEnabled()
           && m_DataCharacteristic.notify(frame, sizeof(frame));
}

bool BleServiceManager::SendResultMotionMetrics()
{
    uint8_t frame[12]{};

    frame[0] = PROTOCOL_VERSION;
    frame[1] = static_cast<uint8_t>(BleTypes::ResultMessageType::Motion);
    WriteUint16LittleEndian(&frame[2], m_PendingResultId);
    WriteUint16LittleEndian(&frame[4], m_PendingResultData.verticalRomMm);
    WriteUint16LittleEndian(&frame[6], m_PendingResultData.maxVelocityMmPerSec);
    WriteUint16LittleEndian(&frame[8],
                            m_PendingResultData.meanConcentricVelocityMmPerSec);
    WriteUint16LittleEndian(&frame[10],
                            m_PendingResultData.maxWorldGravityStdMg);

    return Bluefruit.connected() && m_DataCharacteristic.notifyEnabled()
           && m_DataCharacteristic.notify(frame, sizeof(frame));
}

bool BleServiceManager::SendResultPower()
{
    uint8_t frame[8]{};

    frame[0] = PROTOCOL_VERSION;
    frame[1] = static_cast<uint8_t>(BleTypes::ResultMessageType::Motion);
    WriteUint16LittleEndian(&frame[2], m_PendingResultId);
    WriteUint16LittleEndian(&frame[4], m_PendingResultData.maxPowerMilliwatts);
    WriteUint16LittleEndian(&frame[6],
                            m_PendingResultData.meanConcentricPowerMilliwatts);

    return Bluefruit.connected() && m_DataCharacteristic.notifyEnabled()
           && m_DataCharacteristic.notify(frame, sizeof(frame));
}

bool BleServiceManager::SendResultTrajectory()
{
    constexpr uint16_t HEADER_SIZE = 5; // in byes
    constexpr uint16_t POINT_SIZE = 4;  // in bytes

    const uint16_t maxPayload = min<uint16_t>(
        GetNotificationPayloadSize(), m_TxBuffer.size()); // mtu in bytes

    if(maxPayload < HEADER_SIZE + POINT_SIZE)
    {
        return false;
    }

    const uint16_t maxPointsInChunk = (maxPayload - HEADER_SIZE) / POINT_SIZE;
    const uint16_t remainingPoints
        = m_PendingResultData.trajectoryPointCount - m_ResultOffset;

    const uint16_t pointsToSend = min(maxPointsInChunk, remainingPoints);

    uint16_t writeOffset = 0;
    m_TxBuffer[writeOffset++] = PROTOCOL_VERSION;
    m_TxBuffer[writeOffset++]
        = static_cast<uint8_t>(BleTypes::ResultMessageType::Trajectory);
    WriteUint16LittleEndian(&m_TxBuffer[writeOffset], m_PendingResultId);
    writeOffset += sizeof(uint16_t);

    m_TxBuffer[writeOffset++] = static_cast<uint8_t>(m_ResultOffset);

    for(uint16_t i = 0; i < pointsToSend; i++)
    {
        const Motion::TrajectoryPoint &point
            = m_PendingResultData.trajectory[m_ResultOffset + i];

        WriteUint16LittleEndian(&m_TxBuffer[writeOffset], point.horizontalMm);
        writeOffset += sizeof(uint16_t);
        WriteUint16LittleEndian(&m_TxBuffer[writeOffset], point.verticalMm);
        writeOffset += sizeof(uint16_t);
    }

    if(!m_DataCharacteristic.notify(m_TxBuffer.data(), writeOffset))
    {
        return false;
    }

    m_ResultOffset += pointsToSend;
    return true;
}

bool BleServiceManager::SendResultEnd()
{
    uint8_t frame[6]{};

    frame[0] = PROTOCOL_VERSION;
    frame[1] = static_cast<uint8_t>(BleTypes::ResultMessageType::End);
    WriteUint16LittleEndian(&frame[2], m_PendingResultId);
    WriteUint16LittleEndian(&frame[4], m_ResultOffset);

    return m_DataCharacteristic.notify(frame, sizeof(frame));
}

uint16_t BleServiceManager::GetNotificationPayloadSize()
{
    BLEConnection *pConnection = Bluefruit.Connection(Bluefruit.connHandle());

    if(pConnection == nullptr)
    {
        return 20;
    }

    const uint16_t mtu = pConnection->getMtu();
    if(mtu <= 3)
    {
        return 20;
    }

    return std::min<uint16_t>(mtu - 3, RESULT_MAX_LENGTH);
}

void BleServiceManager::WriteCallback(uint16_t connHandle,
                                      BLECharacteristic *pCharacteristic,
                                      uint8_t *pData, uint16_t len)
{
    (void)connHandle;
    (void)pCharacteristic;

    if(s_pInstance != nullptr)
    {
        s_pInstance->HandleWrite(pData, len);
    }
}

void BleServiceManager::ConnectCallback(uint16_t conn_hdl)
{
    Serial.println("BLE connected");

    BLEConnection *conn = Bluefruit.Connection(conn_hdl);
    if(conn)
    {
        conn->requestMtuExchange(REQUESTED_ATT_MTU);
        conn->requestDataLengthUpdate();
        conn->requestPHY();
    }
}

void BleServiceManager::DisconnectCallback(uint16_t, uint8_t)
{
    Serial.println("BLE disconnected");
    s_pInstance->m_TxPhase = BleTypes::ResultTxPhase::Idle;
}
} // namespace w8band::BleServiceManager
