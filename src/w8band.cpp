#include "w8band.hpp"

namespace w8band
{

// ── Statyczne składowe ───────────────────────────────────────────────────────
static W8Band *instance = nullptr;
static uint16_t fifo_samples = 0;

volatile bool W8Band::s_SmdFired = false;
volatile bool W8Band::s_InactFired = false;

// ── ISR ──────────────────────────────────────────────────────────────────────
void W8Band::IsrInt1() { s_SmdFired = true; }
void W8Band::IsrInt2() { s_InactFired = true; }

// ── Pomocnicza: Q1.14 ────────────────────────────────────────────────────────
static inline int16_t toQ14(float v)
{
    int32_t x = (int32_t)(v * 16384.0f);
    if(x > 32767)
        x = 32767;
    if(x < -32768)
        x = -32768;
    return (int16_t)x;
}

// ── Konstruktor ───────────────────────────────────────────────────────────────
W8Band::W8Band(TwoWire &wireBus, uint8_t i2cAddress)
    : m_Imu(&wireBus, i2cAddress),
      m_BLEService("19B10000-E8F2-537E-4F6C-D104768A1214"),
      m_ControlCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214",
                              BLERead | BLEWrite),
      m_DataCharacteristic("19B10002-E8F2-537E-4F6C-D104768A1214")
{ instance = this; }

// ── Init ──────────────────────────────────────────────────────────────────────
void W8Band::Init()
{
    Serial.begin(115200);
    InitLsm();
    InitBle();

    // Podłącz ISR do pinów przerwań
    pinMode(PIN_INT1, INPUT);
    pinMode(PIN_INT2, INPUT);
    attachInterrupt(digitalPinToInterrupt(PIN_INT1), IsrInt1, RISING);
    attachInterrupt(digitalPinToInterrupt(PIN_INT2), IsrInt2, RISING);

    Serial.println("Ready. Send 0x01 via BLE to arm.");
}

// ── InitLsm ───────────────────────────────────────────────────────────────────
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

    // FIFO: kwaterniony + akcelerometr
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

// ── ConfigureWakeInact
// ──────────────────────────────────────────────────────── Konfiguruje:
//   INT1 → Significant Motion Detection (SMD)  — start ruchu
//   INT2 → Sleep / Inactivity                  — koniec ruchu
//
// Rejestry wg AN5763 Rev2:
//   FUNCTIONS_ENABLE (50h)  — włącza Activity/Inactivity
//   INACTIVITY_DUR   (54h)  — czas bezruchu + ODR w sleep
//   INACTIVITY_THS   (55h)  — próg przyspieszenia
//   WAKE_UP_THS      (5Bh)  — próg wake-up (ten sam co inactivity w tym
//   trybie) WAKE_UP_DUR      (5Ch)  — czas trwania wake + sleep_dur MD1_CFG
//   (5Eh)  — mapa INT1: SMD MD2_CFG          (5Fh)  — mapa INT2: Sleep change
//   EMB_FUNC_EN_A    (04h)  — włącza SMD (przez embedded functions)
//   EMB_FUNC_INT1    (0Ah)  — SMD → INT1
// ─────────────────────────────────────────────────────────────────────────────
void W8Band::ConfigureWakeInact()
{
    // Pomocnicze lambdy — stm32duino API: ReadReg(reg, *val), WriteReg(reg, val)
    auto imuRead = [&](uint8_t reg) -> uint8_t {
        uint8_t v = 0;
        m_Imu.Read_Reg(reg, &v);
        return v;
    };
    auto imuWrite
        = [&](uint8_t reg, uint8_t val) { m_Imu.Write_Reg(reg, val); };

    // ── 1. Włącz Activity/Inactivity w FUNCTIONS_ENABLE (0x50)
    // ─────────────── INACT_EN[1:0] = 01 → stationary/motion detection (bez
    // zmiany ODR) INTERRUPTS_ENABLE = 1
    uint8_t func_en = imuRead(LSM6DSV16X_FUNCTIONS_ENABLE);
    func_en |= (1 << 0);  // INACT_EN_0 = 1
    func_en &= ~(1 << 1); // INACT_EN_1 = 0  → tryb 01
    func_en |= (1 << 7);  // INTERRUPTS_ENABLE
    imuWrite(LSM6DSV16X_FUNCTIONS_ENABLE, func_en);

    // ── 2. Czas bezruchu i ODR w sleep — INACTIVITY_DUR (0x54) ──────────────
    // INACT_DUR[1:0]    = 00 → 1 próbka powyżej progu → wychodzi ze sleep
    // XL_INACT_ODR[1:0] = 00 → akcelerometr 1.875 Hz w sleep (oszczędność)
    // WU_INACT_THS_W    = 0  → próg wake-up = próg inactivity
    // SLEEP_STATUS_ON_INT = 1 → INT2 pulsuje przy każdej zmianie sleep/wake
    imuWrite(LSM6DSV16X_INACTIVITY_DUR, (1 << 7)); // SLEEP_STATUS_ON_INT = 1

    // ── 3. Próg inactivity — INACTIVITY_THS (0x55)
    // ─────────────────────────── INACT_THS[5:0]: 1 LSB = 31.25 mg przy FS=±4g
    // 3 LSB ≈ 94 mg — spokojny leżak, bez fałszywych wyzwoleń od drżenia rąk
    imuWrite(LSM6DSV16X_INACTIVITY_THS, INACT_THRESHOLD_LSB & 0x3F);

    // ── 4. Próg wake-up i czas — WAKE_UP_THS (0x5B) + WAKE_UP_DUR (0x5C) ────
    imuWrite(LSM6DSV16X_WAKE_UP_THS, INACT_THRESHOLD_LSB & 0x3F);

    // SLEEP_DUR[3:0]: 1 LSB = 512/ODR_XL ≈ 4.3 s przy 120 Hz
    // SLEEP_DUR=4 → ~17 s bezruchu → koniec serii
    imuWrite(LSM6DSV16X_WAKE_UP_DUR, SLEEP_DUR_VAL & 0x0F);

    // ── 5. Mapa INT2: Sleep change → INT2 (MD2_CFG 0x5F) ─────────────────────
    uint8_t md2 = imuRead(LSM6DSV16X_MD2_CFG);
    md2 |= (1 << 7); // INT2_SLEEP_CHANGE = 1
    imuWrite(LSM6DSV16X_MD2_CFG, md2);

    // ── 6. Significant Motion Detection przez embedded functions
    // ────────────── Włącz dostęp do embedded function registers
    imuWrite(LSM6DSV16X_FUNC_CFG_ACCESS, 0x80); // EMB_FUNC_REG_ACCESS = 1
    delay(5);

    // EMB_FUNC_EN_A (0x04): SIGN_MOTION_EN = 1 (bit 5)
    uint8_t emb_en_a = imuRead(LSM6DSV16X_EMB_FUNC_EN_A);
    emb_en_a |= (1 << 5); // SIGN_MOTION_EN
    imuWrite(LSM6DSV16X_EMB_FUNC_EN_A, emb_en_a);

    // EMB_FUNC_INT1 (0x0A): INT1_SIG_MOT = 1 (bit 5)
    uint8_t emb_int1 = imuRead(LSM6DSV16X_EMB_FUNC_INT1);
    emb_int1 |= (1 << 5); // INT1_SIG_MOT
    imuWrite(LSM6DSV16X_EMB_FUNC_INT1, emb_int1);

    // MD1_CFG (0x5E): INT1_EMB_FUNC = 1 (bit 1) — routing embedded → INT1
    uint8_t md1 = imuRead(LSM6DSV16X_MD1_CFG);
    md1 |= (1 << 1); // INT1_EMB_FUNC
    imuWrite(LSM6DSV16X_MD1_CFG, md1);

    // Zainicjuj SMD
    uint8_t emb_init = imuRead(LSM6DSV16X_EMB_FUNC_INIT_A);
    emb_init |= (1 << 5); // SIG_MOT_INIT
    imuWrite(LSM6DSV16X_EMB_FUNC_INIT_A, emb_init);

    // Zamknij dostęp do embedded function registers
    imuWrite(LSM6DSV16X_FUNC_CFG_ACCESS, 0x00);
    delay(5);

    Serial.println("SMD + Inactivity: configured");
    Serial.print("  Wake/inactivity threshold: ");
    Serial.print(INACT_THRESHOLD_LSB * 31.25f, 1);
    Serial.println(" mg");
    Serial.print("  Inactivity timeout: ~");
    Serial.print(SLEEP_DUR_VAL * 512.0f / IMU_FREQ, 1);
    Serial.println(" s");
}

// ── StartCalibration ──────────────────────────────────────────────────────────
void W8Band::StartCalibration()
{
    m_State = State::CALIBRATING;
    s_SmdFired = false;
    s_InactFired = false;

    // Wyczyść i zarezerwuj bufor
    m_Data.clear();
    m_Data.reserve(1200);
    m_HaveFreshQuat = false;
    m_HaveFreshAccel = false;
    m_Seq = 0;

    // Flush FIFO
    m_Imu.FIFO_Set_Mode(LSM6DSV16X_BYPASS_MODE);
    delay(10);
    m_Imu.FIFO_Set_Mode(LSM6DSV16X_STREAM_MODE);
    delay(10);

    // Włącz SMD + Inactivity
    ConfigureWakeInact();

    Serial.println("CALIBRATING — waiting for Significant Motion...");
    Serial.println("(Lift the bar to start recording)");
}

// ── StartRecording ────────────────────────────────────────────────────────────
void W8Band::StartRecording()
{
    m_State = State::RECORDING;
    Serial.println("RECORDING started.");
}

// ── StopRecording ─────────────────────────────────────────────────────────────
void W8Band::StopRecording()
{
    m_State = State::TRANSFERRING;
    Serial.print("RECORDING stopped. Samples: ");
    Serial.println(m_Data.size());
    SendBLEData();
}

// ── InitBle ───────────────────────────────────────────────────────────────────
void W8Band::InitBle()
{
    Bluefruit.begin();
    Bluefruit.setTxPower(4);
    Bluefruit.setName("W8Band");
    Bluefruit.Periph.setConnectCallback(ConnectCallback);
    Bluefruit.Periph.setDisconnectCallback(DisconnectCallback);

    m_BLEService.begin();

    m_ControlCharacteristic.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE);
    m_ControlCharacteristic.setPermission(SECMODE_OPEN, SECMODE_OPEN);
    m_ControlCharacteristic.setFixedLen(1);
    m_ControlCharacteristic.begin();
    m_ControlCharacteristic.write8(0);
    m_ControlCharacteristic.setWriteCallback(WriteCallback);

    m_DataCharacteristic.setProperties(CHR_PROPS_NOTIFY);
    m_DataCharacteristic.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
    m_DataCharacteristic.setFixedLen(sizeof(SamplePacket));
    m_DataCharacteristic.begin();

    Bluefruit.Advertising.addFlags(
        BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
    Bluefruit.Advertising.addTxPower();
    Bluefruit.Advertising.addService(m_BLEService);
    Bluefruit.Advertising.addName();
    Bluefruit.Advertising.restartOnDisconnect(true);
    Bluefruit.Advertising.start(0);

    Serial.println("BLE: OK");
}

// ── Update ────────────────────────────────────────────────────────────────────
void W8Band::Update()
{
    switch(m_State)
    {
    // ── IDLE: nic nie rób ────────────────────────────────────────────────────
    case State::IDLE: break;

    // ── CALIBRATING: czekaj na SMD od czujnika ───────────────────────────────
    case State::CALIBRATING:
        if(s_SmdFired)
        {
            s_SmdFired = false;
            StartRecording();
        }
        break;

    // ── RECORDING: zbieraj próbki, obserwuj Inactivity ───────────────────────
    case State::RECORDING: {
        // Koniec ruchu: czujnik wykrył bezruch przez SLEEP_DUR
        if(s_InactFired)
        {
            s_InactFired = false;
            StopRecording();
            return;
        }

        // Odczyt FIFO
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
                float tmp[4];
                m_Imu.FIFO_Get_Rotation_Vector(tmp);
                m_LatestQuat[0] = tmp[3]; // Qw = r
                m_LatestQuat[1] = tmp[0]; // Qx = i
                m_LatestQuat[2] = tmp[1]; // Qy = j
                m_LatestQuat[3] = tmp[2]; // Qz = k
                m_HaveFreshQuat = true;
            } else if(m_Tag == TAG_ACCELEROMETER)
            {
                m_Imu.FIFO_Get_X_Axes(m_LatestAccel);
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
            }
        }
        break;
    }

    // ── TRANSFERRING: SendBLEData() już wywołane w StopRecording() ───────────
    case State::TRANSFERRING: break;
    }
}

// ── SendBLEData ───────────────────────────────────────────────────────────────
void W8Band::SendBLEData()
{
    if(m_Data.empty())
    {
        m_State = State::IDLE;
        m_ControlCharacteristic.write8(0);
        return;
    }

    const int total = (int)m_Data.size();
    int sent = 0;

    while(sent < total)
    {
        int wait = 200;
        while(!Bluefruit.connected() && wait-- > 0)
            delay(10);
        if(!Bluefruit.connected())
        {
            Serial.println("BLE lost");
            break;
        }

        bool ok = m_DataCharacteristic.notify(
            reinterpret_cast<const uint8_t *>(&m_Data[sent]),
            sizeof(SamplePacket));

        if(ok)
        {
            sent++;
            if(sent % 8 == 0)
                delay(1);
        } else
            delay(2);
    }

    // Pakiet EOF
    SamplePacket eof;
    memset(&eof, 0, sizeof(eof));
    eof.seq = 0xFFFF;
    eof.timestamp_ms = (uint32_t)total;

    for(int i = 0; i < 5; i++)
    {
        int wait = 100;
        while(!Bluefruit.connected() && wait-- > 0)
            delay(10);
        if(!Bluefruit.connected())
            break;
        m_DataCharacteristic.notify(reinterpret_cast<const uint8_t *>(&eof),
                                    sizeof(eof));
        delay(20);
    }

    Serial.print("Sent: ");
    Serial.print(sent);
    Serial.print(" / ");
    Serial.println(total);

    m_ControlCharacteristic.write8(0);
    m_State = State::IDLE;
}

// ── Callbacki BLE ─────────────────────────────────────────────────────────────
void W8Band::WriteCallback(uint16_t, BLECharacteristic *, uint8_t *data,
                           uint16_t len)
{
    if(len < 1 || !instance)
        return;

    if(data[0] == 0x01 && instance->m_State == State::IDLE)
        instance->StartCalibration();
    else if(data[0] == 0x00 && instance->m_State == State::RECORDING)
        instance->StopRecording();
}

void W8Band::ConnectCallback(uint16_t conn_hdl)
{
    Serial.println("BLE connected");
    BLEConnection *conn = Bluefruit.Connection(conn_hdl);
    if(conn)
        conn->requestMtuExchange(247);
}

void W8Band::DisconnectCallback(uint16_t, uint8_t)
{ Serial.println("BLE disconnected"); }

} // namespace w8band