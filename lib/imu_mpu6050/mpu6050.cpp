/*****************************************************************************
 * | File        :   mpu6050.cpp
 * | Function    :   Driver for the external MPU6050 6-axis IMU.
 *
 * Register map based on the official InvenSense MPU-6000/MPU-6050 Register
 * Map document (WHO_AM_I=0x75, PWR_MGMT_1=0x6B, data registers 0x3B..0x48).
 * NOTA: ao contrario do QMI8658 (little-endian), os registradores do
 * MPU6050 sao BIG-ENDIAN (byte alto primeiro).
 ******************************************************************************/

#include <math.h>
#include <Arduino.h>
#include "mpu6050.h"

namespace imu {

namespace {

constexpr uint8_t REG_SMPLRT_DIV   = 0x19;
constexpr uint8_t REG_CONFIG       = 0x1A; // DLPF
constexpr uint8_t REG_GYRO_CONFIG  = 0x1B; // full scale select
constexpr uint8_t REG_ACCEL_CONFIG = 0x1C; // full scale select
constexpr uint8_t REG_PWR_MGMT_1   = 0x6B; // bit6 = sleep (precisa ser zerado no boot)
constexpr uint8_t REG_WHO_AM_I     = 0x75;
constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B; // Accel X..Z, Temp, Gyro X..Z (14 bytes, auto-increment)

// Valores tipicos de WHO_AM_I para o MPU6050 (0x68) - alguns clones/variantes
// (ex. MPU6500 na mesma pinagem) respondem 0x70/0x72, aceitos aqui tambem
// para nao travar a deteccao por causa de uma revisao de chip diferente.
constexpr uint8_t kWhoAmIAccepted[] = {0x68, 0x70, 0x72};

// Enderecos candidatos: 0x68 (AD0 em GND/flutuando, padrao da maioria dos
// modulos GY-521) ou 0x69 (AD0 em VCC).
constexpr uint8_t kCandidateAddresses[] = {0x68, 0x69};

// ACCEL_CONFIG = 0x10 -> full scale +-8g (bits 4:3 = 10)
constexpr uint8_t ACCEL_CONFIG_8G = 0x10;
// GYRO_CONFIG = 0x08 -> full scale +-500dps (bits 4:3 = 01)
constexpr uint8_t GYRO_CONFIG_500DPS = 0x08;

constexpr float kAccelLsbPerG = 4096.0f;  // 32768 / 8g
constexpr float kGyroLsbPerDps = 65.5f;   // 32768 / 500dps

uint8_t s_addr = 0;
bool s_initialized = false;
// true assim que o chip e identificado (WHO_AM_I aceito) pela primeira vez -
// NUNCA volta a false depois disso (diferente de s_initialized, que reflete
// o estado "configurado" mais recente e pode cair para false numa tentativa
// de reinicializacao que falhou). Usado por read() para decidir se vale a
// pena continuar tentando falar com o sensor, mesmo que uma reinicializacao
// recente tenha falhado - sem isso, uma unica falha de reinit deixaria o
// sensor "morto" ate o proximo reset fisico do ESP32.
bool s_detected_once = false;

float s_filtered_roll = 0.0f;
float s_filtered_pitch = 0.0f;
bool s_filter_seeded = false;
uint32_t s_last_read_ms = 0;

// ---------------------------------------------------------------------------
// I2C por bit-bang proprio (GPIO puro), em vez do Wire/TwoWire. O driver
// "i2c_master" do arduino-esp32 3.x/ESP-IDF 5.x apresentou
// ESP_ERR_INVALID_STATE intermitente e ate um crash (LoadProhibited dentro
// de i2c_master_receive) neste projeto, e a ESP32_SoftWire se mostrou
// pouco confiavel com o ACK. Este bit-bang e minimo, sem dependencias, e
// foi validado em bancada (src/mpu6050_test): 0 erros em centenas de
// leituras. IMPORTANTE: a CAUSA REAL da instabilidade observada era a
// alimentacao do GY-521 - o modulo tem regulador de 3,3V proprio e deve
// ser alimentado com 5V; com 3,3V no VCC o regulador fica sem folga e as
// leituras ficam corrompidas (valores com bits deslocados, NACKs).
// ---------------------------------------------------------------------------
uint8_t s_sda = 0;
uint8_t s_scl = 0;
uint32_t s_half_period_us = 5; // 5us + 5us = 100kHz

inline void sclHigh() { digitalWrite(s_scl, HIGH); } // open-drain: solta, o pull-up sobe
inline void sclLow()  { digitalWrite(s_scl, LOW); }
inline void sdaHigh() { digitalWrite(s_sda, HIGH); }
inline void sdaLow()  { digitalWrite(s_sda, LOW); }
inline bool sdaRead() { return digitalRead(s_sda) != 0; }
inline void i2cDelay() { delayMicroseconds(s_half_period_us); }

void i2cStart()
{
    sdaHigh();
    sclHigh();
    i2cDelay();
    sdaLow(); // SDA cai com SCL alto = START
    i2cDelay();
    sclLow();
    i2cDelay();
}

void i2cStop()
{
    sdaLow();
    i2cDelay();
    sclHigh();
    i2cDelay();
    sdaHigh(); // SDA sobe com SCL alto = STOP
    i2cDelay();
}

// Escreve 8 bits (MSB primeiro) e le o ACK do escravo (SDA baixo no 9o pulso).
bool i2cWriteByte(uint8_t byte)
{
    for (uint8_t i = 0; i < 8; i++) {
        if (byte & 0x80) {
            sdaHigh();
        } else {
            sdaLow();
        }
        byte <<= 1;
        i2cDelay();
        sclHigh();
        i2cDelay();
        sclLow();
        i2cDelay();
    }
    sdaHigh();
    i2cDelay();
    sclHigh();
    i2cDelay();
    bool ack = !sdaRead();
    sclLow();
    i2cDelay();
    return ack;
}

// Le 8 bits (MSB primeiro). ack=true -> mestre responde ACK (quer mais
// bytes); ack=false -> NACK (ultimo byte da leitura).
uint8_t i2cReadByte(bool ack)
{
    uint8_t value = 0;
    sdaHigh();
    for (uint8_t i = 0; i < 8; i++) {
        i2cDelay();
        sclHigh();
        i2cDelay();
        value = (uint8_t)((value << 1) | (sdaRead() ? 1 : 0));
        sclLow();
    }
    if (ack) {
        sdaLow();
    } else {
        sdaHigh();
    }
    i2cDelay();
    sclHigh();
    i2cDelay();
    sclLow();
    sdaHigh();
    i2cDelay();
    return value;
}

// Configura os pinos e destrava um barramento preso (ex.: ESP32 resetou no
// meio de uma transacao e o MPU6050, que continua energizado, ficou
// esperando clocks): pulsa SCL ate 9 vezes ate o escravo soltar SDA e
// termina com um STOP limpo.
void i2cInit(uint8_t sda_pin, uint8_t scl_pin, uint32_t freq_hz)
{
    s_sda = sda_pin;
    s_scl = scl_pin;
    s_half_period_us = 500000UL / freq_hz;
    if (s_half_period_us == 0) {
        s_half_period_us = 1;
    }

    pinMode(s_sda, INPUT_PULLUP);
    pinMode(s_scl, OUTPUT_OPEN_DRAIN);
    sclHigh();
    delayMicroseconds(5);

    for (uint8_t i = 0; i < 9; i++) {
        if (sdaRead()) {
            break;
        }
        sclLow();
        delayMicroseconds(5);
        sclHigh();
        delayMicroseconds(5);
    }

    pinMode(s_sda, OUTPUT_OPEN_DRAIN);
    sdaHigh();
    delayMicroseconds(5);
    i2cStop();
}

bool writeReg(uint8_t reg, uint8_t value)
{
    i2cStart();
    bool ok = i2cWriteByte((uint8_t)(s_addr << 1));
    if (ok) {
        ok = i2cWriteByte(reg);
    }
    if (ok) {
        ok = i2cWriteByte(value);
    }
    i2cStop();
    return ok;
}

// Pequeno retry como margem normal contra ruido eletrico pontual.
constexpr uint8_t kMaxAttempts = 3;

bool writeRegRetry(uint8_t reg, uint8_t value)
{
    for (uint8_t attempt = 0; attempt < kMaxAttempts; attempt++) {
        if (writeReg(reg, value)) {
            return true;
        }
        delay(2);
    }
    return false;
}

// Le "len" bytes a partir de "reg". STOP completo entre a escrita do
// ponteiro de registro e a leitura (sem repeated start) - o ponteiro
// fica latched no MPU6050, e isso evita qualquer ambiguidade de timing.
bool readRegs(uint8_t reg, uint8_t *buf, uint8_t len)
{
    i2cStart();
    bool ok = i2cWriteByte((uint8_t)(s_addr << 1));
    if (ok) {
        ok = i2cWriteByte(reg);
    }
    i2cStop();
    if (!ok) {
        return false;
    }

    i2cStart();
    if (!i2cWriteByte((uint8_t)((s_addr << 1) | 1))) {
        i2cStop();
        return false;
    }
    for (uint8_t i = 0; i < len; i++) {
        buf[i] = i2cReadByte(i < (uint8_t)(len - 1));
    }
    i2cStop();
    return true;
}

// Tenta readRegs() algumas vezes - ver comentario de kMaxAttempts acima.
bool readRegsRetry(uint8_t reg, uint8_t *buf, uint8_t len)
{
    for (uint8_t attempt = 0; attempt < kMaxAttempts; attempt++) {
        if (readRegs(reg, buf, len)) {
            return true;
        }
        delay(2);
    }
    return false;
}

bool isAcceptedWhoAmI(uint8_t who)
{
    for (uint8_t candidate : kWhoAmIAccepted) {
        if (who == candidate) {
            return true;
        }
    }
    return false;
}

bool probeAddress(uint8_t addr)
{
    s_addr = addr;
    uint8_t who = 0;
    if (!readRegsRetry(REG_WHO_AM_I, &who, 1)) {
        return false;
    }
    return isAcceptedWhoAmI(who);
}

} // namespace

bool begin(uint8_t sda_pin, uint8_t scl_pin)
{
    s_initialized = false;

    i2cInit(sda_pin, scl_pin, 100000);
    delay(20);

    bool found = false;
    for (uint8_t addr : kCandidateAddresses) {
        if (probeAddress(addr)) {
            found = true;
            break;
        }
    }
    if (!found) {
        return false;
    }
    s_detected_once = true;

    // Sai do modo sleep (ligado por padrao ao energizar o chip). 0x01 =
    // clock PLL do giroscopio X, recomendado pelo datasheet por ser mais
    // preciso que o oscilador interno (0x00).
    writeRegRetry(REG_PWR_MGMT_1, 0x01);
    delay(10);
    writeRegRetry(REG_SMPLRT_DIV, 0x04);   // 1kHz / (1+4) = 200Hz
    writeRegRetry(REG_CONFIG, 0x03);       // DLPF ~44Hz (reduz ruido de vibracao do veiculo)
    writeRegRetry(REG_GYRO_CONFIG, GYRO_CONFIG_500DPS);
    writeRegRetry(REG_ACCEL_CONFIG, ACCEL_CONFIG_8G);

    delay(10);

    s_filter_seeded = false;
    s_last_read_ms = millis();
    s_initialized = true;
    return true;
}

Attitude read()
{
    Attitude result;

    if (!s_detected_once) {
        return result;
    }

    uint8_t raw[14] = {0};
    if (!readRegsRetry(REG_ACCEL_XOUT_H, raw, sizeof(raw))) {
        return result;
    }

    // Big-endian: byte alto primeiro. raw[6..7] = temperatura (ignorada aqui).
    int16_t ax_raw = (int16_t)((raw[0] << 8) | raw[1]);
    int16_t ay_raw = (int16_t)((raw[2] << 8) | raw[3]);
    int16_t az_raw = (int16_t)((raw[4] << 8) | raw[5]);
    int16_t gx_raw = (int16_t)((raw[8] << 8) | raw[9]);
    int16_t gy_raw = (int16_t)((raw[10] << 8) | raw[11]);
    int16_t gz_raw = (int16_t)((raw[12] << 8) | raw[13]);
    (void)gz_raw; // gyro Z (yaw) nao usado no filtro de roll/pitch abaixo

    float ax = ax_raw / kAccelLsbPerG;
    float ay = ay_raw / kAccelLsbPerG;
    float az = az_raw / kAccelLsbPerG;

    float gx_dps = gx_raw / kGyroLsbPerDps; // rotation rate affecting roll
    float gy_dps = gy_raw / kGyroLsbPerDps; // rotation rate affecting pitch

    // Tilt angles purely from accelerometer (degrees)
    float accel_roll_deg  = atan2f(ay, az) * 180.0f / (float)M_PI;
    float accel_pitch_deg = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / (float)M_PI;

    uint32_t now = millis();
    float dt = (now - s_last_read_ms) / 1000.0f;
    if (dt <= 0.0f || dt > 0.5f) {
        dt = 0.02f; // fall back to a sane default (first sample or long gap)
    }
    s_last_read_ms = now;

    if (!s_filter_seeded) {
        s_filtered_roll = accel_roll_deg;
        s_filtered_pitch = accel_pitch_deg;
        s_filter_seeded = true;
    } else {
        // Complementary filter: mostly trust gyro integration short-term,
        // slowly correct drift using the accelerometer's absolute tilt.
        constexpr float kAlpha = 0.98f;
        float gyro_roll  = s_filtered_roll + gx_dps * dt;
        float gyro_pitch = s_filtered_pitch + gy_dps * dt;
        s_filtered_roll  = kAlpha * gyro_roll  + (1.0f - kAlpha) * accel_roll_deg;
        s_filtered_pitch = kAlpha * gyro_pitch + (1.0f - kAlpha) * accel_pitch_deg;
    }

    result.roll_deg = s_filtered_roll;
    result.pitch_deg = s_filtered_pitch;
    result.has_heading = false; // sem magnetometro
    result.heading_deg = NAN;
    result.valid = true;
    return result;
}

} // namespace imu
