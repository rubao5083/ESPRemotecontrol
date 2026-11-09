/*****************************************************************************
 * | File        :   main.cpp  (mpu6050_test)
 * | Function    :   Firmware TEMPORARIO de diagnostico - testa o MPU6050
 * |                 isolado, usando um bit-bang I2C ESCRITO POR NOS (sem
 * |                 Wire nem bibliotecas externas), apos o Wire nativo
 * |                 (bug ESP_ERR_INVALID_STATE do driver "ng") e a
 * |                 ESP32_SoftWire (ACK pouco confiavel/dados as vezes
 * |                 "presos" em bancada) terem se mostrado problematicos
 * |                 nesta placa. Implementacao simples e didatica
 * |                 (digitalWrite/digitalRead/delayMicroseconds), facil de
 * |                 auditar e ajustar.
 ******************************************************************************/

#include <Arduino.h>

// Mesmos pinos fisicos confirmados em src/relay_main/main.cpp
// (MPU6050_SDA_PIN/MPU6050_SCL_PIN) - movidos de 21/22 para 17/18.
static const uint8_t SDA_PIN = 17;
static const uint8_t SCL_PIN = 18;

// Endereco I2C do MPU6050 (0x68 com AD0 em GND/flutuando, padrao GY-521).
static const uint8_t MPU_ADDR = 0x68;

// ======================= Bit-bang I2C (proprio) =============================

static uint8_t s_sda = 0, s_scl = 0;
static uint32_t s_half_period_us = 5; // 5us+5us = 10us periodo = 100kHz

static inline void sclHigh() { digitalWrite(s_scl, HIGH); } // solta (open-drain, pull-up externo sobe)
static inline void sclLow()  { digitalWrite(s_scl, LOW); }  // puxa ativamente para baixo
static inline void sdaHigh() { digitalWrite(s_sda, HIGH); }
static inline void sdaLow()  { digitalWrite(s_sda, LOW); }
static inline bool sdaRead() { return digitalRead(s_sda) != 0; }
static inline void i2cDelay() { delayMicroseconds(s_half_period_us); }

static void i2cStart()
{
    sdaHigh();
    sclHigh();
    i2cDelay();
    sdaLow(); // SDA cai com SCL alto = START
    i2cDelay();
    sclLow();
    i2cDelay();
}

static void i2cStop()
{
    sdaLow();
    i2cDelay();
    sclHigh();
    i2cDelay();
    sdaHigh(); // SDA sobe com SCL alto = STOP
    i2cDelay();
}

// Escreve 8 bits (MSB primeiro) e le o bit de ACK. Retorna true se o
// escravo confirmou (ACK = SDA baixo durante o 9o pulso de clock).
static bool i2cWriteByte(uint8_t byte)
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
    // 9o pulso: solta SDA para o escravo puxar para baixo (ACK)
    sdaHigh();
    i2cDelay();
    sclHigh();
    i2cDelay();
    bool ack = !sdaRead();
    sclLow();
    i2cDelay();
    return ack;
}

// Le 8 bits (MSB primeiro). "ack" controla o que o MESTRE envia no 9o bit:
// true = ACK (quero mais bytes), false = NACK (ultimo byte desta leitura).
static uint8_t i2cReadByte(bool ack)
{
    uint8_t value = 0;
    sdaHigh(); // solta para o escravo poder dirigir SDA
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

// Escreve "len" bytes de "data" no registrador "reg" do dispositivo "addr".
// Retorna false se o endereco OU o registrador nao forem confirmados (ACK).
static bool i2cWriteRegs(uint8_t addr, uint8_t reg, const uint8_t *data, uint8_t len)
{
    i2cStart();
    bool ok = i2cWriteByte((uint8_t)(addr << 1));
    if (ok) {
        ok = i2cWriteByte(reg);
    }
    for (uint8_t i = 0; ok && i < len; i++) {
        ok = i2cWriteByte(data[i]);
    }
    i2cStop();
    return ok;
}

// Le "len" bytes a partir do registrador "reg" do dispositivo "addr" para
// "buf". STOP completo entre a escrita do ponteiro e a leitura (nao usa
// repeated start) - mais simples e evita qualquer ambiguidade de timing.
static bool i2cReadRegs(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len)
{
    i2cStart();
    bool ok = i2cWriteByte((uint8_t)(addr << 1));
    if (ok) {
        ok = i2cWriteByte(reg);
    }
    i2cStop();
    if (!ok) {
        return false;
    }

    i2cStart();
    ok = i2cWriteByte((uint8_t)((addr << 1) | 1));
    if (!ok) {
        i2cStop();
        return false;
    }
    for (uint8_t i = 0; i < len; i++) {
        buf[i] = i2cReadByte(i < (uint8_t)(len - 1));
    }
    i2cStop();
    return true;
}

// Inicializa os pinos e faz a recuperacao de barramento "preso" (problema
// classico apos reset do ESP32 no meio de uma transacao - o MPU6050
// continua ligado e pode ficar esperando mais pulsos de clock para
// terminar de enviar um byte, nunca mais respondendo depois disso).
static void i2cBegin(uint8_t sda_pin, uint8_t scl_pin, uint32_t freq_hz)
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
            break; // escravo ja liberou SDA, barramento OK
        }
        sclLow();
        delayMicroseconds(5);
        sclHigh();
        delayMicroseconds(5);
    }

    pinMode(s_sda, OUTPUT_OPEN_DRAIN);
    sdaHigh();
    delayMicroseconds(5);
    i2cStop(); // garante um STOP limpo antes de qualquer transacao real
}

// ======================= Teste do MPU6050 ===================================

static int16_t rawX, rawY, rawZ;
static float ax, ay, az;
static float pitch, roll;

void setup()
{
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n=== Teste isolado do MPU6050 (bit-bang proprio) ===");

    i2cBegin(SDA_PIN, SCL_PIN, 100000);

    uint8_t who = 0xFF;
    bool ok = i2cReadRegs(MPU_ADDR, 0x75, &who, 1);
    Serial.printf("WHO_AM_I: 0x%02X (ok=%d, esperado 0x68)\n", who, ok);

    uint8_t pwr_before = 0xFF;
    i2cReadRegs(MPU_ADDR, 0x6B, &pwr_before, 1);
    Serial.printf("PWR_MGMT_1 ANTES da escrita: 0x%02X\n", pwr_before);

    // 0x01 = PLL com referencia no giroscopio do eixo X (recomendado pelo
    // datasheet, mais preciso que o oscilador interno/0x00).
    uint8_t clk_source = 0x01;
    bool wrote = i2cWriteRegs(MPU_ADDR, 0x6B, &clk_source, 1);
    Serial.printf("Escrita de wake-up (PWR_MGMT_1=0x01): ok=%d\n", wrote);

    uint8_t pwr_after = 0xFF;
    i2cReadRegs(MPU_ADDR, 0x6B, &pwr_after, 1);
    Serial.printf("PWR_MGMT_1 DEPOIS da escrita: 0x%02X (esperado 0x01)\n", pwr_after);
}

void loop()
{
    // Burst read de 14 bytes a partir de 0x3B: o ponteiro de registro do
    // MPU6050 incrementa automaticamente, entao uma unica transacao traz
    // Accel(6) + Temp(2) + Gyro(6) - mesmo padrao usado pelo driver real
    // em lib/imu_mpu6050/mpu6050.cpp.
    uint8_t raw[14];
    bool ok = i2cReadRegs(MPU_ADDR, 0x3B, raw, sizeof(raw));

    if (!ok) {
        Serial.println("Erro: leitura do acelerometro falhou.");
    } else {
        rawX = (int16_t)((raw[0] << 8) | raw[1]);
        rawY = (int16_t)((raw[2] << 8) | raw[3]);
        rawZ = (int16_t)((raw[4] << 8) | raw[5]);
        // raw[6..7] = temperatura, raw[8..13] = giroscopio X/Y/Z (nao
        // usados neste teste, so para validar o burst read completo).

        // Escala padrao de fabrica +-2g -> 16384 LSB/g.
        ax = (float)rawX / 16384.0f;
        ay = (float)rawY / 16384.0f;
        az = (float)rawZ / 16384.0f;

        pitch = atan2(ay, sqrt(ax * ax + az * az)) * RAD_TO_DEG;
        roll = atan2(-ax, az) * RAD_TO_DEG;

        Serial.print("Acel_g(X:"); Serial.print(ax, 2);
        Serial.print(" Y:"); Serial.print(ay, 2);
        Serial.print(" Z:"); Serial.print(az, 2);
        Serial.print(") | Inclinacao(Pitch:"); Serial.print(pitch, 1);
        Serial.print(" Roll:"); Serial.print(roll, 1);
        Serial.println(")");
    }

    delay(50); // ~20Hz, mesmo ritmo do driver real em lib/imu_mpu6050
}
