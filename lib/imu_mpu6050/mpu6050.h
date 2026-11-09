/*****************************************************************************
 * | File        :   mpu6050.h
 * | Function    :   Driver for an external MPU6050 6-axis IMU (3-axis
 * |                 accelerometer + 3-axis gyroscope) montado sobre a placa
 * |                 do modulo de reles (ESP32_Relay_30A_X4, ESP32 classico -
 * |                 ver CLAUDE.md regra 7). Usa I2C por bit-bang proprio em
 * |                 GPIO puro (nao o Wire/driver i2c_master do ESP-IDF, que
 * |                 deu ESP_ERR_INVALID_STATE e ate crash em bancada), ao
 * |                 contrario de lib/i2c (que usa o driver ESP-IDF com pinos
 * |                 fixos GPIO8/9, especificos da tela S3) - aqui os pinos
 * |                 SDA/SCL sao passados em begin(), pois variam por placa.
 * |
 * | ALIMENTACAO :   O modulo GY-521 tem regulador de 3,3V proprio: alimente
 * |                 o VCC com 5V. Com 3,3V no VCC o regulador fica sem folga
 * |                 e as leituras ficam corrompidas/instaveis.
 * |
 * | NOTE        :   Assim como o QMI8658 (lib/imu_qmi8658), o MPU6050 NAO
 * |                 contem magnetometro - so fornece inclinacao (roll/pitch),
 * |                 nunca heading/bussola (ver CLAUDE.md regra 5).
 ******************************************************************************/

#ifndef __MPU6050_H
#define __MPU6050_H

#include <stdint.h>
#include <stdbool.h>

namespace imu {

/**
 * @brief Attitude (inclination) reading, in degrees. Mesmo formato usado
 *        por lib/imu_qmi8658 (Attitude), para manter o codigo de quem
 *        consome a leitura simetrico entre os dois firmwares.
 *
 * roll_deg  : rotation around the front-back axis (side-to-side tilt)
 * pitch_deg : rotation around the left-right axis (nose up/down tilt)
 * heading_deg / has_heading : always false/NaN (sem magnetometro)
 */
struct Attitude {
    float roll_deg = 0.0f;
    float pitch_deg = 0.0f;
    float heading_deg = 0.0f;
    bool has_heading = false;
    bool valid = false;
};

/**
 * @brief Initialize the MPU6050 over I2C (bit-bang proprio), usando os pinos
 *        SDA/SCL informados. Na placa de reles deste projeto
 *        (ESP32_Relay_30A_X4) estao ligados em GPIO17 (SDA) / GPIO18 (SCL) -
 *        ver constantes MPU6050_SDA_PIN/MPU6050_SCL_PIN em
 *        src/relay_main/main.cpp.
 *
 * @return true if the sensor was detected (WHO_AM_I) and configured.
 */
bool begin(uint8_t sda_pin, uint8_t scl_pin);

/**
 * @brief Read accelerometer + gyroscope and compute filtered roll/pitch.
 *
 * Call periodically (e.g. every 20-50 ms) from the main loop/task.
 * Uses a complementary filter (accelerometer tilt + gyro integration) to
 * reduce noise while staying responsive.
 *
 * @return Attitude with valid=false if the sensor is not initialized/present.
 */
Attitude read();

} // namespace imu

#endif // __MPU6050_H
