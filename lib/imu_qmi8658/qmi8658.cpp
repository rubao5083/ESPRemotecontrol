/*****************************************************************************
 * | File        :   qmi8658.cpp
 * | Function    :   Driver for the onboard QMI8658 6-axis IMU.
 *
 * Register map and enable/range/odr bitfields based on the official QMI8658
 * register layout (WHO_AM_I=0x00, CTRL1..CTRL9=0x02..0x0A, accel/gyro data
 * registers 0x35..0x40).
 ******************************************************************************/

#include <math.h>
#include <Arduino.h>
#include "i2c.h"
#include "qmi8658.h"

namespace imu {

namespace {

constexpr uint8_t REG_WHO_AM_I = 0x00;
constexpr uint8_t REG_CTRL1    = 0x02;
constexpr uint8_t REG_CTRL2    = 0x03; // Accelerometer: range + ODR
constexpr uint8_t REG_CTRL3    = 0x04; // Gyroscope: range + ODR
constexpr uint8_t REG_CTRL7    = 0x08; // Sensor enable (ACC/GYR)
constexpr uint8_t REG_AX_L     = 0x35; // Accel X..Z, then Gyro X..Z (12 bytes, auto-increment)

constexpr uint8_t WHO_AM_I_VALUE = 0x05;

// CTRL2: accel range +-8g (0x02 << 4) | ODR 250Hz (0x05)
constexpr uint8_t CTRL2_ACC_8G_250HZ = (0x02 << 4) | 0x05;
// CTRL3: gyro range +-512dps (0x04 << 4) | ODR 250Hz (0x05)
constexpr uint8_t CTRL3_GYR_512DPS_250HZ = (0x04 << 4) | 0x05;
// CTRL7: enable accelerometer (bit0) + gyroscope (bit1)
constexpr uint8_t CTRL7_ACC_GYR_ENABLE = 0x03;

// Candidate I2C addresses: 0x6B is the default, 0x6A if SA0 is strapped low.
constexpr uint8_t kCandidateAddresses[] = {0x6B, 0x6A};

// Sensitivity for the ranges configured above.
constexpr float kAccelLsbPerG = 4096.0f;   // 32768 / 8g
constexpr float kGyroLsbPerDps = 64.0f;    // 32768 / 512dps

uint8_t s_addr = 0;
bool s_initialized = false;

float s_filtered_roll = 0.0f;
float s_filtered_pitch = 0.0f;
bool s_filter_seeded = false;
uint32_t s_last_read_ms = 0;

bool readRegs(uint8_t reg, uint8_t *buf, uint32_t len)
{
    return DEV_I2C_Read_Nbyte(s_addr, reg, buf, len) == ESP_OK;
}

bool writeReg(uint8_t reg, uint8_t value)
{
    return DEV_I2C_Write_Byte(s_addr, reg, value) == ESP_OK;
}

bool probeAddress(uint8_t addr)
{
    s_addr = addr;
    uint8_t who = 0;
    if (!readRegs(REG_WHO_AM_I, &who, 1)) {
        return false;
    }
    return who == WHO_AM_I_VALUE;
}

} // namespace

bool begin()
{
    s_initialized = false;

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

    // CTRL1: enable auto-increment of register address + little-endian output (default 0x60)
    writeReg(REG_CTRL1, 0x60);
    writeReg(REG_CTRL2, CTRL2_ACC_8G_250HZ);
    writeReg(REG_CTRL3, CTRL3_GYR_512DPS_250HZ);
    writeReg(REG_CTRL7, CTRL7_ACC_GYR_ENABLE);

    delay(10);

    s_filter_seeded = false;
    s_last_read_ms = millis();
    s_initialized = true;
    return true;
}

Attitude read()
{
    Attitude result;

    if (!s_initialized) {
        return result;
    }

    uint8_t raw[12] = {0};
    if (!readRegs(REG_AX_L, raw, sizeof(raw))) {
        return result;
    }

    int16_t ax_raw = (int16_t)((raw[1] << 8) | raw[0]);
    int16_t ay_raw = (int16_t)((raw[3] << 8) | raw[2]);
    int16_t az_raw = (int16_t)((raw[5] << 8) | raw[4]);
    int16_t gx_raw = (int16_t)((raw[7] << 8) | raw[6]);
    (void)gx_raw; // gyro X not used in roll/pitch complementary filter below

    float ax = ax_raw / kAccelLsbPerG;
    float ay = ay_raw / kAccelLsbPerG;
    float az = az_raw / kAccelLsbPerG;

    int16_t gy_raw = (int16_t)((raw[9] << 8) | raw[8]);
    int16_t gz_raw = (int16_t)((raw[11] << 8) | raw[10]);
    (void)gz_raw;

    float gy_dps = gy_raw / kGyroLsbPerDps; // rotation rate affecting pitch
    float gx_dps = gx_raw / kGyroLsbPerDps; // rotation rate affecting roll

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
    result.has_heading = false; // no magnetometer on this board
    result.heading_deg = NAN;
    result.valid = true;
    return result;
}

} // namespace imu
