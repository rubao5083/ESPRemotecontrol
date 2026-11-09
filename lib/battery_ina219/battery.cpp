/*****************************************************************************
 * | File        :   battery.cpp
 * | Function    :   Driver for an external INA219 current/voltage monitor
 * |                 used as a LiPo battery charge estimator (see battery.h
 * |                 for why this board needs an external sensor instead of
 * |                 reading the battery through a GPIO).
 *
 * Register map and calibration formula based on the official Texas
 * Instruments INA219 datasheet (SBOS448).
 ******************************************************************************/

#include <Arduino.h>
#include "i2c.h"
#include "battery.h"

namespace battery {

namespace {

constexpr uint8_t REG_CONFIG      = 0x00;
constexpr uint8_t REG_SHUNT_V     = 0x01;
constexpr uint8_t REG_BUS_V       = 0x02;
constexpr uint8_t REG_CURRENT     = 0x04;
constexpr uint8_t REG_CALIBRATION = 0x05;

// Default address with both A0/A1 address pins tied low/floating (the
// common case on generic breakout boards). Change here if your module is
// strapped differently (A0/A1 solder jumpers give 0x40..0x4F).
constexpr uint8_t kAddress = 0x40;

// BRNG=32V, PG=/8 (+-320mV shunt range - safest margin for the up to ~1A
// charge/discharge currents this battery sees), BADC/SADC=12bit no extra
// averaging, MODE=shunt+bus continuous. Written explicitly (instead of
// relying on the power-on-reset default) so the behaviour is documented
// here rather than only in the datasheet.
constexpr uint16_t kConfigValue = 0x399F;

// Shunt resistor on the INA219 breakout board (most common value on
// generic modules - check yours if it is marked differently, e.g. a "R100"
// print near the shunt means 0.1 ohm).
constexpr float kShuntResistanceOhm = 0.1f;

// Max current we ever expect to measure (battery charge/discharge on this
// board - CS8501 charges at ~580mA by default with R45=1.2K, see
// CLAUDE.md/README.md). Some headroom keeps the ADC's current_lsb
// resolution reasonable without risking a math-overflow reading.
constexpr float kMaxExpectedCurrentA = 1.0f;
constexpr float kCurrentLsbAPerBit = kMaxExpectedCurrentA / 32768.0f;

// 1S LiPo open-circuit-voltage discharge curve (volts -> percent),
// descending order. This is an approximation (real curves vary with
// temperature, cell age and discharge rate) - good enough for a UI
// indicator, not a precision fuel gauge.
struct CurvePoint { float voltage; uint8_t percent; };
constexpr CurvePoint kDischargeCurve[] = {
    {4.20f, 100}, {4.15f, 95}, {4.11f, 90}, {4.08f, 85}, {4.02f, 80},
    {3.98f, 75},  {3.95f, 70}, {3.91f, 65}, {3.87f, 60}, {3.85f, 55},
    {3.84f, 50},  {3.82f, 45}, {3.80f, 40}, {3.79f, 35}, {3.77f, 30},
    {3.75f, 25},  {3.73f, 20}, {3.71f, 15}, {3.69f, 10}, {3.61f, 5},
    {3.27f, 0},
};
constexpr int kDischargeCurvePoints = sizeof(kDischargeCurve) / sizeof(kDischargeCurve[0]);

bool s_initialized = false;

bool writeReg16(uint8_t reg, uint16_t value)
{
    uint8_t buf[3] = { reg, (uint8_t)(value >> 8), (uint8_t)(value & 0xFF) };
    return DEV_I2C_Write_nByte(kAddress, buf, sizeof(buf)) == ESP_OK;
}

bool readReg16(uint8_t reg, uint16_t *out)
{
    uint8_t raw[2] = {0};
    if (DEV_I2C_Read_Nbyte(kAddress, reg, raw, sizeof(raw)) != ESP_OK) {
        return false;
    }
    *out = ((uint16_t)raw[0] << 8) | raw[1];
    return true;
}

uint16_t calibrationValue()
{
    // Cal = trunc(0.04096 / (Current_LSB * Rshunt)) - INA219 datasheet 8.5.1.
    return (uint16_t)(0.04096f / (kCurrentLsbAPerBit * kShuntResistanceOhm));
}

uint8_t percentFromVoltage(float voltage)
{
    if (voltage >= kDischargeCurve[0].voltage) {
        return 100;
    }
    if (voltage <= kDischargeCurve[kDischargeCurvePoints - 1].voltage) {
        return 0;
    }
    for (int i = 0; i < kDischargeCurvePoints - 1; i++) {
        const CurvePoint &hi = kDischargeCurve[i];
        const CurvePoint &lo = kDischargeCurve[i + 1];
        if (voltage <= hi.voltage && voltage >= lo.voltage) {
            float span = hi.voltage - lo.voltage;
            float frac = (span > 0.0f) ? (voltage - lo.voltage) / span : 0.0f;
            return (uint8_t)(lo.percent + frac * (hi.percent - lo.percent));
        }
    }
    return 0;
}

} // namespace

bool begin()
{
    s_initialized = false;

    // Reset (bit15) doubles as a presence probe: a device that is not
    // actually on the bus fails this write.
    if (!writeReg16(REG_CONFIG, 0x8000)) {
        return false;
    }
    delay(5);

    if (!writeReg16(REG_CONFIG, kConfigValue)) {
        return false;
    }
    if (!writeReg16(REG_CALIBRATION, calibrationValue())) {
        return false;
    }

    // Confirm the config register actually stuck - a floating/absent bus
    // reads back garbage (e.g. all-0xFF) instead of what we just wrote.
    uint16_t readback = 0;
    if (!readReg16(REG_CONFIG, &readback) || readback != kConfigValue) {
        return false;
    }

    s_initialized = true;
    return true;
}

Reading read()
{
    Reading result;
    if (!s_initialized) {
        return result;
    }

    uint16_t bus_raw = 0, shunt_raw = 0, current_raw = 0;
    if (!readReg16(REG_BUS_V, &bus_raw) || !readReg16(REG_SHUNT_V, &shunt_raw) ||
        !readReg16(REG_CURRENT, &current_raw)) {
        return result;
    }

    // Bit1 of the bus voltage register = math overflow (current/power
    // calculation invalid this cycle) - skip, caller just tries again later.
    if (bus_raw & 0x0002) {
        return result;
    }

    float bus_v = (float)(bus_raw >> 3) * 0.004f;        // LSB = 4mV after discarding status bits
    float shunt_mv = (float)(int16_t)shunt_raw * 0.01f;  // LSB = 10uV = 0.01mV
    float current_a = (float)(int16_t)current_raw * kCurrentLsbAPerBit;

    // "Load voltage": bus voltage plus the small drop across the shunt -
    // approximates the battery terminal voltage better than bus_v alone,
    // since bus_v is measured on the far side of the shunt from VBAT.
    float load_v = bus_v + (shunt_mv / 1000.0f);

    result.voltage_v = load_v;
    result.current_ma = current_a * 1000.0f;
    result.percent = percentFromVoltage(load_v);
    result.valid = true;
    return result;
}

} // namespace battery
