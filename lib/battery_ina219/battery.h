/*****************************************************************************
 * | File        :   battery.h
 * | Function    :   Driver for an EXTERNAL INA219 current/voltage monitor,
 * |                 used to estimate the charge level of the single-cell
 * |                 LiPo battery plugged into the Waveshare
 * |                 ESP32-S3-Touch-LCD-4.3B board's MX1.25 battery connector
 * |                 (J5 on the official schematic).
 * |
 * | NOTE        :   The board's own battery charge manager (CS8501) only
 * |                 boosts the battery voltage to power the system - it does
 * |                 NOT expose battery voltage/charge to any GPIO (confirmed
 * |                 against the official ESP32-S3-Touch-LCD-4.3B schematic,
 * |                 "Battery" sheet: VBAT only reaches the J5 connector,
 * |                 through SW1 ("Battery Connection Switch" - item 10 in the
 * |                 official docs), the internal VBAT net at C50/C51, and
 * |                 the CS8501's own internal BAT pin). There is also no
 * |                 spare ADC-capable pin broken out on any connector of this
 * |                 board revision: the official schematic's WROOM-1/2 pin
 * |                 mapping table shows GPIO6 (the one pin that looked
 * |                 "free" from generic ESP32-S3-WROOM-1 docs) is actually
 * |                 wired to RTC_INT (the onboard PCF85063 RTC's interrupt
 * |                 pin) on THIS specific board, so it is not actually
 * |                 available either - confirmed from the official PDF
 * |                 schematic, not just the product page/wiki.
 * |
 * |                 Because of that, battery monitoring here relies on an
 * |                 EXTERNAL INA219 module wired in series with the LiPo's
 * |                 VBAT line - preferably spliced at the C50/C51 test
 * |                 points (i.e. AFTER SW1), so the reading reflects the
 * |                 board's internal VBAT net regardless of that switch's
 * |                 position - and connected to this board's external I2C
 * |                 terminal (shared bus with touch/CH422G/IMU/RTC - see
 * |                 i2c.h; physically it's the "SDA"/"SCL"/"VOUT"/"GND"
 * |                 pins on the board's big screw terminal, silkscreened
 * |                 "PORT1" - VOUT defaults to 3.3V via the H8 jumper).
 * |                 Without that external wiring, begin() simply
 * |                 fails to detect the sensor and the UI falls back to "no
 * |                 sensor" (same graceful-degradation pattern as
 * |                 imu_qmi8658 when the IMU chip is not populated on a
 * |                 given board/batch). Note that SW1 OFF (needed when
 * |                 debugging over USB per the official schematic note)
 * |                 disconnects the battery entirely, so the INA219 will
 * |                 correctly read ~0V/"no sensor-like" values in that case.
 ******************************************************************************/

#ifndef __BATTERY_H
#define __BATTERY_H

#include <stdint.h>
#include <stdbool.h>

namespace battery {

/**
 * @brief Latest battery reading/estimate.
 *
 * voltage_v  : estimated battery terminal voltage (bus voltage + shunt drop)
 * current_ma : positive = battery discharging into the system, negative =
 *              charging (depends on which way the shunt was wired in series)
 * percent    : 0-100, derived from a 1S LiPo open-circuit discharge curve
 *              approximation (see kDischargeCurve in battery.cpp) - NOT a
 *              true fuel-gauge measurement, just a voltage-based estimate.
 * valid      : false if the INA219 was never detected, or the last read
 *              failed/overflowed.
 */
struct Reading {
    float voltage_v = 0.0f;
    float current_ma = 0.0f;
    uint8_t percent = 0;
    bool valid = false;
};

/**
 * @brief Probe and configure the external INA219 on the shared I2C bus.
 *
 * @note The I2C bus (DEV_I2C_Init) must already be initialized before
 *       calling this - in this project that happens inside
 *       touch_gt911_init(), called earlier in setup().
 *
 * @return true if an INA219 responded at the expected address (kAddress in
 *         battery.cpp) and accepted its configuration.
 */
bool begin();

/**
 * @brief Read bus/shunt voltage and current, and estimate charge percentage.
 *
 * Call periodically (e.g. every few seconds - battery level does not need
 * IMU-like update rates).
 *
 * @return Reading with valid=false if the sensor was not detected/begin()
 *         was never called or failed.
 */
Reading read();

} // namespace battery

#endif // __BATTERY_H
