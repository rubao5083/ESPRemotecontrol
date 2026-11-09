/*****************************************************************************
 * | File        :   qmi8658.h
 * | Function    :   Driver for the onboard QMI8658 6-axis IMU
 * |                 (3-axis accelerometer + 3-axis gyroscope) present on the
 * |                 Waveshare ESP32-S3 Touch LCD 4.3" board (I2C, shared bus
 * |                 with the CH422G expander and the GT911 touch controller).
 * |
 * | NOTE        :   The QMI8658 does NOT contain a magnetometer, so it CANNOT
 * |                 provide a magnetic compass heading. Only inclination
 * |                 (pitch/roll) is available from this sensor. A true compass
 * |                 requires an external magnetometer module (e.g. QMC5883L)
 * |                 wired to the same I2C bus - not implemented yet.
 ******************************************************************************/

#ifndef __QMI8658_H
#define __QMI8658_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

namespace imu {

/**
 * @brief Attitude (inclination) reading, in degrees.
 *
 * roll_deg  : rotation around the front-back axis (side-to-side tilt)
 * pitch_deg : rotation around the left-right axis (nose up/down tilt)
 * heading_deg / has_heading : always false/NaN for now (no magnetometer on this board)
 */
struct Attitude {
    float roll_deg = 0.0f;
    float pitch_deg = 0.0f;
    float heading_deg = 0.0f;
    bool has_heading = false;
    bool valid = false;
};

/**
 * @brief Initialize the QMI8658 over I2C.
 *
 * @note The I2C bus (DEV_I2C_Init) must already be initialized before calling
 *       this function. In this project that happens inside touch_gt911_init().
 *
 * @return true if the sensor was detected (WHO_AM_I) and configured.
 */
bool begin();

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

#endif // __QMI8658_H
