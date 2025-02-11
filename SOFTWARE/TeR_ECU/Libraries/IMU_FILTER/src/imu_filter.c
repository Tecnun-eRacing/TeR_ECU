/*
 * imu_filter.c
 *
 *  Created on: Feb 2, 2025
 *      Author: eracing
 */
#include "imu_filter.h"


// Function to update roll, pitch, yaw using complementary filter
void compFilter(float gx, float gy, float gz, float ax, float ay, float az,
                         float mx, float my, float mz, float *roll, float *pitch, float *yaw) {
    // Convert gyroscope degrees/sec to radians/sec
    gx *= M_PI / 180.0f;
    gy *= M_PI / 180.0f;
    gz *= M_PI / 180.0f;

    // Compute roll and pitch from accelerometer (gravity vector)
    float accelRoll  = atan2f(ay, az) * 180.0f / M_PI;
    float accelPitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / M_PI;

    // Integrate gyroscope data
    *roll  = ALPHA * (*roll + gx * DT) + (1 - ALPHA) * accelRoll;
    *pitch = ALPHA * (*pitch + gy * DT) + (1 - ALPHA) * accelPitch;

    // Compute yaw from magnetometer (only needed if magnetometer is used)
    float magYaw = atan2f(-my, mx) * 180.0f / M_PI;
    *yaw = ALPHA * (*yaw + gz * DT) + (1 - ALPHA) * magYaw;
}
