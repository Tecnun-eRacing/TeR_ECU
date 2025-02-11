/*
 * imu_filter.h
 *
 *  Created on: Feb 2, 2025
 *      Author: eracing
 */

#ifndef IMU_FILTER_INC_IMU_FILTER_H_
#define IMU_FILTER_INC_IMU_FILTER_H_

#include <math.h>

// Complementary filter constant (adjust as needed)
#define ALPHA 0.6
#define DT 0.01  // Time step (e.g., 10ms)

void compFilter(float gx, float gy, float gz, float ax, float ay, float az,
                         float mx, float my, float mz, float *roll, float *pitch, float *yaw);



#endif /* IMU_FILTER_INC_IMU_FILTER_H_ */
