/*
 * TeR_IMU.c
 *
 *  Created on: Jan 9, 2025
 *      Author: eracing
 */
#include <TeR_INERTIAL.h>

//MARG devices
stmdev_ctx_t imu;
stmdev_ctx_t mag;

static uint8_t whoamI, rst; //Aux variables for operation

/* IMU variables ---------------------------------------------------------*/
static int16_t acc_raw[3];
static int16_t gy_raw[3];
static float_t acc_xyz[3]; //In m/s
static float_t a_rate_rpy[3]; //In deg/s

/* MAG variables ---------------------------------------------------------*/
static int16_t mag_raw[3];
static float_t mag_xyz[3];

/* Combined attitude ---------------------------------------------------------*/
float roll,pitch,yaw;



void inertial(void *argument) {
    uint32_t lastTick = osKernelGetTickCount(); // Initialize reference time
	configIMU();
	configMAG();

	for (;;) {
	    lastTick += TASK_PERIOD;
        osDelayUntil(lastTick);

		uint8_t reg;
		/* Read output only if new xl value is available */
		asm330lhh_xl_flag_data_ready_get(&imu, &reg);

		if (reg) {
			/* Read acceleration field data */
			memset(acc_raw, 0x00, 3 * sizeof(int16_t));
			asm330lhh_acceleration_raw_get(&imu, acc_raw);
			acc_xyz[0] = asm330lhh_from_fs2g_to_mg(
					acc_raw[0])/1000.0;
			acc_xyz[1] = asm330lhh_from_fs2g_to_mg(
					acc_raw[1])/1000.0;
			acc_xyz[2] = asm330lhh_from_fs2g_to_mg(
					acc_raw[2]/1000.0);

		}

		asm330lhh_gy_flag_data_ready_get(&imu, &reg);

		if (reg) {
			/* Read angular rate field data */
			memset(gy_raw, 0x00, 3 * sizeof(int16_t));
			asm330lhh_angular_rate_raw_get(&imu, gy_raw);
			a_rate_rpy[0] = asm330lhh_from_fs2000dps_to_mdps(
					gy_raw[0])/1000.0;
			a_rate_rpy[1] = asm330lhh_from_fs2000dps_to_mdps(
					gy_raw[1])/1000.0;
			a_rate_rpy[2] = asm330lhh_from_fs2000dps_to_mdps(
					gy_raw[2])/1000.0;
		}

		//-----------------------------------------------------------------------------------------------------//
		/* Read magnetic field data */
		lis3mdl_mag_data_ready_get(&mag, &reg);

		if (reg) {
			memset(mag_raw, 0x00, 3 * sizeof(int16_t));
			lis3mdl_magnetic_raw_get(&mag, mag_raw);
			mag_xyz[0] = 1000
					* lis3mdl_from_fs16_to_gauss(mag_raw[0]);
			mag_xyz[1] = 1000
					* lis3mdl_from_fs16_to_gauss(mag_raw[1]);
			mag_xyz[2] = 1000
					* lis3mdl_from_fs16_to_gauss(mag_raw[2]);
		}
		//Process attitude in euler angles
	    compFilter(a_rate_rpy[0], a_rate_rpy[1], a_rate_rpy[2], acc_xyz[0], acc_xyz[1], acc_xyz[2], mag_xyz[0], mag_xyz[1], mag_xyz[2], &roll, &pitch, &yaw);
	    printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n", roll, pitch, yaw, a_rate_rpy[0], a_rate_rpy[1], a_rate_rpy[2], acc_xyz[0], acc_xyz[1], acc_xyz[2]);

	}
}

void configIMU(void) {

	imu.write_reg = &imu_write;
	imu.read_reg = &imu_read;
	imu.handle = &hi2c1;
	/* Wait sensor boot time */

	osDelay(50);

	asm330lhh_device_id_get(&imu, &whoamI);
	if (whoamI != ASM330LHH_ID)
		while (1)
			;
	/* Restore default configuration */
	asm330lhh_reset_set(&imu, PROPERTY_ENABLE);

	do {
		asm330lhh_reset_get(&imu, &rst);
	} while (rst);

	//Turn on light to indicate IMU is running
	HAL_GPIO_WritePin(IMU_LED_GPIO_Port, IMU_LED_Pin, 1);

	asm330lhh_device_conf_set(&imu, PROPERTY_ENABLE);
	/* Enable Block Data Update */
	asm330lhh_block_data_update_set(&imu, PROPERTY_ENABLE);
	/* Set Output Data Rate */
	asm330lhh_xl_data_rate_set(&imu, ASM330LHH_XL_ODR_104Hz);
	asm330lhh_gy_data_rate_set(&imu, ASM330LHH_GY_ODR_104Hz);
	/* Set full scale */
	asm330lhh_xl_full_scale_set(&imu, ASM330LHH_2g);
	asm330lhh_gy_full_scale_set(&imu, ASM330LHH_2000dps);
	/* Configure filtering chain(No aux interface)
	 * Accelerometer - LPF1 + LPF2 path
	 */
	asm330lhh_xl_hp_path_on_out_set(&imu, ASM330LHH_LP_ODR_DIV_100);
	asm330lhh_xl_filter_lp2_set(&imu, PROPERTY_ENABLE);

}

void configMAG() {
	/* Initialize mems driver interface */
	mag.write_reg = &mag_write;
	mag.read_reg = &mag_read;
	mag.handle = &hi2c1;
	/* Check device ID */
	lis3mdl_device_id_get(&mag, &whoamI);

	if (whoamI != LIS3MDL_ID)
		while (1)
			; /*manage here device not found */

	/* Restore default configuration */
	lis3mdl_reset_set(&mag, PROPERTY_ENABLE);

	do {
		lis3mdl_reset_get(&mag, &rst);
	} while (rst);

	/* Enable Block Data Update */
	lis3mdl_block_data_update_set(&mag, PROPERTY_ENABLE);
	/* Set Output Data Rate */
	lis3mdl_data_rate_set(&mag, LIS3MDL_HP_20Hz);
	/* Set full scale */
	lis3mdl_full_scale_set(&mag, LIS3MDL_16_GAUSS);
	/* Enable temperature sensor */
	lis3mdl_temperature_meas_set(&mag, PROPERTY_ENABLE);
	/* Set device in continuous mode */
	lis3mdl_operating_mode_set(&mag, LIS3MDL_CONTINUOUS_MODE);

}

static int32_t imu_write(void *handle, uint8_t reg, const uint8_t *bufp,
		uint16_t len) {
	return HAL_I2C_Mem_Write(handle, ASM330LHH_I2C_ADD_L, reg,
	I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);
	return 0;
}

static int32_t imu_read(void *handle, uint8_t reg, uint8_t *bufp,
		uint16_t len) {
	return HAL_I2C_Mem_Read(handle, ASM330LHH_I2C_ADD_L, reg,
	I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
}

static int32_t mag_write(void *handle, uint8_t reg, const uint8_t *bufp,
		uint16_t len) {
	return HAL_I2C_Mem_Write(handle, LIS3MDL_I2C_ADD_L, reg,
	I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);
	return 0;
}

static int32_t mag_read(void *handle, uint8_t reg, uint8_t *bufp,
		uint16_t len) {
	return HAL_I2C_Mem_Read(handle, LIS3MDL_I2C_ADD_L, reg,
	I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
}

