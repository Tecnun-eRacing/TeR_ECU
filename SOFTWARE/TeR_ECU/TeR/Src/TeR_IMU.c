/*
 * TeR_IMU.c
 *
 *  Created on: Jan 9, 2025
 *      Author: eracing
 */
#include "TeR_IMU.h"

/* Private variables ---------------------------------------------------------*/
static int16_t data_raw_acceleration[3];
static int16_t data_raw_angular_rate[3];
static int16_t data_raw_temperature;
static float_t acceleration_mg[3];
static float_t angular_rate_mdps[3];
static float_t temperature_degC;
static uint8_t whoamI, rst;


asm330lhh_ctrl3_c_t ctrl3_c;


//Private function prototypes
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
		uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
		uint16_t len);








void imu(void *argument) {

	stmdev_ctx_t dev_ctx;
	dev_ctx.write_reg = platform_write;
	dev_ctx.read_reg = platform_read;
	dev_ctx.handle = &hi2c1;
	/* Wait sensor boot time */

	osDelay(50);

	asm330lhh_device_id_get(&dev_ctx, &whoamI);
	//Check id
	if (whoamI != ASM330LHH_ID)
		while (1)
			;
	/* Restore default configuration */
	asm330lhh_reset_set(&dev_ctx, PROPERTY_ENABLE);

	do {
		asm330lhh_reset_get(&dev_ctx, &rst);
	} while (rst);
	HAL_GPIO_WritePin(IMU_LED_GPIO_Port,IMU_LED_Pin, 1);

	asm330lhh_device_conf_set(&dev_ctx, PROPERTY_ENABLE);
	/* Enable Block Data Update */
	asm330lhh_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
	/* Set Output Data Rate */
	asm330lhh_xl_data_rate_set(&dev_ctx, ASM330LHH_XL_ODR_104Hz);
	asm330lhh_gy_data_rate_set(&dev_ctx, ASM330LHH_GY_ODR_104Hz);
	/* Set full scale */
	asm330lhh_xl_full_scale_set(&dev_ctx, ASM330LHH_2g);
	asm330lhh_gy_full_scale_set(&dev_ctx, ASM330LHH_2000dps);
	/* Configure filtering chain(No aux interface)
	 * Accelerometer - LPF1 + LPF2 path
	 */
	asm330lhh_xl_hp_path_on_out_set(&dev_ctx, ASM330LHH_LP_ODR_DIV_100);
	asm330lhh_xl_filter_lp2_set(&dev_ctx, PROPERTY_ENABLE);



	for (;;) {
		osDelay(5);
		   uint8_t reg;
		    /* Read output only if new xl value is available */
		    asm330lhh_xl_flag_data_ready_get(&dev_ctx, &reg);

		    if (reg) {
		      /* Read acceleration field data */
		      memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
		      asm330lhh_acceleration_raw_get(&dev_ctx, data_raw_acceleration);
		      acceleration_mg[0] =
		        asm330lhh_from_fs2g_to_mg(data_raw_acceleration[0]);
		      acceleration_mg[1] =
		        asm330lhh_from_fs2g_to_mg(data_raw_acceleration[1]);
		      acceleration_mg[2] =
		        asm330lhh_from_fs2g_to_mg(data_raw_acceleration[2]);

		    }

		    asm330lhh_gy_flag_data_ready_get(&dev_ctx, &reg);

		    if (reg) {
		      /* Read angular rate field data */
		      memset(data_raw_angular_rate, 0x00, 3 * sizeof(int16_t));
		      asm330lhh_angular_rate_raw_get(&dev_ctx, data_raw_angular_rate);
		      angular_rate_mdps[0] =
		        asm330lhh_from_fs2000dps_to_mdps(data_raw_angular_rate[0]);
		      angular_rate_mdps[1] =
		        asm330lhh_from_fs2000dps_to_mdps(data_raw_angular_rate[1]);
		      angular_rate_mdps[2] =
		        asm330lhh_from_fs2000dps_to_mdps(data_raw_angular_rate[2]);
		    }

		    asm330lhh_temp_flag_data_ready_get(&dev_ctx, &reg);

		    if (reg) {
		      /* Read temperature data */
		      memset(&data_raw_temperature, 0x00, sizeof(int16_t));
		      asm330lhh_temperature_raw_get(&dev_ctx, &data_raw_temperature);
		      temperature_degC = asm330lhh_from_lsb_to_celsius(
		                           data_raw_temperature);
		    }





		  }
	}

static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
		uint16_t len) {
	return HAL_I2C_Mem_Write(handle, ASM330LHH_I2C_ADD_L, reg, I2C_MEMADD_SIZE_8BIT,
			(uint8_t*) bufp, len, 1000);
	return 0;
}

static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
		uint16_t len) {
	return HAL_I2C_Mem_Read(handle, ASM330LHH_I2C_ADD_L, reg, I2C_MEMADD_SIZE_8BIT,
			bufp, len, 1000);
}

