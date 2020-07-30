#pragma once

#define HAL_BOARD_NAME "PX4"
#define HAL_CPU_CLASS HAL_CPU_CLASS_150
#define HAL_OS_POSIX_IO 1
#define HAL_BOARD_LOG_DIRECTORY "/fs/microsd/APM/LOGS"
#define HAL_BOARD_TERRAIN_DIRECTORY "/fs/microsd/APM/TERRAIN"
#define HAL_PARAM_DEFAULTS_PATH "/etc/defaults.parm"
#define HAL_INS_DEFAULT HAL_INS_PX4
#define HAL_BARO_DEFAULT HAL_BARO_PX4
#define HAL_COMPASS_DEFAULT HAL_COMPASS_PX4

#define HAL_HAVE_GETTIME_SETTIME 1

#ifdef CONFIG_ARCH_BOARD_PX4FMU_V1
#define CONFIG_HAL_BOARD_SUBTYPE HAL_BOARD_SUBTYPE_PX4_V1
#define HAL_STORAGE_SIZE            8192
#elif defined(CONFIG_ARCH_BOARD_PX4FMU_V3)
// check for V3 before V2 as V3 also defines V2
#define CONFIG_HAL_BOARD_SUBTYPE HAL_BOARD_SUBTYPE_PX4_V3
#define HAL_STORAGE_SIZE            16384
#define HAL_HAVE_IMU_HEATER         1 // for Pixhawk2
#define HAL_IMU_TEMP_DEFAULT       -1 // disabled
#define HAL_WITH_UAVCAN             1
#elif defined(CONFIG_ARCH_BOARD_PX4FMU_V2)
#define CONFIG_HAL_BOARD_SUBTYPE HAL_BOARD_SUBTYPE_PX4_V2
#define HAL_STORAGE_SIZE            16384
#define HAL_HAVE_IMU_HEATER         1 // for Pixhawk2
#define HAL_IMU_TEMP_DEFAULT       -1 // disabled
#define HAL_WITH_UAVCAN             1
#define HAL_MINIMIZE_FEATURES       1
#elif defined(CONFIG_ARCH_BOARD_PX4FMU_V4)
#define CONFIG_HAL_BOARD_SUBTYPE HAL_BOARD_SUBTYPE_PX4_V4
#define HAL_STORAGE_SIZE            16384
#define HAL_WITH_UAVCAN				1
#elif defined(CONFIG_ARCH_BOARD_PX4FMU_V4PRO)
#define CONFIG_HAL_BOARD_SUBTYPE HAL_BOARD_SUBTYPE_PX4_V4PRO
#define HAL_STORAGE_SIZE            16384
#define HAL_WITH_UAVCAN				1
#elif defined(CONFIG_ARCH_BOARD_AEROFC_V1)
#define CONFIG_HAL_BOARD_SUBTYPE HAL_BOARD_SUBTYPE_PX4_AEROFC_V1
#define HAL_STORAGE_SIZE            16384
#define USE_FLASH_STORAGE           1
#define HAL_RCOUTPUT_TAP_DEVICE "/dev/ttyS0"
// we don't have any sdcard
#undef HAL_BOARD_LOG_DIRECTORY
#undef HAL_BOARD_TERRAIN_DIRECTORY
#else
#error "Unknown PX4 board type"
#endif

#define HAL_GPIO_A_LED_PIN        27
#define HAL_GPIO_B_LED_PIN        26
#define HAL_GPIO_C_LED_PIN        25
#define HAL_GPIO_LED_ON           1
#define HAL_GPIO_LED_OFF          0

#define HAL_BARO_MS5611_NAME "ms5611"
#define HAL_BARO_MS5611_SPI_INT_NAME "ms5611_int"
#define HAL_BARO_MS5611_SPI_EXT_NAME "ms5611_ext"

#define HAL_INS_MPU60x0_NAME "mpu6000"
#define HAL_INS_MPU60x0_EXT_NAME "mpu6000_ext"

#define HAL_INS_LSM9DS0_G_NAME "lsm9ds0_g"
#define HAL_INS_LSM9DS0_A_NAME "lsm9ds0_am"

#define HAL_INS_LSM9DS0_EXT_G_NAME "lsm9ds0_ext_g"
#define HAL_INS_LSM9DS0_EXT_A_NAME "lsm9ds0_ext_am"

#define HAL_INS_MPU9250_NAME "mpu9250"
#define HAL_INS_MPU9250_EXT_NAME "mpu9250_ext"

#define HAL_INS_MPU6500_NAME "mpu6500"

#define HAL_INS_ICM20608_NAME "icm20608"
#define HAL_INS_ICM20608_AM_NAME "icm20608-am"
#define HAL_INS_ICM20608_EXT_NAME "icm20608_ext"

#define HAL_COMPASS_HMC5843_NAME "hmc5843"
#define HAL_COMPASS_LIS3MDL_NAME "lis3mdl"

/* px4fmu-v1 */
#ifdef CONFIG_ARCH_BOARD_PX4FMU_V1
#define HAL_BARO_MS5611_I2C_ADDR 0x76
#define HAL_BARO_MS5611_I2C_BUS  0
#define HAL_HAVE_BOARD_VOLTAGE 0
#define HAL_PX4_HAVE_PWM_INPUT 0
#endif

/* px4fmu-v4 */
#ifdef CONFIG_ARCH_BOARD_PX4FMU_V4
#define HAL_PX4_HAVE_PX4IO 0
#endif

/* aerofc-v1 */
#ifdef CONFIG_ARCH_BOARD_AEROFC_V1
#define HAL_BARO_MS5607_I2C_ADDR 0x77
#define HAL_BARO_MS5607_I2C_BUS  0
#define HAL_COMPASS_IST8310_I2C_ADDR 0x0E
#define HAL_COMPASS_IST8310_I2C_BUS 1
#define HAL_SERIAL0_BAUD_DEFAULT 921600

#define HAL_HAVE_BOARD_VOLTAGE 0
#define HAL_HAVE_SAFETY_SWITCH 0
#define HAL_PX4_HAVE_MTD_SUPPORT 0
#define HAL_PX4_HAVE_PX4IO 0
#define HAL_PX4_HAVE_PWM_INPUT 0
#endif

/* default values */
#ifndef HAL_SERIAL0_BAUD_DEFAULT
#define HAL_SERIAL0_BAUD_DEFAULT 115200
#endif

#ifndef HAL_HAVE_BOARD_VOLTAGE
#define HAL_HAVE_BOARD_VOLTAGE 1
#endif

#ifndef HAL_PX4_HAVE_MTD_SUPPORT
#define HAL_PX4_HAVE_MTD_SUPPORT 1
#endif

#ifndef HAL_PX4_HAVE_PX4IO
#define HAL_PX4_HAVE_PX4IO 1
#endif

#ifndef HAL_PX4_HAVE_PWM_INPUT
#define HAL_PX4_HAVE_PWM_INPUT 1
#endif

#ifndef HAL_HAVE_SAFETY_SWITCH
#define HAL_HAVE_SAFETY_SWITCH 1
#endif

#ifndef AP_FEATURE_RTSCTS
#define AP_FEATURE_RTSCTS 1
#endif

#ifndef AP_FEATURE_SBUS_OUT
#define AP_FEATURE_SBUS_OUT 1
#endif
