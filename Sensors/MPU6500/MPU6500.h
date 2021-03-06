#ifndef _MPU6500_H_
#define _MPU6500_H_

/*--- Includes. ---*/
#include "stm32f0xx_hal.h"

#include <string.h>
#include <math.h>

#include <DataTypes/SensorTypes.h>

/**
 * This file was originally got from http://www.cs.cmu.edu/~cga/arduino/MPU6500.h.
 * The functions and the setup procedure was based on the files at https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050
 */

#define MPU6500_ADDRESS_AD0_LOW     0x68 // address pin low (GND), default for InvenSense evaluation board
#define MPU6500_ADDRESS_AD0_HIGH    0x69 // address pin high (VCC)
#define MPU6500_DEFAULT_ADDRESS     MPU6500_ADDRESS_AD0_LOW

#define MPU6500_RA_XG_OFFS_TC       0x00 //[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6500_RA_YG_OFFS_TC       0x01 //[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6500_RA_ZG_OFFS_TC       0x02 //[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6500_RA_X_FINE_GAIN      0x03 //[7:0] X_FINE_GAIN
#define MPU6500_RA_Y_FINE_GAIN      0x04 //[7:0] Y_FINE_GAIN
#define MPU6500_RA_Z_FINE_GAIN      0x05 //[7:0] Z_FINE_GAIN
#define MPU6500_RA_XA_OFFS_H        0x06 //[15:0] XA_OFFS
#define MPU6500_RA_XA_OFFS_L_TC     0x07
#define MPU6500_RA_YA_OFFS_H        0x08 //[15:0] YA_OFFS
#define MPU6500_RA_YA_OFFS_L_TC     0x09
#define MPU6500_RA_ZA_OFFS_H        0x0A //[15:0] ZA_OFFS
#define MPU6500_RA_ZA_OFFS_L_TC     0x0B
#define MPU6500_RA_XG_OFFS_USRH     0x13 //[15:0] XG_OFFS_USR
#define MPU6500_RA_XG_OFFS_USRL     0x14
#define MPU6500_RA_YG_OFFS_USRH     0x15 //[15:0] YG_OFFS_USR
#define MPU6500_RA_YG_OFFS_USRL     0x16
#define MPU6500_RA_ZG_OFFS_USRH     0x17 //[15:0] ZG_OFFS_USR
#define MPU6500_RA_ZG_OFFS_USRL     0x18
#define MPU6500_RA_SMPLRT_DIV       0x19
#define MPU6500_RA_CONFIG           0x1A
#define MPU6500_RA_GYRO_CONFIG      0x1B
#define MPU6500_RA_ACCEL_CONFIG     0x1C
#define MPU6500_RA_ACCEL_CONFIG_2   0x1D
#define MPU6500_LP_ACCEL_ODR        0x1E

#define MPU6500_RA_MOT_THR          0x1F

#define MPU6500_RA_FIFO_EN          0x23
#define MPU6500_RA_I2C_MST_CTRL     0x24
#define MPU6500_RA_I2C_SLV0_ADDR    0x25
#define MPU6500_RA_I2C_SLV0_REG     0x26
#define MPU6500_RA_I2C_SLV0_CTRL    0x27
#define MPU6500_RA_I2C_SLV1_ADDR    0x28
#define MPU6500_RA_I2C_SLV1_REG     0x29
#define MPU6500_RA_I2C_SLV1_CTRL    0x2A
#define MPU6500_RA_I2C_SLV2_ADDR    0x2B
#define MPU6500_RA_I2C_SLV2_REG     0x2C
#define MPU6500_RA_I2C_SLV2_CTRL    0x2D
#define MPU6500_RA_I2C_SLV3_ADDR    0x2E
#define MPU6500_RA_I2C_SLV3_REG     0x2F
#define MPU6500_RA_I2C_SLV3_CTRL    0x30
#define MPU6500_RA_I2C_SLV4_ADDR    0x31
#define MPU6500_RA_I2C_SLV4_REG     0x32
#define MPU6500_RA_I2C_SLV4_DO      0x33
#define MPU6500_RA_I2C_SLV4_CTRL    0x34
#define MPU6500_RA_I2C_SLV4_DI      0x35
#define MPU6500_RA_I2C_MST_STATUS   0x36
#define MPU6500_RA_INT_PIN_CFG      0x37
#define MPU6500_RA_INT_ENABLE       0x38
#define MPU6500_RA_DMP_INT_STATUS   0x39
#define MPU6500_RA_INT_STATUS       0x3A
#define MPU6500_RA_ACCEL_XOUT_H     0x3B
#define MPU6500_RA_ACCEL_XOUT_L     0x3C
#define MPU6500_RA_ACCEL_YOUT_H     0x3D
#define MPU6500_RA_ACCEL_YOUT_L     0x3E
#define MPU6500_RA_ACCEL_ZOUT_H     0x3F
#define MPU6500_RA_ACCEL_ZOUT_L     0x40
#define MPU6500_RA_TEMP_OUT_H       0x41
#define MPU6500_RA_TEMP_OUT_L       0x42
#define MPU6500_RA_GYRO_XOUT_H      0x43
#define MPU6500_RA_GYRO_XOUT_L      0x44
#define MPU6500_RA_GYRO_YOUT_H      0x45
#define MPU6500_RA_GYRO_YOUT_L      0x46
#define MPU6500_RA_GYRO_ZOUT_H      0x47
#define MPU6500_RA_GYRO_ZOUT_L      0x48
#define MPU6500_RA_EXT_SENS_DATA_00 0x49
#define MPU6500_RA_EXT_SENS_DATA_01 0x4A
#define MPU6500_RA_EXT_SENS_DATA_02 0x4B
#define MPU6500_RA_EXT_SENS_DATA_03 0x4C
#define MPU6500_RA_EXT_SENS_DATA_04 0x4D
#define MPU6500_RA_EXT_SENS_DATA_05 0x4E
#define MPU6500_RA_EXT_SENS_DATA_06 0x4F
#define MPU6500_RA_EXT_SENS_DATA_07 0x50
#define MPU6500_RA_EXT_SENS_DATA_08 0x51
#define MPU6500_RA_EXT_SENS_DATA_09 0x52
#define MPU6500_RA_EXT_SENS_DATA_10 0x53
#define MPU6500_RA_EXT_SENS_DATA_11 0x54
#define MPU6500_RA_EXT_SENS_DATA_12 0x55
#define MPU6500_RA_EXT_SENS_DATA_13 0x56
#define MPU6500_RA_EXT_SENS_DATA_14 0x57
#define MPU6500_RA_EXT_SENS_DATA_15 0x58
#define MPU6500_RA_EXT_SENS_DATA_16 0x59
#define MPU6500_RA_EXT_SENS_DATA_17 0x5A
#define MPU6500_RA_EXT_SENS_DATA_18 0x5B
#define MPU6500_RA_EXT_SENS_DATA_19 0x5C
#define MPU6500_RA_EXT_SENS_DATA_20 0x5D
#define MPU6500_RA_EXT_SENS_DATA_21 0x5E
#define MPU6500_RA_EXT_SENS_DATA_22 0x5F
#define MPU6500_RA_EXT_SENS_DATA_23 0x60
#define MPU6500_RA_MOT_DETECT_STATUS    0x61
#define MPU6500_RA_I2C_SLV0_DO      0x63
#define MPU6500_RA_I2C_SLV1_DO      0x64
#define MPU6500_RA_I2C_SLV2_DO      0x65
#define MPU6500_RA_I2C_SLV3_DO      0x66
#define MPU6500_RA_I2C_MST_DELAY_CTRL   0x67
#define MPU6500_RA_SIGNAL_PATH_RESET    0x68
#define MPU6500_RA_MOT_DETECT_CTRL      0x69
#define MPU6500_RA_USER_CTRL        0x6A
#define MPU6500_RA_PWR_MGMT_1       0x6B
#define MPU6500_RA_PWR_MGMT_2       0x6C
#define MPU6500_RA_BANK_SEL         0x6D
#define MPU6500_RA_MEM_START_ADDR   0x6E
#define MPU6500_RA_MEM_R_W          0x6F
#define MPU6500_RA_DMP_CFG_1        0x70
#define MPU6500_RA_DMP_CFG_2        0x71
#define MPU6500_RA_FIFO_COUNTH      0x72
#define MPU6500_RA_FIFO_COUNTL      0x73
#define MPU6500_RA_FIFO_R_W         0x74
#define MPU6500_RA_WHO_AM_I         0x75
#define MPU6500_RA_FIFO_COUNT_H     114
#define MPU6500_RA_FIFO_COUNT_L     115

#define MPU6500_TC_PWR_MODE_BIT     7
#define MPU6500_TC_OFFSET_BIT       6
#define MPU6500_TC_OFFSET_LENGTH    6
#define MPU6500_TC_OTP_BNK_VLD_BIT  0

#define MPU6500_VDDIO_LEVEL_VLOGIC  0
#define MPU6500_VDDIO_LEVEL_VDD     1

#define MPU6500_CFG_EXT_SYNC_SET_BIT    5
#define MPU6500_CFG_EXT_SYNC_SET_LENGTH 3
#define MPU6500_CFG_DLPF_CFG_BIT    2
#define MPU6500_CFG_DLPF_CFG_LENGTH 3

#define MPU6500_EXT_SYNC_DISABLED       0x0
#define MPU6500_EXT_SYNC_TEMP_OUT_L     0x1
#define MPU6500_EXT_SYNC_GYRO_XOUT_L    0x2
#define MPU6500_EXT_SYNC_GYRO_YOUT_L    0x3
#define MPU6500_EXT_SYNC_GYRO_ZOUT_L    0x4
#define MPU6500_EXT_SYNC_ACCEL_XOUT_L   0x5
#define MPU6500_EXT_SYNC_ACCEL_YOUT_L   0x6
#define MPU6500_EXT_SYNC_ACCEL_ZOUT_L   0x7

#define MPU6500_DLPF_BW_256         0x00
#define MPU6500_DLPF_BW_188         0x01
#define MPU6500_DLPF_BW_98          0x02
#define MPU6500_DLPF_BW_42          0x03
#define MPU6500_DLPF_BW_20          0x04
#define MPU6500_DLPF_BW_10          0x05
#define MPU6500_DLPF_BW_5           0x06

#define MPU6500_GCONFIG_FCHOICE_B_BIT 		1
#define MPU6500_GCONFIG_FCHOICE_B_LENGTH 	2

#define MPU6500_GCONFIG_FS_SEL_BIT      4
#define MPU6500_GCONFIG_FS_SEL_LENGTH   2

#define MPU6500_ACONFIG_FCHOICE_B_BIT 	3
#define MPU6500_ACONFIG_DLPF_BIT		2
#define MPU6500_ACONFIG_DLPF_LENGTH		3

#define MPU6500_ACONFIG_XA_ST_BIT           7
#define MPU6500_ACONFIG_YA_ST_BIT           6
#define MPU6500_ACONFIG_ZA_ST_BIT           5
#define MPU6500_ACONFIG_AFS_SEL_BIT         4
#define MPU6500_ACONFIG_AFS_SEL_LENGTH      2
#define MPU6500_ACONFIG_ACCEL_HPF_BIT       2
#define MPU6500_ACONFIG_ACCEL_HPF_LENGTH    3

#define MPU6500_DHPF_RESET          0x00
#define MPU6500_DHPF_5              0x01
#define MPU6500_DHPF_2P5            0x02
#define MPU6500_DHPF_1P25           0x03
#define MPU6500_DHPF_0P63           0x04
#define MPU6500_DHPF_HOLD           0x07

#define MPU6500_TEMP_FIFO_EN_BIT    7
#define MPU6500_XG_FIFO_EN_BIT      6
#define MPU6500_YG_FIFO_EN_BIT      5
#define MPU6500_ZG_FIFO_EN_BIT      4
#define MPU6500_ACCEL_FIFO_EN_BIT   3
#define MPU6500_SLV2_FIFO_EN_BIT    2
#define MPU6500_SLV1_FIFO_EN_BIT    1
#define MPU6500_SLV0_FIFO_EN_BIT    0

#define MPU6500_FIFO_EN_TEMP 0x80
#define MPU6500_FIFO_EN_GYROX 0x40
#define MPU6500_FIFO_EN_GYROY 0x20
#define MPU6500_FIFO_EN_GYROZ 0x10
#define MPU6500_FIFO_EN_GYRO 0x70
#define MPU6500_FIFO_EN_ACC 0x08

#define MPU6500_MULT_MST_EN_BIT     7
#define MPU6500_WAIT_FOR_ES_BIT     6
#define MPU6500_SLV_3_FIFO_EN_BIT   5
#define MPU6500_I2C_MST_P_NSR_BIT   4
#define MPU6500_I2C_MST_CLK_BIT     3
#define MPU6500_I2C_MST_CLK_LENGTH  4

#define MPU6500_CLOCK_DIV_348       0x0
#define MPU6500_CLOCK_DIV_333       0x1
#define MPU6500_CLOCK_DIV_320       0x2
#define MPU6500_CLOCK_DIV_308       0x3
#define MPU6500_CLOCK_DIV_296       0x4
#define MPU6500_CLOCK_DIV_286       0x5
#define MPU6500_CLOCK_DIV_276       0x6
#define MPU6500_CLOCK_DIV_267       0x7
#define MPU6500_CLOCK_DIV_258       0x8
#define MPU6500_CLOCK_DIV_500       0x9
#define MPU6500_CLOCK_DIV_471       0xA
#define MPU6500_CLOCK_DIV_444       0xB
#define MPU6500_CLOCK_DIV_421       0xC
#define MPU6500_CLOCK_DIV_400       0xD
#define MPU6500_CLOCK_DIV_381       0xE
#define MPU6500_CLOCK_DIV_364       0xF

#define MPU6500_I2C_SLV_RW_BIT      7
#define MPU6500_I2C_SLV_ADDR_BIT    6
#define MPU6500_I2C_SLV_ADDR_LENGTH 7
#define MPU6500_I2C_SLV_EN_BIT      7
#define MPU6500_I2C_SLV_BYTE_SW_BIT 6
#define MPU6500_I2C_SLV_REG_DIS_BIT 5
#define MPU6500_I2C_SLV_GRP_BIT     4
#define MPU6500_I2C_SLV_LEN_BIT     3
#define MPU6500_I2C_SLV_LEN_LENGTH  4

#define MPU6500_I2C_SLV4_RW_BIT         7
#define MPU6500_I2C_SLV4_ADDR_BIT       6
#define MPU6500_I2C_SLV4_ADDR_LENGTH    7
#define MPU6500_I2C_SLV4_EN_BIT         7
#define MPU6500_I2C_SLV4_INT_EN_BIT     6
#define MPU6500_I2C_SLV4_REG_DIS_BIT    5
#define MPU6500_I2C_SLV4_MST_DLY_BIT    4
#define MPU6500_I2C_SLV4_MST_DLY_LENGTH 5

#define MPU6500_MST_PASS_THROUGH_BIT    7
#define MPU6500_MST_I2C_SLV4_DONE_BIT   6
#define MPU6500_MST_I2C_LOST_ARB_BIT    5
#define MPU6500_MST_I2C_SLV4_NACK_BIT   4
#define MPU6500_MST_I2C_SLV3_NACK_BIT   3
#define MPU6500_MST_I2C_SLV2_NACK_BIT   2
#define MPU6500_MST_I2C_SLV1_NACK_BIT   1
#define MPU6500_MST_I2C_SLV0_NACK_BIT   0

#define MPU6500_INTCFG_INT_LEVEL_BIT        7
#define MPU6500_INTCFG_INT_OPEN_BIT         6
#define MPU6500_INTCFG_LATCH_INT_EN_BIT     5
#define MPU6500_INTCFG_INT_RD_CLEAR_BIT     4
#define MPU6500_INTCFG_FSYNC_INT_LEVEL_BIT  3
#define MPU6500_INTCFG_FSYNC_INT_EN_BIT     2
#define MPU6500_INTCFG_I2C_BYPASS_EN_BIT    1
#define MPU6500_INTCFG_CLKOUT_EN_BIT        0

#define MPU6500_INTMODE_ACTIVEHIGH  0x00
#define MPU6500_INTMODE_ACTIVELOW   0x01

#define MPU6500_INTDRV_PUSHPULL     0x00
#define MPU6500_INTDRV_OPENDRAIN    0x01

#define MPU6500_INTLATCH_50USPULSE  0x00
#define MPU6500_INTLATCH_WAITCLEAR  0x01

#define MPU6500_INTCLEAR_STATUSREAD 0x00
#define MPU6500_INTCLEAR_ANYREAD    0x01

#define MPU6500_INTERRUPT_FF_BIT            7
#define MPU6500_INTERRUPT_MOT_BIT           6
#define MPU6500_INTERRUPT_ZMOT_BIT          5
#define MPU6500_INTERRUPT_FIFO_OFLOW_BIT    4
#define MPU6500_INTERRUPT_I2C_MST_INT_BIT   3
#define MPU6500_INTERRUPT_PLL_RDY_INT_BIT   2
#define MPU6500_INTERRUPT_DMP_INT_BIT       1
#define MPU6500_INTERRUPT_DATA_RDY_BIT      0

// TODO: figure out what these actually do
// UMPL source code is not very obivous
#define MPU6500_DMPINT_5_BIT            5
#define MPU6500_DMPINT_4_BIT            4
#define MPU6500_DMPINT_3_BIT            3
#define MPU6500_DMPINT_2_BIT            2
#define MPU6500_DMPINT_1_BIT            1
#define MPU6500_DMPINT_0_BIT            0

#define MPU6500_MOTION_MOT_XNEG_BIT     7
#define MPU6500_MOTION_MOT_XPOS_BIT     6
#define MPU6500_MOTION_MOT_YNEG_BIT     5
#define MPU6500_MOTION_MOT_YPOS_BIT     4
#define MPU6500_MOTION_MOT_ZNEG_BIT     3
#define MPU6500_MOTION_MOT_ZPOS_BIT     2
#define MPU6500_MOTION_MOT_ZRMOT_BIT    0

#define MPU6500_DELAYCTRL_DELAY_ES_SHADOW_BIT   7
#define MPU6500_DELAYCTRL_I2C_SLV4_DLY_EN_BIT   4
#define MPU6500_DELAYCTRL_I2C_SLV3_DLY_EN_BIT   3
#define MPU6500_DELAYCTRL_I2C_SLV2_DLY_EN_BIT   2
#define MPU6500_DELAYCTRL_I2C_SLV1_DLY_EN_BIT   1
#define MPU6500_DELAYCTRL_I2C_SLV0_DLY_EN_BIT   0

#define MPU6500_PATHRESET_GYRO_RESET_BIT    2
#define MPU6500_PATHRESET_ACCEL_RESET_BIT   1
#define MPU6500_PATHRESET_TEMP_RESET_BIT    0

#define MPU6500_DETECT_ACCEL_ON_DELAY_BIT       5
#define MPU6500_DETECT_ACCEL_ON_DELAY_LENGTH    2
#define MPU6500_DETECT_FF_COUNT_BIT             3
#define MPU6500_DETECT_FF_COUNT_LENGTH          2
#define MPU6500_DETECT_MOT_COUNT_BIT            1
#define MPU6500_DETECT_MOT_COUNT_LENGTH         2

#define MPU6500_DETECT_DECREMENT_RESET  0x0
#define MPU6500_DETECT_DECREMENT_1      0x1
#define MPU6500_DETECT_DECREMENT_2      0x2
#define MPU6500_DETECT_DECREMENT_4      0x3

#define MPU6500_USERCTRL_DMP_EN_BIT             7
#define MPU6500_USERCTRL_FIFO_EN_BIT            6
#define MPU6500_USERCTRL_I2C_MST_EN_BIT         5
#define MPU6500_USERCTRL_I2C_IF_DIS_BIT         4
#define MPU6500_USERCTRL_DMP_RESET_BIT          3
#define MPU6500_USERCTRL_FIFO_RESET_BIT         2
#define MPU6500_USERCTRL_I2C_MST_RESET_BIT      1
#define MPU6500_USERCTRL_SIG_COND_RESET_BIT     0

#define MPU6500_PWR1_DEVICE_RESET_BIT   7
#define MPU6500_PWR1_SLEEP_BIT          6
#define MPU6500_PWR1_CYCLE_BIT          5
#define MPU6500_PWR1_TEMP_DIS_BIT       3
#define MPU6500_PWR1_CLKSEL_BIT         2
#define MPU6500_PWR1_CLKSEL_LENGTH      3

#define MPU6500_CLOCK_INTERNAL          0x00
#define MPU6500_CLOCK_PLL_XGYRO         0x01
#define MPU6500_CLOCK_PLL_YGYRO         0x02
#define MPU6500_CLOCK_PLL_ZGYRO         0x03
#define MPU6500_CLOCK_PLL_EXT32K        0x04
#define MPU6500_CLOCK_PLL_EXT19M        0x05
#define MPU6500_CLOCK_KEEP_RESET        0x07

#define MPU6500_PWR2_LP_WAKE_CTRL_BIT       7
#define MPU6500_PWR2_LP_WAKE_CTRL_LENGTH    2
#define MPU6500_PWR2_STBY_XA_BIT            5
#define MPU6500_PWR2_STBY_YA_BIT            4
#define MPU6500_PWR2_STBY_ZA_BIT            3
#define MPU6500_PWR2_STBY_XG_BIT            2
#define MPU6500_PWR2_STBY_YG_BIT            1
#define MPU6500_PWR2_STBY_ZG_BIT            0

#define MPU6500_WAKE_FREQ_1P25      0x0
#define MPU6500_WAKE_FREQ_2P5       0x1
#define MPU6500_WAKE_FREQ_5         0x2
#define MPU6500_WAKE_FREQ_10        0x3

#define MPU6500_BANKSEL_PRFTCH_EN_BIT       6
#define MPU6500_BANKSEL_CFG_USER_BANK_BIT   5
#define MPU6500_BANKSEL_MEM_SEL_BIT         4
#define MPU6500_BANKSEL_MEM_SEL_LENGTH      5

#define MPU6500_WHO_AM_I_BIT        6
#define MPU6500_WHO_AM_I_LENGTH     6

#define MPU6500_DMP_MEMORY_BANKS        8
#define MPU6500_DMP_MEMORY_BANK_SIZE    256
#define MPU6500_DMP_MEMORY_CHUNK_SIZE   16

#define MPU6500_RA_SELF_TEST_X 0x00
#define MPU6500_RA_SELF_TEST_Y 0x00
#define MPU6500_RA_SELF_TEST_Z 0x00
#define MPU6500_RA_SELF_TEST_A 0x00

/*-- This section is dedicated for some Setups. --*/
#define BUFFER_SIZE 30

namespace Sensors {
class MPU6500 {

public:
	enum State {
		UNKNOWN = 0x00,
		DISCONNECTED = 0x01,
		NOT_INITIALIZED = 0x02,
		CONFIG_MODE = 0x03,
		RUNNING = 0x04,
		SELF_TEST = 0x05,	//Self testing, although do not discriminate between accel or gyro self test
		ERROR = 0xFF
	};

	enum AccelRange {
		SCALE_2_G = 0x00,
		SCALE_4_G = 0x01,
		SCALE_8_G = 0x02,
		SCALE_16_G = 0x03
	};

	enum AccelFChoiceB {
		ACCEL_USE_LPF = 0x00,					//Route through the LPF. FS will be choose according the SMPLR_RATE_DIVIDER
		ACCEL_BYPASS_LPF = 0x01				//Bypass the LPF and set the Bandwidth to 1130 KHz and FS 4 Khz
	};

	enum AccelLPF {
		ACCEL_LPF_460_HZ = 0x00,
		ACCEL_LPF_184_HZ = 0x01,
		ACCEL_LPF_92_HZ = 0x02,
		ACCEL_LPF_41_HZ = 0x03,
		ACCEL_LPF_20_HZ = 0x04,
		ACCEL_LPF_10_HZ = 0x05,
		ACCEL_LPF_5_HZ = 0x06,
		ACCEL_LPF_460_HZ_2 = 0x07
	};

	enum GyroRange {
		SCALE_250_DPS = 0x00,
		SCALE_500_DPS = 0x01,
		SCALE_1000_DPS = 0x02,
		SCALE_2000_DPS = 0x03
	};

	enum GyroFChoiceB {
		GYRO_USE_LPF = 0x00,					//Route through the LPF. The Bandwidth and FS will be choose according the LPF Setup
		GYRO_BYPASS_LPF_BW_8800_HZ = 0x01,	//Bypass the LPF and set the Bandwidth to 8800 Hz and FS 32 KHz
		GYRO_BYPASS_LPF_BW_3600_HZ = 0x02,	//Bypass the LPF and set the Bandwidth to 3600 Hz and FS 32 KHz
	};

	//These setup and FS Will depend upon the GyroFChoiceB
	enum GyroLPF {
		GYRO_LPF_250_HZ = 	0x00,	//FS 8 KHz, Delay 0.97 ms, SMPLRT_DIV not used
		GYRO_LPF_184_HZ = 	0x01,	//FS 1 KHz, Delay 2.90 ms, if SMPLRT_DIV = 0
		GYRO_LPF_92_HZ = 	0x02,	//FS 1 KHz, Delay 3.90 ms, if SMPLRT_DIV = 0
		GYRO_LPF_41_HZ = 	0x03,	//FS 1 KHz, Delay 5.90 ms, if SMPLRT_DIV = 0
		GYRO_LPF_20_HZ = 	0x04,	//FS 1 KHz, Delay 9.90 ms, if SMPLRT_DIV = 0
		GYRO_LPF_10_HZ = 	0x05,	//FS 1 KHz, Delay 17.85 ms, if SMPLRT_DIV = 0
		GYRO_LPF_5_HZ = 		0x06,	//FS 1 KHz, Delay 33.48 ms, if SMPLRT_DIV = 0
		GYRO_LPF_3600_HZ = 	0x07	//FS 8 KHz, Delay 0.17 ms, SMPLRT_DIV not used
	};

	MPU6500(  SPI_HandleTypeDef &spi, GPIO_TypeDef * csPort, uint16_t csPin );
	virtual ~MPU6500();

	friend void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

	void setGyroRateAndLPF( GyroFChoiceB path, GyroLPF lpfConfig, uint8_t sampleRateDivider = 0x00  );
	void setAccelRateAndLPF( AccelFChoiceB path, AccelLPF lpfConfig );

	void startReading();

	void intCallback();

	inline const ThreeAxisReadingsStamped< int16_t > & getAccelRaw() const {
		return m_rawAccel;
	}

	inline const ThreeAxisReadingsStamped< int16_t > & getGyroRaw() const {
		return m_rawGyro;
	}

	inline const SingleValueReadingStamped< int16_t > & getTempRaw() const {
		return m_rawTemp;
	}

	inline const ThreeAxisReadingsStamped< float > & getAccel() const {
		return m_accel;
	}

	inline const ThreeAxisReadingsStamped< float > & getGyro() const {
		return m_gyro;
	}

	inline const SingleValueReadingStamped< float > & getTemp() const {
		return m_temp;
	}


	// CONFIG register
	uint8_t getExternalFrameSync();
	void setExternalFrameSync(uint8_t sync);
	uint8_t getDLPFMode();
	void setDLPFMode(uint8_t bandwidth);

	// GYRO_CONFIG register
	uint8_t getFullScaleGyroRange();
	void setFullScaleGyroRange(GyroRange range);

	// SELF_TEST registers
	uint8_t getAccelXSelfTestFactoryTrim();
	uint8_t getAccelYSelfTestFactoryTrim();
	uint8_t getAccelZSelfTestFactoryTrim();

	uint8_t getGyroXSelfTestFactoryTrim();
	uint8_t getGyroYSelfTestFactoryTrim();
	uint8_t getGyroZSelfTestFactoryTrim();

	// ACCEL_CONFIG register
	bool getAccelXSelfTest();
	void setAccelXSelfTest(bool enabled);
	bool getAccelYSelfTest();
	void setAccelYSelfTest(bool enabled);
	bool getAccelZSelfTest();
	void setAccelZSelfTest(bool enabled);
	uint8_t getFullScaleAccelRange();
	void setFullScaleAccelRange(AccelRange range);
	uint8_t getDHPFMode();
	void setDHPFMode(uint8_t mode);

	// FF_THR register
	uint8_t getFreefallDetectionThreshold();
	void setFreefallDetectionThreshold(uint8_t threshold);

	// FF_DUR register
	uint8_t getFreefallDetectionDuration();
	void setFreefallDetectionDuration(uint8_t duration);

	// MOT_THR register
	uint8_t getMotionDetectionThreshold();
	void setMotionDetectionThreshold(uint8_t threshold);

	// MOT_DUR register
	uint8_t getMotionDetectionDuration();
	void setMotionDetectionDuration(uint8_t duration);

	// ZRMOT_THR register
	uint8_t getZeroMotionDetectionThreshold();
	void setZeroMotionDetectionThreshold(uint8_t threshold);

	// ZRMOT_DUR register
	uint8_t getZeroMotionDetectionDuration();
	void setZeroMotionDetectionDuration(uint8_t duration);

	// FIFO_EN register
	bool getTempFIFOEnabled();
	void setTempFIFOEnabled(bool enabled);
	bool getXGyroFIFOEnabled();
	void setXGyroFIFOEnabled(bool enabled);
	bool getYGyroFIFOEnabled();
	void setYGyroFIFOEnabled(bool enabled);
	bool getZGyroFIFOEnabled();
	void setZGyroFIFOEnabled(bool enabled);
	bool getAccelFIFOEnabled();
	void setAccelFIFOEnabled(bool enabled);
	bool getSlave2FIFOEnabled();
	void setSlave2FIFOEnabled(bool enabled);
	bool getSlave1FIFOEnabled();
	void setSlave1FIFOEnabled(bool enabled);
	bool getSlave0FIFOEnabled();
	void setSlave0FIFOEnabled(bool enabled);

	// I2C_MST_CTRL register
	bool getMultiMasterEnabled();
	void setMultiMasterEnabled(bool enabled);
	bool getWaitForExternalSensorEnabled();
	void setWaitForExternalSensorEnabled(bool enabled);
	bool getSlave3FIFOEnabled();
	void setSlave3FIFOEnabled(bool enabled);
	bool getSlaveReadWriteTransitionEnabled();
	void setSlaveReadWriteTransitionEnabled(bool enabled);
	uint8_t getMasterClockSpeed();
	void setMasterClockSpeed(uint8_t speed);

	// I2C_SLV* registers (Slave 0-3)
	uint8_t getSlaveAddress(uint8_t num);
	void setSlaveAddress(uint8_t num, uint8_t address);
	uint8_t getSlaveRegister(uint8_t num);
	void setSlaveRegister(uint8_t num, uint8_t reg);
	bool getSlaveEnabled(uint8_t num);
	void setSlaveEnabled(uint8_t num, bool enabled);
	bool getSlaveWordByteSwap(uint8_t num);
	void setSlaveWordByteSwap(uint8_t num, bool enabled);
	bool getSlaveWriteMode(uint8_t num);
	void setSlaveWriteMode(uint8_t num, bool mode);
	bool getSlaveWordGroupOffset(uint8_t num);
	void setSlaveWordGroupOffset(uint8_t num, bool enabled);
	uint8_t getSlaveDataLength(uint8_t num);
	void setSlaveDataLength(uint8_t num, uint8_t length);

	// I2C_SLV* registers (Slave 4)
	uint8_t getSlave4Address();
	void setSlave4Address(uint8_t address);
	uint8_t getSlave4Register();
	void setSlave4Register(uint8_t reg);
	void setSlave4OutputByte(uint8_t data);
	bool getSlave4Enabled();
	void setSlave4Enabled(bool enabled);
	bool getSlave4InterruptEnabled();
	void setSlave4InterruptEnabled(bool enabled);
	bool getSlave4WriteMode();
	void setSlave4WriteMode(bool mode);
	uint8_t getSlave4MasterDelay();
	void setSlave4MasterDelay(uint8_t delay);
	uint8_t getSlate4InputByte();

	// I2C_MST_STATUS register
	bool getPassthroughStatus();
	bool getSlave4IsDone();
	bool getLostArbitration();
	bool getSlave4Nack();
	bool getSlave3Nack();
	bool getSlave2Nack();
	bool getSlave1Nack();
	bool getSlave0Nack();

	// INT_PIN_CFG register
	bool getInterruptMode();
	void setInterruptMode(bool mode);
	bool getInterruptDrive();
	void setInterruptDrive(bool drive);
	bool getInterruptLatch();
	void setInterruptLatch(bool latch);
	bool getInterruptLatchClear();
	void setInterruptLatchClear(bool clear);
	bool getFSyncInterruptLevel();
	void setFSyncInterruptLevel(bool level);
	bool getFSyncInterruptEnabled();
	void setFSyncInterruptEnabled(bool enabled);
	bool getI2CBypassEnabled();
	void setI2CBypassEnabled(bool enabled);
	bool getClockOutputEnabled();
	void setClockOutputEnabled(bool enabled);

	// INT_ENABLE register
	uint8_t getIntEnabled();
	void setIntEnabled(uint8_t enabled);
	bool getIntFreefallEnabled();
	void setIntFreefallEnabled(bool enabled);
	bool getIntMotionEnabled();
	void setIntMotionEnabled(bool enabled);
	bool getIntZeroMotionEnabled();
	void setIntZeroMotionEnabled(bool enabled);
	bool getIntFIFOBufferOverflowEnabled();
	void setIntFIFOBufferOverflowEnabled(bool enabled);
	bool getIntI2CMasterEnabled();
	void setIntI2CMasterEnabled(bool enabled);
	bool getIntDataReadyEnabled();
	void setIntDataReadyEnabled(bool enabled);

	// INT_STATUS register
	uint8_t getIntStatus();
	bool getIntFreefallStatus();
	bool getIntMotionStatus();
	bool getIntZeroMotionStatus();
	bool getIntFIFOBufferOverflowStatus();
	bool getIntI2CMasterStatus();
	bool getIntDataReadyStatus();

	void updateAll();

	//Update sensors readings
	void updateAcceleration();
	void updateRotation();
	void updateTemperature();

	// EXT_SENS_DATA_* registers
	uint8_t getExternalSensorByte(int position);
	uint16_t getExternalSensorWord(int position);
	uint32_t getExternalSensorDWord(int position);

	// MOT_DETECT_STATUS register
	uint8_t getMotionStatus();
	bool getXNegMotionDetected();
	bool getXPosMotionDetected();
	bool getYNegMotionDetected();
	bool getYPosMotionDetected();
	bool getZNegMotionDetected();
	bool getZPosMotionDetected();
	bool getZeroMotionDetected();

	// I2C_SLV*_DO register
	void setSlaveOutputByte(uint8_t num, uint8_t data);

	// I2C_MST_DELAY_CTRL register
	bool getExternalShadowDelayEnabled();
	void setExternalShadowDelayEnabled(bool enabled);
	bool getSlaveDelayEnabled(uint8_t num);
	void setSlaveDelayEnabled(uint8_t num, bool enabled);

	// SIGNAL_PATH_RESET register
	void resetGyroscopePath();
	void resetAccelerometerPath();
	void resetTemperaturePath();

	// MOT_DETECT_CTRL register
	uint8_t getAccelerometerPowerOnDelay();
	void setAccelerometerPowerOnDelay(uint8_t delay);
	uint8_t getFreefallDetectionCounterDecrement();
	void setFreefallDetectionCounterDecrement(uint8_t decrement);
	uint8_t getMotionDetectionCounterDecrement();
	void setMotionDetectionCounterDecrement(uint8_t decrement);

	// USER_CTRL register
	bool getFIFOEnabled();
	void setFIFOEnabled(bool enabled);
	bool getI2CMasterModeEnabled();
	void setI2CMasterModeEnabled(bool enabled);
	void switchSPIEnabled(bool enabled);
	void resetFIFO();
	void resetI2CMaster();
	void resetSensors();

	// PWR_MGMT_1 register
	void reset();
	bool getSleepEnabled();
	void setSleepEnabled(bool enabled);
	bool getWakeCycleEnabled();
	void setWakeCycleEnabled(bool enabled);
	bool getTempSensorEnabled();
	void setTempSensorEnabled(bool enabled);
	uint8_t getClockSource();
	void setClockSource(uint8_t source);

	// PWR_MGMT_2 register
	uint8_t getWakeFrequency();
	void setWakeFrequency(uint8_t frequency);
	bool getStandbyXAccelEnabled();
	void setStandbyXAccelEnabled(bool enabled);
	bool getStandbyYAccelEnabled();
	void setStandbyYAccelEnabled(bool enabled);
	bool getStandbyZAccelEnabled();
	void setStandbyZAccelEnabled(bool enabled);
	bool getStandbyXGyroEnabled();
	void setStandbyXGyroEnabled(bool enabled);
	bool getStandbyYGyroEnabled();
	void setStandbyYGyroEnabled(bool enabled);
	bool getStandbyZGyroEnabled();
	void setStandbyZGyroEnabled(bool enabled);

	// FIFO_COUNT_* registers
	uint16_t getFIFOCount();

	// FIFO_R_W register
	uint8_t getFIFOByte();
	void setFIFOByte(uint8_t data);
	void getFIFOBytes(uint8_t *data, uint8_t length);

	// WHO_AM_I register
	uint8_t getDeviceID();
	void setDeviceID(uint8_t id);

	// ======== UNDOCUMENTED/DMP REGISTERS/METHODS ========

	// XG_OFFS_TC register
	uint8_t getOTPBankValid();
	void setOTPBankValid(bool enabled);
	int8_t getXGyroOffsetTC();
	void setXGyroOffsetTC(int8_t offset);

	// YG_OFFS_TC register
	int8_t getYGyroOffsetTC();
	void setYGyroOffsetTC(int8_t offset);

	// ZG_OFFS_TC register
	int8_t getZGyroOffsetTC();
	void setZGyroOffsetTC(int8_t offset);

	// X_FINE_GAIN register
	int8_t getXFineGain();
	void setXFineGain(int8_t gain);

	// Y_FINE_GAIN register
	int8_t getYFineGain();
	void setYFineGain(int8_t gain);

	// Z_FINE_GAIN register
	int8_t getZFineGain();
	void setZFineGain(int8_t gain);

	// XA_OFFS_* registers
	int16_t getXAccelOffset();
	void setXAccelOffset(int16_t offset);

	// YA_OFFS_* register
	int16_t getYAccelOffset();
	void setYAccelOffset(int16_t offset);

	// ZA_OFFS_* register
	int16_t getZAccelOffset();
	void setZAccelOffset(int16_t offset);

	// XG_OFFS_USR* registers
	int16_t getXGyroOffset();
	void setXGyroOffset(int16_t offset);

	// YG_OFFS_USR* register
	int16_t getYGyroOffset();
	void setYGyroOffset(int16_t offset);

	// ZG_OFFS_USR* register
	int16_t getZGyroOffset();
	void setZGyroOffset(int16_t offset);

	// INT_ENABLE register (DMP functions)
	bool getIntPLLReadyEnabled();
	void setIntPLLReadyEnabled(bool enabled);
	bool getIntDMPEnabled();
	void setIntDMPEnabled(bool enabled);

	// DMP_INT_STATUS
	bool getDMPInt5Status();
	bool getDMPInt4Status();
	bool getDMPInt3Status();
	bool getDMPInt2Status();
	bool getDMPInt1Status();
	bool getDMPInt0Status();

	// INT_STATUS register (DMP functions)
	bool getIntPLLReadyStatus();
	bool getIntDMPStatus();

	// USER_CTRL register (DMP functions)
	bool getDMPEnabled();
	void setDMPEnabled(bool enabled);
	void resetDMP();

	// BANK_SEL register
	void setMemoryBank(uint8_t bank, bool prefetchEnabled=false, bool userBank=false);

	// MEM_START_ADDR register
	void setMemoryStartAddress(uint8_t address);

	// MEM_R_W register
	uint8_t readMemoryByte();
	void writeMemoryByte(uint8_t data);
	void readMemoryBlock(uint8_t *data, uint16_t dataSize, uint8_t bank=0, uint8_t address=0);
	bool writeMemoryBlock(const uint8_t *data, uint16_t dataSize, uint8_t bank=0, uint8_t address=0, bool verify=true, bool useProgMem=false);
	bool writeProgMemoryBlock(const uint8_t *data, uint16_t dataSize, uint8_t bank=0, uint8_t address=0, bool verify=true);

	bool writeDMPConfigurationSet(const uint8_t *data, uint16_t dataSize, bool useProgMem=false);
	bool writeProgDMPConfigurationSet(const uint8_t *data, uint16_t dataSize);

	// DMP_CFG_1 register
	uint8_t getDMPConfig1();
	void setDMPConfig1(uint8_t config);

	// DMP_CFG_2 register
	uint8_t getDMPConfig2();
	void setDMPConfig2(uint8_t config);

private:
	/*--- Methods based on
	 * https://github.com/jrowberg/i2cdevlib/blob/master/Arduino/MPU6050/MPU6050.h
	 * Those methods were based on the above github but modified in order for allowing use of the STM32 SPI Library
	 * for interface
	 */

	void readRegisters( uint8_t reg, size_t size, uint8_t data[] );
	void writeRegisters( uint8_t reg,  uint8_t data[], size_t size );

	void readRegBit( uint8_t reg, uint8_t bitPos, uint8_t *data );
	void writeRegBit( uint8_t reg, uint8_t bitPos, uint8_t data );

	void readRegBits( uint8_t reg, uint8_t bitPos, uint8_t bitsLenght, uint8_t *data );
	void writeRegBits( uint8_t reg, uint8_t bitPos, uint8_t bitsLenght, uint8_t data );

	void readRegByte( uint8_t reg, uint8_t *data );
	void writeRegByte( uint8_t reg, uint8_t data );

	bool testConnection();

	// AUX_VDDIO register
	uint8_t getAuxVDDIOLevel();
	void setAuxVDDIOLevel(uint8_t level);

	/*-- Methods. --*/
	inline void activateDevice() const {
		HAL_GPIO_WritePin( m_csPort, m_csPin, GPIO_PIN_RESET );
	}

	inline void deactivateDevice() const {
		HAL_GPIO_WritePin( m_csPort, m_csPin, GPIO_PIN_SET );
	}

	inline void clearRxBuffer() {
		for( size_t i = 0; i < BUFFER_SIZE; ++i ) {
			m_rxBuffer[i] = 0;
		}
	}

	inline void clearTxBuffer() {
		for( size_t i = 0; i < BUFFER_SIZE; ++i ) {
			m_txBuffer[i] = 0;
		}
	}

	inline void clearBuffers() {
		for( size_t i = 0; i < BUFFER_SIZE; ++i ) {
			m_txBuffer[i] = 0;
			m_rxBuffer[i] = 0;
			buffer[i] = 0;
		}
	}

	/*-- Variables. --*/
	SPI_HandleTypeDef & m_spi;

	GPIO_TypeDef * m_csPort;
	const uint16_t m_csPin;

	State m_state;

	/*--- Variáveis de configuração do sensor. ---*/
	float m_accelRate;
	float m_gyroRate;
	float m_tempRate;

	/*-- Largura de banda dos sensores, confiugrada nos sensores. ---*/
	float m_accelLPFBandwidth;
	float m_gyroLPFBandwidth;

	uint8_t m_sampleRateDivider;

	/*-- Fundo de escala configurado dos sensores. ---*/
	float m_accelScale;
	float m_gyroScale;
	float m_tempScale;

	/*--- Variáveis que armazenam configurações e leituras. ---*/
	ThreeAxisReadingsStamped< int16_t > m_rawAccel;
	ThreeAxisReadingsStamped< int16_t > m_rawGyro;
	SingleValueReadingStamped< int16_t > m_rawTemp;

	ThreeAxisReadingsStamped< float > m_accel;
	ThreeAxisReadingsStamped< float > m_gyro;
	SingleValueReadingStamped< float > m_temp;

	uint8_t m_txBuffer[BUFFER_SIZE];
	uint8_t m_rxBuffer[BUFFER_SIZE];
	uint8_t buffer[BUFFER_SIZE];

};

}  // namespace Sensors

#endif /* _MPU6500_H_ */
