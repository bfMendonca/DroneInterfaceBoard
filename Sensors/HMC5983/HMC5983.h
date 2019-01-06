/*
 * HMC5983.h
 *
 *  Created on: 5 de jan de 2019
 *      Author: bmendonca
 */

#ifndef HMC5983_HMC5983_H_
#define HMC5983_HMC5983_H_

/*--- Includes. ---*/
#include "stm32f0xx_hal.h"

#include <string.h>
#include <math.h>

#include <DataTypes/SensorTypes.h>

//Registers
#define HMC5983_CONF_A 0x00
#define HMC5983_CONF_B 0x01
#define HMC5983_MODE_C 0x02

#define HMC5983_XOUT_H 0x03
#define HMC5983_XOUT_L 0x04
#define HMC5983_ZOUT_H 0x05
#define HMC5983_ZOUT_L 0x06
#define HMC5983_YOUT_H 0x07
#define HMC5983_YOUT_L 0x08

#define HMC5983_STATUS    0x09

#define HMC5983_ID_A 0x0A
#define HMC5983_ID_B 0x0B
#define HMC5983_ID_C 0x0C

#define HMC5983_TEMP_H 0x31
#define HMC5983_TEMP_L 0x32

//Regiters positions and bits

//CONFIGURATION_A
#define HMC5983_CONF_A_TS_BIT 			7

#define HMC5983_CONF_A_AVRG_BIT			6
#define HMC5983_CONF_A_AVRG_LENGTH		2

#define HMC5983_CONF_A_RATE_BIT 		4
#define HMC5983_CONF_A_RATE_LENGTH  	3

#define HMC5983_CONF_A_MEAS_MODE_BIT	1
#define HMC5983_CONF_A_MEAS_MODE_LENGTH	2

//CONFIGURATION_B
#define HMC5983_CONF_B_GAIN_BIT		7
#define HMC5983_CONF_B_GAIN_LENGTH	3

//Mode Register
#define HMC5983_MODE_I2C_HS_BIT			7
#define HMC5983_MODE_LOW_POWER			5
#define HMC5983_SPI_MODE				2
#define HMC5983_OPERATION_MODE_BIT		1
#define HMC5983_OPERATION_MODE_LENGTH	2

//Status Register
#define HMC5983_STATUS_DOW_BIT	4
#define HMC5983_STATUS_LOCK_BIT	1
#define HMC5983_STATUS_RDY_BIT	0

/*--- Setups. ---*/
#define BUFFER_SIZE 30

namespace Sensors {

class HMC5983 {
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

	enum Samples {
		SMPLS_1 = 0x00,
		SMPLS_2 = 0x01,
		SMPLS_4 = 0x02,
		SMPLS_8 = 0x03
	};

	enum Rate {
		DR_000_75_HZ = 0x00,	//Output Data rate of 0.75 Hz
		DR_001_50_HZ = 0x01,	//Output Data rate of 1.50 Hz
		DR_003_00_HZ = 0x02,	//Output Data rate of 3.00 Hz
		DR_007_50_HZ = 0x03,	//Output Data rate of 7.50 Hz
		DR_015_00_HZ = 0x04,	//Output Data rate of 15.00 Hz
		DR_030_00_HZ = 0x05,	//Output Data rate of 30.00 Hz
		DR_075_00_HZ = 0x06,	//Output Data rate of 75.00 Hz
		DR_220_00_HZ = 0x07		//Output Data rate of 220.00 Hz
	};

	enum MeasurementMode {
		NORMAL_MODE = 0x00,
		POSITIVE_BIAS = 0x01,
		NEGATIVE_BIAS = 0x02,
		TEMPERATURE_ONLY = 0x03
	};

	enum OperatingMode {
		CONTINOUS = 0x00,
		SINGLE_MEASUREMENT = 0x01,
		IDLE = 0x02
	};

	enum Range {
		SCALE_00_88_GA = 0x00,	//0.88 Gauss
		SCALE_01_30_GA = 0x01,  //1.30 Gauss
		SCALE_01_90_GA = 0x02,	//1.90 Gauss
		SCALE_02_50_GA = 0x03,	//2.50 Gauss
		SCALE_04_00_GA = 0x04,	//4.00 Gauss
		SCALE_04_70_GA = 0x05,	//4.70 Gauss
		SCALE_05_60_GA = 0x06,	//5.60 Gauss
		SCALE_08_10_GA = 0x07,	//8.10 Gauss
	};

	HMC5983( SPI_HandleTypeDef &spi, GPIO_TypeDef *csPort, uint16_t csPin );

	virtual ~HMC5983();


	void setMeasurementConfig( bool tempCompensated, Samples samples, Rate rate, MeasurementMode mode );

	void setFullScaleRange( Range range );

	void setOperationState( OperatingMode mode );

	void updateAll();

	void intCallback();

private:

	void readRegisters( uint8_t reg, size_t size, uint8_t data[], bool autoIncAddress = true );
	void writeRegisters( uint8_t reg,  uint8_t data[], size_t size, bool autoIncAddress = true );

	void readRegBit( uint8_t reg, uint8_t bitPos, bool &state);
	void writeRegBit( uint8_t reg, uint8_t bitPos, bool state );

	void readRegBits( uint8_t reg, uint8_t bitPos, uint8_t bitsLenght, uint8_t &data );
	void writeRegBits( uint8_t reg, uint8_t bitPos, uint8_t bitsLenght, uint8_t data );

	void readRegByte( uint8_t reg, uint8_t *data );
	void writeRegByte( uint8_t reg, uint8_t data );

	inline const ThreeAxisReadingsStamped< int16_t > & getMagRaw() const {
		return m_rawMag;
	}

	inline const SingleValueReadingStamped< int16_t > & getTempRaw() const {
		return m_rawTemp;
	}

	inline const ThreeAxisReadingsStamped< float > & getMag() const {
		return m_mag;
	}

	inline const SingleValueReadingStamped< float > & getTemp() const {
		return m_temp;
	}


	bool testConnection();

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

	float m_rate;
	float m_scale;

	ThreeAxisReadingsStamped< int16_t > m_rawMag;
	ThreeAxisReadingsStamped< float > m_mag;

	SingleValueReadingStamped< int16_t > m_rawTemp;
	SingleValueReadingStamped< float > m_temp;

	uint8_t m_txBuffer[BUFFER_SIZE];
	uint8_t m_rxBuffer[BUFFER_SIZE];
	uint8_t buffer[BUFFER_SIZE];
};

} /* namespace Sensors */

#endif /* HMC5983_HMC5983_H_ */
