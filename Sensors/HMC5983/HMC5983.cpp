/*
 * HMC5983.cpp
 *
 *  Created on: 5 de jan de 2019
 *      Author: bmendonca
 */

#include <HMC5983/HMC5983.h>

using namespace Sensors;

HMC5983::HMC5983( SPI_HandleTypeDef &spi, GPIO_TypeDef *csPort, uint16_t csPin ) :
	m_spi( spi ),
	m_csPort( csPort ),
	m_csPin( csPin )
{
	deactivateDevice();
	//
	clearBuffers();

	if( testConnection()  ) {
		setMeasurementConfig( true, SMPLS_1, DR_220_00_HZ, NORMAL_MODE );
		setFullScaleRange( SCALE_01_90_GA );
		setOperationState( CONTINOUS );
	}else {

	}
}

void HMC5983::setMeasurementConfig( bool tempCompensated, Samples samples, Rate rate, MeasurementMode mode ) {

	uint8_t data = 0x00;

	//First enabling the temperature compensation
	if( tempCompensated ) {
		data |= 0x01 << ( HMC5983_CONF_A_TS_BIT );
	}

	//Setting samples
	uint8_t rawSamples = static_cast< uint8_t >( samples );
	data |= ( rawSamples << ( HMC5983_CONF_A_AVRG_BIT - ( HMC5983_CONF_A_AVRG_LENGTH - 1 ) ) );

	//Setting rate
	uint8_t rawRate = static_cast< uint8_t >( rate );
	data |= ( rawRate << ( HMC5983_CONF_A_RATE_BIT - ( HMC5983_CONF_A_RATE_LENGTH -1 ) ) );

	//Setting mode
	uint8_t rawMode = static_cast< uint8_t >( mode );
	data |= ( rawMode << ( HMC5983_CONF_A_MEAS_MODE_BIT - ( HMC5983_CONF_A_MEAS_MODE_LENGTH -1 ) ) );

	writeRegByte( HMC5983_CONF_A, data );
}

void HMC5983::setFullScaleRange( Range range ) {

	switch( range ) {
	case SCALE_00_88_GA:
		m_scale = 1.49431; //
		break;

	case SCALE_01_30_GA:
		m_scale = 1.88324;
		break;

	case SCALE_01_90_GA:
		m_scale = 2.4973;
		break;

	case SCALE_02_50_GA:
		m_scale = 3.11144;
		break;

	case SCALE_04_00_GA:
		m_scale = 4.64669;
		break;

	case SCALE_04_70_GA:
		m_scale = 5.26079;
		break;

	case SCALE_05_60_GA:
		m_scale = 6.20241;
		break;

	case SCALE_08_10_GA:
		m_scale = 8.90445;
		break;

	default:
		range = SCALE_01_30_GA;
		m_scale = 1.88324;
	}

	uint8_t rawScale = static_cast< uint8_t >( range );

	uint8_t data = rawScale << ( HMC5983_CONF_B_GAIN_BIT - ( HMC5983_CONF_B_GAIN_LENGTH -1 ) );

	//Make sure that the least 5 bits of the register are not written
	data &= 0xE0;

	writeRegByte( HMC5983_CONF_B, data );

	//Saving the rate in order to convert the data

}

void HMC5983::setOperationState( OperatingMode mode ) {
	uint8_t data = 0x00;
	uint8_t rawMode = static_cast< uint8_t >( mode );
	data |= rawMode;

	writeRegByte( HMC5983_MODE_C, data );
}

void HMC5983::updateAll() {

}

HMC5983::~HMC5983() {
	// TODO Auto-generated destructor stub
}

void HMC5983::readRegisters( uint8_t reg, size_t size, uint8_t *data, bool autoIncAddress ) {

	m_txBuffer[0] = reg | 0x80;

	if( autoIncAddress ) {
		m_txBuffer[0] |= 0x40;
	}

	activateDevice();
	HAL_SPI_TransmitReceive( &m_spi, m_txBuffer, m_rxBuffer, (size+1), 1000 );
	deactivateDevice();

	for( size_t i = 0; i < size; ++i ) {
		data[i] = m_rxBuffer[i+1];
	}
}

void HMC5983::writeRegisters( uint8_t reg, uint8_t data[], size_t size, bool autoIncAddress ) {
	m_txBuffer[0] = reg;

	if( autoIncAddress ) {
		m_txBuffer[0] = reg | 0x40;
	}

	for( size_t i = 0; i < size; ++i) {
		m_txBuffer[1+i] = data[i];
	}

	activateDevice();
	HAL_SPI_TransmitReceive( &m_spi, m_txBuffer, m_rxBuffer, (size+1), 1000 );
	deactivateDevice();

	return;
}

void HMC5983::readRegBit( uint8_t reg, uint8_t bitPos, bool &state ) {
	uint8_t buffer = 0x00;
	readRegByte( reg, &buffer );

	if( ( buffer & ( 0x01 << bitPos ) ) != 0x00 ) {
		state = true;
	}else{
		state = false;
	}
}

void HMC5983::writeRegBit( uint8_t reg, uint8_t bitPos, bool state ) {
	//Primeiro, le o registrador
	uint8_t buffer = 0x00;
	readRegByte( reg, &buffer );

	uint8_t data;
	state ? data = 0x01 : data = 0x00;

	//Ajusta a máscara dos bit que será ajustado
	uint8_t mask = ( 0x01 << bitPos );
	data = data << bitPos;

	//Garante que somente um bit do argumento esteja setado
	data = data & mask;

	//Limpa o bit do registrador que iremos escrever
	buffer = buffer & ~mask;

	//Agora que o Registrado está limpo, escrevemos o dado.
	buffer = buffer | data;

	//Escrita no registrador
	writeRegByte( reg, buffer );
}

void HMC5983::readRegBits( uint8_t reg, uint8_t bitPos, uint8_t bitsLenght, uint8_t &data ) {
	uint8_t buffer = 0x00;
	readRegByte( reg, &buffer );

	uint8_t mask = 0x00;
	uint8_t temp = 0x01;
	for( size_t i = 0; i < bitsLenght; ++i ) {
		mask = mask | temp << i ;
	}

	mask = mask << ( bitPos - ( bitsLenght -1 ) );
	data = buffer & mask;
}

void HMC5983::writeRegBits( uint8_t reg, uint8_t bitPos, uint8_t bitsLenght, uint8_t data ) {
	uint8_t buffer = 0x00;
	readRegByte( reg, &buffer );

	uint8_t mask = 0x00;
	uint8_t temp = 0x01;
	for( size_t i = 0; i < bitsLenght; ++i ) {
		mask = mask | temp << i ;
	}


	mask = mask << ( bitPos - ( bitsLenght - 1 ) );
	data = data << ( bitPos - ( bitsLenght - 1 ) );

	//Primeiro, vamos limpar os bits em que iremos escrever
	buffer = buffer & ~mask;

	//Agora, vamos sobreescrever a seção dos bits - Data & Mask garante que somente a seção desejada será sobrescrita
	buffer = buffer | ( data & mask );

	writeRegByte( reg, buffer );
}

void HMC5983::readRegByte( uint8_t reg, uint8_t *data ) {
	readRegisters(reg, 1, data);
}

void HMC5983::writeRegByte( uint8_t reg, uint8_t data ) {
	writeRegisters(reg, &data, 1 );
}

void HMC5983::intCallback() {
	/*--- Se estamos aqui, então uma interrupção foi disparada, vamos recuerar o registrador de status. ---*/

	uint32_t ts = HAL_GetTick();
	readRegisters( HMC5983_XOUT_H, 6, buffer, true );

	m_rawMag.rx() =	(((int16_t)buffer[0]) << 8) | buffer[1];
	m_rawMag.rz() = (((int16_t)buffer[2]) << 8) | buffer[3];
	m_rawMag.ry() = (((int16_t)buffer[4]) << 8) | buffer[5];

	//Let's convert the values using the full-scale setup
	m_mag.setX( m_rawMag.x()*m_scale/2048.0 );
	m_mag.setY( m_rawMag.y()*m_scale/2048.0 );
	m_mag.setZ( m_rawMag.z()*m_scale/2048.0 );


	readRegisters( HMC5983_TEMP_H, 2, buffer, true );
	m_rawTemp.rValue() = 	(((int16_t)buffer[0]) << 8) | buffer[1];

	//Get the temperature
	m_temp = m_rawTemp.value()*100.0/32768.0 + 21.0;

	m_rawMag.setTimeStamp( ts );
	m_mag.setTimeStamp( ts );
}

/** Verify the SPI connection.
 * Make sure the device is connected and responds as expected.
 * @return True if connection is valid, false otherwise
 */
bool HMC5983::testConnection() {
	readRegisters( HMC5983_ID_A, 3, buffer, true );

	if( buffer[0] == 'H' && buffer[1] == '4' && buffer[2] == '3' ) {
		return true;
	}

	return false;
}
