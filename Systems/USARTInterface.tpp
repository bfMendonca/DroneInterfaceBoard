/*
 * USARTInterface.cpp
 *
 *  Created on: 26 de jan de 2019
 *      Author: bmendonca
 */

#include "USARTInterface.h"

namespace Comm {


template< size_t Tx_Size, size_t Rx_Size >
USARTInterface< Tx_Size, Rx_Size >::USARTInterface( UART_HandleTypeDef * huart ) :
		m_huart( huart ),
		m_txOngoing( false ),
		m_inputBuffer(),
		m_outputBuffer() {

	HAL_UART_Receive_DMA( m_huart, m_dmaInputBuffer, Rx_Size );
}

template< size_t Tx_Size, size_t Rx_Size >
USARTInterface< Tx_Size, Rx_Size >::~USARTInterface() {
	// TODO Auto-generated destructor stub
}

template< size_t Tx_Size, size_t Rx_Size >
bool USARTInterface< Tx_Size, Rx_Size >::appendDataToSend( const uint8_t *data, size_t size ) {
	if( ( size >= Tx_Size ) || (size >=  m_outputBuffer.availableSize() ) ) {
		return false;
	}

	for( size_t  i = 0; i < size; ++i ) {
		if( !m_outputBuffer.put( data[i] ) ) {
			return false;
		}
	}

	//NOTE: This could be further improved, because we do not need to put in the output buffer if the dma is free

	//not sending, we can deliver data to the USART DMA
	if( !m_txOngoing ) {

		//Copy at most TX_DMA_SIZE bytes to the dma mid buffer
		uint8_t temp;
		for( size_t i = 0; ( ( i < TX_DMA_SIZE ) && ( i < size ) ) ; ++i ) {
			if( !m_outputBuffer.get( temp ) ) {
				return false;
			}
			m_dmaOutputBuffer[i] = temp;
		}

		size_t toSendDataSize = 0;

		size < TX_DMA_SIZE ? toSendDataSize = size : toSendDataSize = TX_DMA_SIZE;

		HAL_UART_Transmit_DMA( m_huart, m_dmaOutputBuffer, toSendDataSize );

		m_txOngoing = true;
	}

	return true;
}

template< size_t Tx_Size, size_t Rx_Size >
void USARTInterface< Tx_Size, Rx_Size >::txCallback() {
	//Let's check if we have more data to send

	size_t size = m_outputBuffer.size();
	if( size != 0 ) {
		//Copy at most TX_DMA_SIZE bytes to the dma mid buffer
		uint8_t temp;
		for( size_t i = 0; ( ( i < TX_DMA_SIZE ) && ( i < size ) ) ; ++i ) {
			if( !m_outputBuffer.get( temp ) ) {
				break;
			}
			m_dmaOutputBuffer[i] = temp;
		}

		size_t toSendDataSize = 0;

		size < TX_DMA_SIZE ? toSendDataSize = size : toSendDataSize = TX_DMA_SIZE;

		HAL_UART_Transmit_DMA( m_huart, m_dmaOutputBuffer, toSendDataSize );

		m_txOngoing = true;
	}else {
		m_txOngoing = false;	
	}
}

template< size_t Tx_Size, size_t Rx_Size >
void USARTInterface< Tx_Size, Rx_Size >::rxCallbacl() {

}

template< size_t Tx_Size, size_t Rx_Size >
void USARTInterface< Tx_Size, Rx_Size >::usartErrorCallback() {

}


} /* namespace Comm */
