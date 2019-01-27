/*
 * USARTInterface.h
 *
 *  Created on: 26 de jan de 2019
 *      Author: bmendonca
 */

#ifndef USARTINTERFACE_H_
#define USARTINTERFACE_H_

#include "stm32f0xx_hal.h"

#include "CircularQueue.h"

namespace Comm {

template< size_t Tx_Size, size_t Rx_Size >
class USARTInterface {
public:
	USARTInterface( UART_HandleTypeDef * huart );
	virtual ~USARTInterface();

	bool appendDataToSend( const uint8_t *data, size_t size );

	void txCallback();
	void rxCallbacl();

	void usartErrorCallback();

private:

	UART_HandleTypeDef *m_huart;

	bool m_txOngoing;

	Containers::CircularQueue<uint8_t, 150> m_inputBuffer;
	Containers::CircularQueue<uint8_t, 150> m_outputBuffer;

	//DMA Buffer. This is a mid layer dma buffers for handling data
	static const size_t TX_DMA_SIZE = 50;
	static const size_t RX_DMA_SIZE = 50;

	uint8_t m_dmaOutputBuffer[ TX_DMA_SIZE ];
	uint8_t m_dmaInputBuffer[ RX_DMA_SIZE ];
};

} /* namespace Comm */

#include "USARTInterface.tpp"

#endif /* USARTINTERFACE_H_ */
