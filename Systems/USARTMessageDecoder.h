/*
 * USARTMessageDecoder.h
 *
 *  Created on: 27 de jan de 2019
 *      Author: bmendonca
 */

#ifndef USARTMESSAGEDECODER_H_
#define USARTMESSAGEDECODER_H_

#include "stm32f0xx_hal.h"

#include "CircularQueue.h"

#include "pb_common.h"
#include "pb_encode.h"
#include "pb_decode.h"

#include "motorsinterface.pb.h"

namespace Comm {

class USARTMessageDecoder {
public:
	USARTMessageDecoder();
	virtual ~USARTMessageDecoder();

	bool appendDataToDecode( uint8_t value[], size_t size );

private:
	uint8_t m_decodeBuffer[100];
	size_t m_writeIndex;

	/*--- Decoder state. ---*/
	bool m_messageStarted;

	struct {
		size_t startIndex;
		size_t endIndex;
		size_t length;
		uint8_t type;
		uint8_t data[100];
	} m_currentMessage;

};

} /* namespace Comm */

#endif /* USARTMESSAGEDECODER_H_ */
