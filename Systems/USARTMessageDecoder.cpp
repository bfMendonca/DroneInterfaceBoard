/*
 * USARTMessageDecoder.cpp
 *
 *  Created on: 27 de jan de 2019
 *      Author: bmendonca
 */

#include <USARTMessageDecoder.h>

namespace Comm {

USARTMessageDecoder::USARTMessageDecoder() :
		m_writeIndex( 0 ),
		m_messageStarted( false ) {
	// TODO Auto-generated constructor stub

}

USARTMessageDecoder::~USARTMessageDecoder() {
	// TODO Auto-generated destructor stub
}

bool USARTMessageDecoder::appendDataToDecode( uint8_t value[], size_t size ) {

	for( size_t i = 0; i < size; ++i ) {

		if( ( m_writeIndex == 0 ) && ( value[i] == 0xAA ) ) {
			m_decodeBuffer[m_writeIndex] = value[i];
			m_writeIndex++;
		}else if( ( m_writeIndex == 1 ) && ( value[i] == 0xBB ) ) {
			m_decodeBuffer[m_writeIndex] = value[i];
			m_writeIndex++;
		}else if( ( m_writeIndex == 2 ) && ( value[i] == 0xCC ) ) {
			m_decodeBuffer[m_writeIndex] = value[i];
			m_writeIndex++;

			//If we are here, so we received a valid sequence
			m_messageStarted = true;
		}else if( m_messageStarted ) {
			if( m_writeIndex == 3 ) {
				m_currentMessage.type = value[i];
				m_currentMessage.startIndex = m_writeIndex+1;

				m_decodeBuffer[m_writeIndex] = value[i];
				m_writeIndex++;
		}else if( m_writeIndex > 3 ) {
				m_decodeBuffer[m_writeIndex] = value[i];
				m_writeIndex++;

				if( ( value[i] == 'M') && (value[i-1] == 'I') && (value[i-2] == 'F') && (value[i-3] == '*') ) {
					m_currentMessage.endIndex = m_writeIndex;
					m_currentMessage.length = m_currentMessage.endIndex - m_currentMessage.startIndex - 4;

					switch( m_currentMessage.type ) {
					case 0x99: {
						MotorsInterfaceMessage motorsInterface = MotorsInterfaceMessage_init_zero;
						pb_istream_t motorsStream = pb_istream_from_buffer(&m_decodeBuffer[m_currentMessage.startIndex], m_currentMessage.length);

						bool status = pb_decode( &motorsStream, MotorsInterfaceMessage_fields, &motorsInterface );

						if( status ) {
							//Success, need to warn the callback
						}
					}
						break;
					}


					m_writeIndex = 0;
				}
			}
		}

	}
}

} /* namespace Comm */
