/*
 * CircularQueue.h
 *
 *  Created on: 27 de jan de 2019
 *      Author: Bruno Mendonca
 */

#ifndef CIRCULARQUEUE_H_
#define CIRCULARQUEUE_H_

#include <ctype.h>
#include <stddef.h>


namespace Containers {

template< class Type, size_t Size >
class CircularQueue {
public:
    CircularQueue();
    virtual ~CircularQueue();

    bool put( const Type & value );

    bool get( Type & value);
    bool peek( Type & value);

    size_t size() {
    	return m_size;
    }

    size_t availableSize() {
    	return (Size - m_size);
    }

private:
    struct {
        Type item;
        bool status;
    } m_vec[ Size ];

    size_t m_writeIndex;
    size_t m_readIndex;

    size_t m_size;

};

} /* namespace Containers */

#include "CircularQueue.tpp"

#endif /* CIRCULARQUEUE_H_ */
