/*
 * CircularQueue.cpp
 *
 *  Created on: 27 de jan de 2019
 *      Author: Bruno Mendonca
 */

namespace Containers {

template< class Type, size_t Size >
CircularQueue< Type, Size >::CircularQueue() :
m_writeIndex( 0 ),
m_readIndex( 0 ),
m_size( 0 )
{
	// TODO Auto-generated constructor stub

	for( size_t i = 0; i < Size; ++i ) {
		m_vec[i].item = Type();
		m_vec[i].status = false;
	}
}

template< class Type, size_t Size >
CircularQueue< Type, Size >::~CircularQueue() {
	// TODO Auto-generated destructor stub
}

template< class Type, size_t Size >
bool CircularQueue< Type, Size >::put( const Type & value ) {

	//Detect overwrite
	if( m_vec[ m_writeIndex ].status == true ) {
		return false;
	}

	m_vec[ m_writeIndex ].item = value;
	m_vec[ m_writeIndex ].status = true;

	m_writeIndex = (m_writeIndex+1)%Size;

	++m_size;

	return true;
}

template< class Type, size_t Size >
bool CircularQueue< Type, Size >::get( Type & value ) {

	if( m_vec[m_readIndex].status ) {
		value = m_vec[m_readIndex].item;
		m_vec[m_readIndex].status = false;

		m_readIndex = (m_readIndex+1)%Size;

		--m_size;

		return true;
	}

	return false;
}

template< class Type, size_t Size >
bool CircularQueue< Type, Size >::peek( Type & value ) {

	if( m_vec[m_readIndex].status ) {
		value = m_vec[m_readIndex].item;
		return true;
	}

	return false;
}

} /* namespace Containers */
