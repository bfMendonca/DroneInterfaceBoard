/*
 * SensorTypes.h
 *
 *  Created on: 6 de jan de 2019
 *      Author: bmendonca
 */

#ifndef DATATYPES_SENSORTYPES_H_
#define DATATYPES_SENSORTYPES_H_

/*--- Includes. ---*/
#include "stm32f0xx_hal.h"

#include <string.h>
#include <math.h>

namespace Sensors {

template< typename Targ >
class ThreeAxisReading {
public:
	ThreeAxisReading() :
		m_x( Targ() ),
		m_y( Targ() ),
		m_z( Targ() ) {	}

	ThreeAxisReading( const Targ &x, const Targ &y, const Targ &z ) :
		m_x( x ),
		m_y( y ),
		m_z( z ) { }

	inline const Targ & x() const { return m_x; };
	inline const Targ & y() const { return m_y; };
	inline const Targ & z() const { return m_z; };

	inline void setX( const Targ & x ) { m_x = x; };
	inline void setY( const Targ & y ) { m_y = y; };
	inline void setZ( const Targ & z ) { m_z = z; };

	inline Targ & rx() { return m_x; };
	inline Targ & ry() { return m_y; };
	inline Targ & rz() { return m_z; };


private:
	Targ m_x, m_y, m_z;
};

template< typename Targ >
class ThreeAxisReadingsStamped : public ThreeAxisReading< Targ > {
public:
	ThreeAxisReadingsStamped() : ThreeAxisReading< Targ >(),
			m_timeStamp() { }

	ThreeAxisReadingsStamped( uint64_t initialTs ) : ThreeAxisReading< Targ >(),
			m_timeStamp( initialTs ) { }

	ThreeAxisReadingsStamped( const Targ &x, const Targ &y, const Targ &z, uint64_t initialTs = 0) : ThreeAxisReading< Targ >( x, y, z),
			m_timeStamp( initialTs ) { }

	inline void setTimeStamp( uint64_t timeStamp ) {
		m_timeStamp = timeStamp;
	}

	inline const uint64_t & timeStamp() const { return m_timeStamp; };

private:
	uint64_t m_timeStamp;
};

template< typename Targ >
class SingleValueReadingStamped {
public:
	SingleValueReadingStamped( ) :
		m_value( Targ() ),
		m_timeStamp( 0 ) { }

	SingleValueReadingStamped( const Targ & value, uint64_t timeStamp  = 0 ) :
		m_value( value ),
		m_timeStamp( timeStamp ) { }

	inline void setTimeStamp( uint64_t timeStamp ) {
		m_timeStamp = timeStamp;
	}

	inline const uint64_t & timeStamp() const { return m_timeStamp; };

	inline Targ & rValue() { return m_value; };

	inline void setValue( const Targ & value ) {
		m_value = value;
	}

	inline const Targ & value() const { return m_value; };

private:
	Targ m_value;
	uint64_t m_timeStamp;
};

} /* namespace Sensors */

#endif /* DATATYPES_SENSORTYPES_H_ */
