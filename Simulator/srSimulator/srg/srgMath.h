/*******************************************************************************
[ SNU Robotics Graphic ]

author: JeongSeok Lee

Version information
v0.001 : 2008-11-12			Jeongseok Lee
*******************************************************************************/

#ifndef __SRG_MATH__
#define __SRG_MATH__

#define _USE_MATH_DEFINES
#include <math.h>


/*!
	Useful constants.
*/
#define SRG_ONETHIRD		(0.333333333333333333333)
#define SRG_ONESIXTH		(0.166666666666666666667)
#define SRG_HALF_PI			1.57079632679489661923f
#define SRG_PI				3.14159265358979323846f
#define SRG_TWO_PI			6.28318530717958647693f		// = 2 * pi
#define SRG_PI_DIV_180		0.017453292519943296f
#define SRG_INV_PI_DIV_180	57.2957795130823229f

/*!
	Radians and degrees.
*/
#define SRG_DEG2RAD(x)	(SRG_PI_DIV_180*(x))
#define SRG_RAD2DEG(x)	(SRG_INV_PI_DIV_180*(x))
inline double SRG_DEG2RADf(const float val)		{ return val*0.017453292519943296f;}
inline double SRG_RAD2DEGf(const float val)		{ return val*57.2957795130823229f;}
inline double SRG_DEG2RADd(const double val)	{ return val*0.0174532925199432957692369076848861;}
inline double SRG_RAD2DEGd(const double val)	{ return val*57.2957795130823208767981548141052;}

/*!
	Macros for big/little endian happiness.
*/
#define SRG_BYTE_SWAP(x)    x = ((x) >> 8) + ((x) << 8)
#define SRG_MAX(a, b)	((a) > (b) ? (a) : (b))
#define SRG_SQR(x) ( (x) * (x) )



#endif // __SRG_MATH__

