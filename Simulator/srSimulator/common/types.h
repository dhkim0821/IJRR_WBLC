#ifndef	SRLIB_TYPE
#define	SRLIB_TYPE

#define SRLIB_DOUBLE
#ifdef SRLIB_DOUBLE
	// 기본형이 real 일 때,
	typedef double			sr_real;
	#define SR_EPS			(1E-6)
	#define SR_ZERO			(0.0)
	#define SR_HALF			(0.5)
	#define SR_ONE			(1.0)
	#define SR_TWO			(2.0)
	#define	SR_PI_HALF		(1.57079632679489661923)	//< $\frac{\pi}{2}
	#define SR_PI			(3.14159265358979323846)	//< $\pi$
	#define SR_TWO_PI		(6.28318530717958647693)	//< $ 2 \times \pi $
	#define SR_PI_SQRT2		(2.22144146907918312351)	//< $\frac {pi}{\sqrt{2}}$
	#define SR_PI_SQR		(9.86960440108935861883)	//< $\pi^2$
	#define SR_ONETHIRD		(0.33333333333333333333)	//< $\frac{1}{3}$
	#define SR_ONESIXTH		(0.16666666666666666667)	//< $\frac{1}{6}$
	#define SR_FOURTHIRD	(1.33333333333333333333)	//< $\frac{4}{3}$
	#define SR_RADIAN		(0.01745329251994329577)	//< pi / 180
	#define SR_DEGREE		(57.2957795130823208768)	//< 180 / pi

	#define SR_ISZERO(x)	(fabs(x) < SR_EPS)			// zero test for floating point numbers
	#define SR_ISEQUAL(x,y)	(fabs((x) - (y)) < SR_EPS) // test for equality of float numbers
	#define SR_ROUND(x)		(floor((x) + 0.5))			// floating point number rounding
	#define SR_RAND(l,u)	((double)rand() / RAND_MAX * ((u) - (l)) + (l))	// float random number from interval < l ; u >
#else
	// 기본형이 float 일 때,
//	typedef float			sr_real;
	#define SR_EPS			(0.000001f)
	#define SR_ZERO			(0.0f)
	#define SR_HALF			(0.5f)
	#define SR_ONE			(1.0f)
	#define SR_TWO			(2.0f)
	#define	SR_PI_HALF		(1.57079632679489661923f)	//< $\frac{\pi}{2}
	#define SR_PI			(3.14159265358979323846f)	//< $\pi$
	#define SR_TWO_PI		(6.28318530717958647693f)	//< $ 2 \times \pi $
	#define SR_PI_SQRT2		(2.22144146907918312351f)	//< $\frac {pi}{\sqrt{2}}$
	#define SR_PI_SQR		(9.86960440108935861883f)	//< $\pi^2$
	#define SR_ONETHIRD		(0.33333333333333333333f)	//< $\frac{1}{3}$
	#define SR_ONESIXTH		(0.16666666666666666667f)	//< $\frac{1}{6}$
	#define SR_FOURTHIRD	(1.33333333333333333333f)	//< $\frac{4}{3}$
	#define SR_RADIAN		(0.01745329251994329577f)	//< pi / 180
	#define SR_DEGREE		(57.2957795130823208768f)	//< 180 / pi

	#define SR_ISZERO(x)	(abs(x) < EPSILON)     // zero test for floating point numbers
	#define SR_ISEQUAL(x,y)	(abs((x) - (y)) < EPSILON) // test for equality of float numbers
	#define SR_ROUND(x)		(floor((x) + 0.5))			// floating point number rounding
	#define SR_RAND(l,u)	((float)rand() / RAND_MAX * ((u) - (l)) + (l))	// float random number from interval < l ; u >
#endif

inline sr_real DEG2RAD(int d)	 { return (d * SR_RADIAN); }
inline sr_real RAD2DEG(int r)	 { return (r * SR_DEGREE); }
inline sr_real DEG2RAD(float d) { return (d * SR_RADIAN); }
inline sr_real RAD2DEG(float r) { return (r * SR_DEGREE); }
inline sr_real DEG2RAD(double d){ return (d * SR_RADIAN); }
inline sr_real RAD2DEG(double r){ return (r * SR_DEGREE); }

typedef	short int			Int16;		//< 2 bytes integer -32768 ~ 32767
typedef unsigned short int	UInt16;		//< 2 bytes integer 0 ~ 65535
typedef int					Int32;		//< 4 bytes integer -2147483648 ~ 2147483647
typedef unsigned int		UInt32;		//< 4 bytes integer 0 ~ 4294967255


// Sensor Message
#define SR_SENSOR_MSG_NA	-1
#endif

