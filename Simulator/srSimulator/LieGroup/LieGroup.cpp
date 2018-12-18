//////////////////////////////////////////////////////////////////////////////////
//
//		title		:	LieGroup.cpp
//						
//		version		:	v0.987
//		author		:	Jinwook Kim (jwkim@imrc.kist.re.kr)
//		last update	:	2004.9.13
//
//////////////////////////////////////////////////////////////////////////////////

#include "LieGroup.h"
#include <iomanip>



ostream &operator << (ostream &os, const Vec3 &v)
{
	ios_base::fmtflags flags = os.setf(ios::left | ios::fixed);
	streamsize sz = os.precision(3);
	os << "[ ";
	for ( int i = 0; i < 3; i++ )
	{ 
		if ( v[i] >= 0.0 ) os << " " << setw(6) << v[i] << " ";
		else os << setw(7) << v[i] << " ";
	}
	os << "];" << endl;
	os.setf(flags);
	os.precision(sz);
	return os;
}

ostream &operator << (ostream &os, const Axis &v)
{
	ios_base::fmtflags flags = os.setf(ios::left | ios::fixed);
	streamsize sz = os.precision(3);
	os << "[ ";
	for ( int i = 0; i < 3; i++ )
	{ 
		if ( v[i] >= 0.0 ) os << " " << setw(6) << v[i] << " ";
		else os << setw(7) << v[i] << " ";
	}
	os << "];" << endl;
	os.setf(flags);
	os.precision(sz);
	return os;
}

ostream &operator << (ostream &os, const se3 &s)
{
	ios_base::fmtflags flags = os.setf(ios::left | ios::fixed);
	streamsize sz = os.precision(3);
	os << "[ ";
	for ( int i = 0; i < 6; i++ )
	{ 
		if ( s[i] >= 0.0 ) os << " " << setw(6) << s[i] << " ";
		else os << setw(7) << s[i] << " ";
	}
	os << "];" << endl;
	os.setf(flags);
	os.precision(sz);
	return os;
}

ostream &operator << (ostream &os, const dse3 &t)
{
	ios_base::fmtflags flags = os.setf(ios::left | ios::fixed);
	streamsize sz = os.precision(3);
	os << "[ ";
	for ( int i = 0; i < 6; i++ )
	{ 
		if ( t[i] >= 0.0 ) os << " " << setw(6) << t[i] << " ";
		else os << setw(7) << t[i] << " ";
	}
	os << "];" << endl;
	os.setf(flags);
	os.precision(sz);
	return os;
}

ostream &operator << (ostream &os, const SE3 &T)
{
	ios_base::fmtflags flags = os.setf(ios::left | ios::fixed);
	streamsize sz = os.precision(3);
	os << "[" << endl;
	for ( int i = 0; i < 4; i++ )
	{
		for ( int j = 0; j < 4; j++ )
		{
			if ( T(i,j) >= 0.0 ) os << " " << setw(6) << T(i,j) << " ";
			else os << setw(7) << T(i,j) << " ";
		}
		os << ";" << endl;
	}
	os << "];" << endl;
	os.setf(flags);
	os.precision(sz);
	return os;
}


// Vec3 size: half of each dimension i.e. size[0] = width/2, size[1] = depth/2, size[2] = height/2
Inertia BoxInertia(sr_real density, const Vec3 &size)
{
	sr_real mass = (sr_real)8.0 * density * size[0] * size[1] * size[2];

	return Inertia(
		mass,	// mass
		mass * (size[1] * size[1] + size[2] * size[2]) * SR_ONETHIRD,	// ix
		mass * (size[0] * size[0] + size[2] * size[2]) * SR_ONETHIRD,	// iy
		mass * (size[0] * size[0] + size[1] * size[1]) * SR_ONETHIRD	// iz
		);
}

Inertia SphereInertia(sr_real density, sr_real rad)  
{
	sr_real mass = density * SR_FOURTHIRD * SR_PI * rad * rad * rad;
	sr_real i = (sr_real)0.4 * mass * rad * rad;

	return Inertia(mass, i, i, i);
}

Inertia CylinderInertia(sr_real density, sr_real rad, sr_real height)
{
	rad *= rad;
	sr_real mass = density * SR_PI * rad * height;
	sr_real ix = mass * height * height / (sr_real)12.0 + (sr_real)0.25 * mass * rad;

	return Inertia(
		mass,						// mass
		ix,							// ix
		ix,							// iy
		0.5 * mass * rad	// iz
		);
}


Inertia	 CapsuleInertia(sr_real density, sr_real rad, sr_real height)
{
	sr_real cyl_mass = density * SR_PI * rad * rad * height;
	sr_real sph_mass	= density * SR_FOURTHIRD * SR_PI * rad * rad * rad;

	sr_real ix = cyl_mass * height * height / (sr_real)12.0 + (sr_real)0.25 * cyl_mass * rad			// cylinder
				+ (sr_real)0.4 * sph_mass * rad * rad + (sr_real)0.25 * sph_mass * height * height;	// sphere

	return Inertia(
		cyl_mass + sph_mass,													// mass
		ix,																		// ix
		ix,																		// iy
		0.5 * cyl_mass * rad * rad + (sr_real)0.4 * sph_mass * rad * rad	// iz
		);
}





///////////////////////////////////////////////////////////////////////////////
//
// union Quaternion methods
//
///////////////////////////////////////////////////////////////////////////////

// construction of quaternion from an axis and angle

Quaternion::Quaternion(const Vec3& axis, sr_real angle)
{
	sr_real d, s, theta;

	d = Norm(axis);
	if (SR_ISZERO(d))
	{
		identity();
		return;
	}

	theta = 0.5f * angle;
	s = sin(theta) / d;
	x = s * axis[0];
	y = s * axis[1];
	z = s * axis[2];
	w = cos(theta);
}


// construction of unit quaternion from a 4x4 rotation matrix

Quaternion::Quaternion(const SE3& m)
{
	sr_real tr, s;
	int i, j, k;
	static int nxt[3] = {1, 2, 0};

	tr = m[0] + m[4] + m[8];
	if (tr > (sr_real)0.0)
	{
		s = sqrt(tr + (sr_real)1.0);
		w = s * (sr_real)0.5;
		s = (sr_real)0.5 / s;
		x = (m[7] - m[5]) * s;
		y = (m[2] - m[6]) * s;
		z = (m[3] - m[1]) * s;
	}

	else {
		i = 0;
		if (m[4] > m[0]) i = 1;
		if (m[8] > m[i*4]) i = 2;
		j = nxt[i];
		k = nxt[j];
		s = sqrt((m[i*4] - (m[j*4] + m[k*4])) + (sr_real)1.0);
		c[i] = s * (sr_real)0.5;

		if (SR_ISZERO(s)) identity();
		else {
			s = (sr_real)0.5 / s;
			w = (m[j + 3*k] - m[k + 3*j]) * s;
			c[j] = (m[i + 3*j] + m[j + 3*i]) * s;
			c[k] = (m[i + 3*k] + m[k + 3*i]) * s;
		}
	}
}


Quaternion& Quaternion::operator *= (const Quaternion q)
{
	sr_real xx, yy, zz, ww;

	xx = x;
	yy = y;
	zz = z;
	ww = w;

	x = ww * q.x + xx * q.w + yy * q.z - zz * q.y;
	y = ww * q.y + yy * q.w + zz * q.x - xx * q.z;
	z = ww * q.z + zz * q.w + xx * q.y - yy * q.x;
	w = ww * q.w - xx * q.x - yy * q.y - zz * q.z;

	return *this;
}


// inverse quaternion

Quaternion& Quaternion::invert()
{
	sr_real k;

	k = x * x + y * y + z * z + w * w;
	if (SR_ISZERO(k))
	{
		identity();
		return *this;
	}

	k = (sr_real)1.0 / k;
	x *= -k;
	y *= -k;
	z *= -k;
	w *= k;

	return *this;
}


// adjust quaternion to unit length

Quaternion& Quaternion::normalize()
{
	sr_real d, dd;

	d = length();
	if (SR_ISZERO(d))
	{
		identity();
		return *this;
	}

	dd = (sr_real)1.0 / d;
	x *= dd;
	y *= dd;
	z *= dd;
	w *= dd;

	return *this;
}


// logarithm of quaternion

Quaternion& Quaternion::log()
{
	sr_real d, k, theta;

	d = sqrt(x * x + y * y + z * z);
	theta = atan2(d, w);

	if (SR_ISZERO(d))
		k = (sr_real)1.0;
	else
		k = theta / d;

	x *= k;
	y *= k;
	z *= k;
	w = (sr_real)0.0;

	return *this;
}


// exponentiation of quaternion

Quaternion& Quaternion::exp()
{
	sr_real k, theta;

	theta = sqrt(x * x + y * y + z * z);
	if (SR_ISZERO(theta))
		k = 1.0f;
	else
		k = sin(theta) / theta;

	x *= k;
	y *= k;
	z *= k;
	w = cos(theta);

	return *this;
}


Quaternion operator + (const Quaternion& p, const Quaternion& q)
{
	return Quaternion(p.x + q.x, p.y + q.y, p.z + q.z, p.w + q.w);
}


Quaternion operator - (const Quaternion& p, const Quaternion& q)
{
	return Quaternion(p.x - q.x, p.y - q.y, p.z - q.z, p.w - q.w);
}


Quaternion operator - (const Quaternion& q)
{
	return Quaternion(-q.x, -q.y, -q.z, -q.w);
}


Quaternion operator * (sr_real k, const Quaternion& q)
{
	return Quaternion(k * q.x, k * q.y, k * q.z, k * q.w);
}


Quaternion operator * (const Quaternion& p, const Quaternion& q)
{
	return Quaternion(
		p.w * q.x + p.x * q.w + p.y * q.z - p.z * q.y,
		p.w * q.y + p.y * q.w + p.z * q.x - p.x * q.z,
		p.w * q.z + p.z * q.w + p.x * q.y - p.y * q.x,
		p.w * q.w - p.x * q.x - p.y * q.y - p.z * q.z);
}


bool operator == (const Quaternion& p, const Quaternion& q)
{

	Quaternion d = p - q;
	return SR_ISZERO(d.x * d.x + d.y * d.y + d.z * d.z + d.w * d.w);
}


bool operator != (const Quaternion& p, const Quaternion& q)
{
	Quaternion d = p - q;
	return !SR_ISZERO(d.x * d.x + d.y * d.y + d.z * d.z + d.w * d.w);
}


// dot product of two quaternions

sr_real dot(const Quaternion& p, const Quaternion& q)
{
	return p.x * q.x + p.y * q.y + p.z * q.z + p.w * q.w;
}


// what is the small cosine to change from spherical to linear interpolation 
#define SLERP_EPSILON   0.05f

Quaternion slerp(const Quaternion& p, const Quaternion& q, sr_real t)
{
	sr_real theta, sintheta, costheta, pscl, qscl = 1.0f;

	costheta = p.x * q.x + p.y * q.y + p.z * q.z + p.w * q.w;

	// if angle between quaternions is > PI/2, negate q, to choose
	// the shorter one of two paths for interpolation
	if (costheta < 0.0f)
	{
		qscl = -1.0f;
		costheta = -costheta;
	}

	// for close quaternions use linear interpolation instead
	if ((1 - costheta) < SLERP_EPSILON)
	{
		pscl = 1.0f - t;
		qscl *= t;
	}
	// otherwise actually do spherical interpolation
	else
	{
		theta = acos(costheta);
		sintheta = sin(theta);
		pscl = sin((1.0f - t) * theta) / sintheta;
		qscl *= sin(t * theta) / sintheta;
	}

	return Quaternion(pscl * p.x + qscl * q.x,
		pscl * p.y + qscl * q.y,
		pscl * p.z + qscl * q.z,
		pscl * p.w + qscl * q.w);
}


// print a quaternion

std::ostream& operator << (std::ostream& os, const Quaternion& q)
{
	return os << '(' << q.x << ", " << q.y << ", " << q.z << ", " << q.w << ')';
}
