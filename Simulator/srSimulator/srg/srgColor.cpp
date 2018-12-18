#include "srgColor.h"

srgColor::srgColor()
{
};

srgColor::srgColor(float Red, float Green, float Blue, float Alpha)
{
	if (Red > 1.0f) Red = 1.0f;	if (Green > 1.0f) Green = 1.0f;
	if (Blue > 1.0f) Blue = 1.0f;	if (Alpha > 1.0f) Alpha = 1.0f;
	if (Red < 0.0f) Red = 0.0f;	if (Green < 0.0f) Green = 0.0f;
	if (Blue < 0.0f) Blue = 0.0f;	if (Alpha < 0.0f) Alpha = 0.0f;

	m_RGBA[0] = Red;
	m_RGBA[1] = Green;
	m_RGBA[2] = Blue;
	m_RGBA[3] = Alpha;
}

srgColor::~srgColor() {};

void srgColor::SetRed(float Red)		{m_RGBA[0] = Red;};
void srgColor::SetGreen(float Green)	{m_RGBA[1] = Green;};
void srgColor::SetBlue(float Blue)	{m_RGBA[2] = Blue;};
void srgColor::SetAlpha(float Alpha)	{m_RGBA[3] = Alpha;};
void srgColor::SetRGB(float Red, float Green, float Blue)
{
	m_RGBA[0] = Red;
	m_RGBA[1] = Green;
	m_RGBA[2] = Blue;
}
void srgColor::SetRGBA(float Red, float Green, float Blue, float Alpha)
{
	m_RGBA[0] = Red;
	m_RGBA[1] = Green;
	m_RGBA[2] = Blue;
	m_RGBA[3] = Alpha;
}

float srgColor::GetRed(void)		{return m_RGBA[0];};
float srgColor::GetGreen(void)	{return m_RGBA[1];};
float srgColor::GetBlue(void)		{return m_RGBA[2];};
float srgColor::GetAlpha(void)	{return m_RGBA[3];};

// TODO: validate!
const float* const srgColor::GetRGBA(void)	{return m_RGBA;};


srgColor::srgColor(const srgColor& rhs)
{
	m_RGBA[0] = rhs.m_RGBA[0];
	m_RGBA[1] = rhs.m_RGBA[1];
	m_RGBA[2] = rhs.m_RGBA[2];
	m_RGBA[3] = rhs.m_RGBA[3];
};

//String & operator=(const String &);
srgColor& srgColor::operator = (const srgColor &rhs)
{
	if(this != &rhs)
	{
		m_RGBA[0] = rhs.m_RGBA[0];
		m_RGBA[1] = rhs.m_RGBA[1];
		m_RGBA[2] = rhs.m_RGBA[2];
		m_RGBA[3] = rhs.m_RGBA[3];
	}
	return *this;
};
//const rglMaterialColor& operator = (const rglMaterialColor &rhs);

srgColor srgColor::operator + (const srgColor &rhs)
{
	srgColor res;
	res.m_RGBA[0] = m_RGBA[0] + rhs.m_RGBA[0];
	res.m_RGBA[1] = m_RGBA[1] + rhs.m_RGBA[1];
	res.m_RGBA[2] = m_RGBA[2] + rhs.m_RGBA[2];
	res.m_RGBA[3] = m_RGBA[3] + rhs.m_RGBA[3];
	return res;
};

srgColor srgColor::operator - (const srgColor &rhs)
{
	srgColor res;
	res.m_RGBA[0] = m_RGBA[0] - rhs.m_RGBA[0];
	res.m_RGBA[1] = m_RGBA[1] - rhs.m_RGBA[1];
	res.m_RGBA[2] = m_RGBA[2] - rhs.m_RGBA[2];
	res.m_RGBA[3] = m_RGBA[3] - rhs.m_RGBA[3];
	return res;
};

srgColor srgColor::operator * (const float f)
{
	srgColor res;
	res.m_RGBA[0] = m_RGBA[0]*f;
	res.m_RGBA[1] = m_RGBA[1]*f;
	res.m_RGBA[2] = m_RGBA[2]*f;
	res.m_RGBA[3] = m_RGBA[3]*f;
	return res;
};

bool srgColor::operator == (const srgColor &rhs)
{
	return (m_RGBA[0] == rhs.m_RGBA[0])
		&& (m_RGBA[1] == rhs.m_RGBA[1])
		&& (m_RGBA[2] == rhs.m_RGBA[2])
		&& (m_RGBA[3] == rhs.m_RGBA[3]);
};

void srgColor::_Clamp()
{
	if (m_RGBA[0] > 1.0f) m_RGBA[0] = 1.0f;		if (m_RGBA[1] > 1.0f) m_RGBA[1] = 1.0f;
	if (m_RGBA[2] > 1.0f) m_RGBA[2] = 1.0f;		if (m_RGBA[3] > 1.0f) m_RGBA[3] = 1.0f;
	if (m_RGBA[0] < 0.0f) m_RGBA[0] = 0.0f;		if (m_RGBA[1] < 0.0f) m_RGBA[1] = 0.0f;
	if (m_RGBA[2] < 0.0f) m_RGBA[2] = 0.0f;		if (m_RGBA[3] < 0.0f) m_RGBA[3] = 0.0f;
}

void srgColor::PushAttrib(void) const
{
	glPushAttrib(GL_CURRENT_BIT);
		glColor4fv(m_RGBA);
};
void srgColor::PopAttrib(void) const
{
	glPopAttrib();
};



//////////////////////////////////////////////////////////////////////////

srgMaterialColor::srgMaterialColor():
_Color(SRG_COLOR_DEFAULT_RED, SRG_COLOR_DEFAULT_GREEN, SRG_COLOR_DEFAULT_BLUE, SRG_COLOR_DEFAULT_ALPHA),
_fAmbientFactor(SRG_COLOR_DEFAULT_AMBIENT_FACTOR),
_fDiffuseFactor(SRG_COLOR_DEFAULT_DIFFUSE_FACTOR),
_fSpecularFactor(SRG_COLOR_DEFAULT_SPECULAR_FACTOR),
_fEmissionFactor(SRG_COLOR_DEFAULT_EMISSION_FACTOR),
_fShininessFactor(SRG_COLOR_DEFAULT_SHININESS_FACTOR)
{
	_UpdateMaterialColor();
};

srgMaterialColor::srgMaterialColor(float Red, float Green, float Blue, float Alpha):
_Color(Red, Green, Blue, Alpha),
_fAmbientFactor(SRG_COLOR_DEFAULT_AMBIENT_FACTOR),
_fDiffuseFactor(SRG_COLOR_DEFAULT_DIFFUSE_FACTOR),
_fSpecularFactor(SRG_COLOR_DEFAULT_SPECULAR_FACTOR),
_fEmissionFactor(SRG_COLOR_DEFAULT_EMISSION_FACTOR),
_fShininessFactor(SRG_COLOR_DEFAULT_SHININESS_FACTOR)
{
	_UpdateMaterialColor();
}


srgMaterialColor::srgMaterialColor(float Red, float Green, float Blue,
							   float Alpha,
							   float Ambient,
							   float Diffuse,
							   float Specular,
							   float Emission,
							   float Shininess):
_Color(Red, Green, Blue, Alpha),
_fAmbientFactor(Ambient),
_fDiffuseFactor(Diffuse),
_fSpecularFactor(Specular),
_fEmissionFactor(Emission),
_fShininessFactor(Shininess)
{
	_UpdateMaterialColor();
}

void srgMaterialColor::SetColor(float Red, float Green, float Blue,
							  float Alpha,
							  float Ambient,
							  float Diffuse,
							  float Specular,
							  float Emission,
							  float Shininess)
{
	_Color.m_RGBA[0] = Red;
	_Color.m_RGBA[1] = Green;
	_Color.m_RGBA[2] = Blue;
	_Color.m_RGBA[3] = Alpha;

	_fAmbientFactor = Ambient;
	_fDiffuseFactor = Diffuse;
	_fSpecularFactor = Specular;
	_fEmissionFactor = Emission;
	_fShininessFactor = Shininess;

	_UpdateMaterialColor();
}

void srgMaterialColor::SetColor(float Red, float Green, float Blue, float Alpha)
{
	_Color.m_RGBA[0] = Red;
	_Color.m_RGBA[1] = Green;
	_Color.m_RGBA[2] = Blue;
	_Color.m_RGBA[3] = Alpha;

	_UpdateMaterialColor();
}

const float* const srgMaterialColor::GetColor(void)
{
	return _Color.GetRGBA();
}

//
inline void srgMaterialColor::_UpdateMaterialColor()
{
	_MaterialColor.Ambient[0] = _fAmbientFactor*_Color.m_RGBA[0];
	_MaterialColor.Ambient[1] = _fAmbientFactor*_Color.m_RGBA[1];
	_MaterialColor.Ambient[2] = _fAmbientFactor*_Color.m_RGBA[2];
	_MaterialColor.Ambient[3] = _Color.m_RGBA[3];

	_MaterialColor.Diffuse[0] = _fDiffuseFactor*_Color.m_RGBA[0];
	_MaterialColor.Diffuse[1] = _fDiffuseFactor*_Color.m_RGBA[1];
	_MaterialColor.Diffuse[2] = _fDiffuseFactor*_Color.m_RGBA[2];
	_MaterialColor.Diffuse[3] = _Color.m_RGBA[3];

	_MaterialColor.Specular[0] = _fSpecularFactor*_Color.m_RGBA[0];
	_MaterialColor.Specular[1] = _fSpecularFactor*_Color.m_RGBA[1];
	_MaterialColor.Specular[2] = _fSpecularFactor*_Color.m_RGBA[2];
	_MaterialColor.Specular[3] = _Color.m_RGBA[3];

	_MaterialColor.Emission[0] = _fEmissionFactor*_Color.m_RGBA[0];
	_MaterialColor.Emission[1] = _fEmissionFactor*_Color.m_RGBA[1];
	_MaterialColor.Emission[2] = _fEmissionFactor*_Color.m_RGBA[2];
	_MaterialColor.Emission[3] = _Color.m_RGBA[3];

	_MaterialColor.Shininess	= _fShininessFactor*128.0f;
  // printf("m RGBA: %f, %f, %f, %f \n", _Color.m_RGBA[0],_Color.m_RGBA[1],_Color.m_RGBA[2],_Color.m_RGBA[3]);
};


void srgMaterialColor::PushAttrib(void) const
{
	glPushAttrib(GL_LIGHTING_BIT);
	glMaterialfv(GL_FRONT, GL_AMBIENT, _MaterialColor.Ambient);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, _MaterialColor.Diffuse);
	glMaterialfv(GL_FRONT, GL_SPECULAR, _MaterialColor.Specular);
	glMaterialfv(GL_FRONT, GL_EMISSION, _MaterialColor.Emission);
	glMaterialf(GL_FRONT, GL_SHININESS, _MaterialColor.Shininess);
}

void srgMaterialColor::PopAttrib(void) const
{
	glPopAttrib();
}

void srgMaterialColor::PushAttrib_OnlyColor4(void) const
{
	glPushAttrib(GL_COLOR_BUFFER_BIT);
	glColor4fv(_Color.m_RGBA);
}
void srgMaterialColor::PopAttrib_OnlyColor4(void) const
{
	glPopAttrib();
}

//==============================================================================
// TRANSPARENCY
//==============================================================================
void srgMaterialColor::SetTransp(float _alpha)
{
	_Color.m_RGBA[3]
	= _MaterialColor.Ambient[3]
	= _MaterialColor.Diffuse[3]
	= _MaterialColor.Specular[3]
	= _MaterialColor.Emission[3]
	= _alpha;
}
float srgMaterialColor::GetTransp(void)
{
	return _Color.m_RGBA[3];
}

//==============================================================================
// AMBIENT
//==============================================================================
void srgMaterialColor::SetAmbient(float fValue)
{
	_fAmbientFactor = fValue;

	_MaterialColor.Ambient[0] = _fAmbientFactor*_Color.m_RGBA[0];
	_MaterialColor.Ambient[1] = _fAmbientFactor*_Color.m_RGBA[1];
	_MaterialColor.Ambient[2] = _fAmbientFactor*_Color.m_RGBA[2];
}
const float* const	srgMaterialColor::GetAmbient(void)
{
	return _MaterialColor.Ambient;
}

//==============================================================================
// DIFFUSE
//==============================================================================
void srgMaterialColor::SetDiffuse(float fValue)
{
	_fAmbientFactor = fValue;

	_MaterialColor.Diffuse[0] = _fDiffuseFactor*_Color.m_RGBA[0];
	_MaterialColor.Diffuse[1] = _fDiffuseFactor*_Color.m_RGBA[1];
	_MaterialColor.Diffuse[2] = _fDiffuseFactor*_Color.m_RGBA[2];
}

const float* const	srgMaterialColor::GetDiffuse(void)
{
	return _MaterialColor.Diffuse;
}

//==============================================================================
// SPECULAR
//==============================================================================
void srgMaterialColor::SetSpecular(float fValue)
{
	_fSpecularFactor = fValue;

	_MaterialColor.Specular[0] = _fSpecularFactor*_Color.m_RGBA[0];
	_MaterialColor.Specular[1] = _fSpecularFactor*_Color.m_RGBA[1];
	_MaterialColor.Specular[2] = _fSpecularFactor*_Color.m_RGBA[2];
}

const float* const	srgMaterialColor::GetSpecular(void)
{
	return _MaterialColor.Specular;
}

//==============================================================================
// EMISSION
//==============================================================================
void srgMaterialColor::SetEmission(float fValue)
{
	_fEmissionFactor = fValue;

	_MaterialColor.Emission[0] = _fEmissionFactor*_Color.m_RGBA[0];
	_MaterialColor.Emission[1] = _fEmissionFactor*_Color.m_RGBA[1];
	_MaterialColor.Emission[2] = _fEmissionFactor*_Color.m_RGBA[2];
}

const float* const	srgMaterialColor::GetEmission(void)
{
	return _MaterialColor.Emission;
}

//==============================================================================
// SHININESS
//==============================================================================
float	srgMaterialColor::GetShininess(void)
{
	return _fShininessFactor;
}

void srgMaterialColor::SetShininess(float m_fShininessFactor)
{
	m_fShininessFactor = m_fShininessFactor;
}


//==============================================================================
// AMBIENT
//==============================================================================
srgMaterialColor& srgMaterialColor::operator = (const srgMaterialColor &rhs)
{
	_MaterialColor = rhs._MaterialColor;
	return *this;
}

bool srgMaterialColor::operator == (const srgMaterialColor &rhs)
{
	return (_MaterialColor.Ambient		== rhs._MaterialColor.Ambient)
		&& (_MaterialColor.Diffuse		== rhs._MaterialColor.Diffuse)
		&& (_MaterialColor.Emission	== rhs._MaterialColor.Emission)
		&& (_MaterialColor.Specular	== rhs._MaterialColor.Specular)
		&& (_MaterialColor.Shininess	== rhs._MaterialColor.Shininess);
}

void srgColor::FadeIn(float fValue)
{
	m_RGBA[0] += fValue - m_RGBA[0]*fValue;
	m_RGBA[1] += fValue - m_RGBA[1]*fValue;
	m_RGBA[2] += fValue - m_RGBA[2]*fValue;

	_Clamp();
}

void srgColor::FadeOut(float fValue)
{
	m_RGBA[0] *= fValue;
	m_RGBA[1] *= fValue;
	m_RGBA[2] *= fValue;

	_Clamp();
}























//==============================================================================
// CODE POOL
//==============================================================================

//rglMaterialColor::rglMaterialColor(float _AmbientR, float _AmbientG, float _AmbientB,
//								 float _DiffuseR, float _DiffuseG, float _DiffuseB,
//								 float _SpecularR, float _SpecularG, float _SpecularB,
//								 float _EmissionR, float _EmissionG, float _EmissionB,
//								 float _Shininess)
//{
//	m_MaterialColor.Ambient[0] = _AmbientR;
//	m_MaterialColor.Ambient[1] = _AmbientG;
//	m_MaterialColor.Ambient[2] = _AmbientB;
//	m_MaterialColor.Ambient[3] = 1.0f;
//
//	m_MaterialColor.Diffuse[0] = _DiffuseR;
//	m_MaterialColor.Diffuse[1] = _DiffuseG;
//	m_MaterialColor.Diffuse[2] = _DiffuseB;
//	m_MaterialColor.Diffuse[3] = 1.0f;
//
//	m_MaterialColor.Specular[0] = _SpecularR;
//	m_MaterialColor.Specular[1] = _SpecularG;
//	m_MaterialColor.Specular[2] = _SpecularB;
//	m_MaterialColor.Specular[3] = 1.0f;
//
//	m_MaterialColor.Emission[0] = _EmissionR;
//	m_MaterialColor.Emission[1] = _EmissionG;
//	m_MaterialColor.Emission[2] = _EmissionB;
//	m_MaterialColor.Emission[3] = 1.0f;
//
//	m_MaterialColor.Shininess	= _Shininess;
//};
//
//rglMaterialColor::rglMaterialColor(	float _AmbientR,	float _AmbientG,	float _AmbientB,	float _AmbientAlpha,
//									float _DiffuseR,	float _DiffuseG,	float _DiffuseB,	float _DiffuseAlpha,
//									float _SpecularR,	float _SpecularG,	float _SpecularB,	float _SpecularAlpha,
//									float _EmissionR,	float _EmissionG,	float _EmissionB,	float _EmissionAlpha,
//									float _Shininess)
//{
//	m_MaterialColor.Ambient[0] = _AmbientR;
//	m_MaterialColor.Ambient[1] = _AmbientG;
//	m_MaterialColor.Ambient[2] = _AmbientB;
//	m_MaterialColor.Ambient[3] = _AmbientAlpha;
//
//	m_MaterialColor.Diffuse[0] = _DiffuseR;
//	m_MaterialColor.Diffuse[1] = _DiffuseG;
//	m_MaterialColor.Diffuse[2] = _DiffuseB;
//	m_MaterialColor.Diffuse[3] = _DiffuseAlpha;
//
//	m_MaterialColor.Specular[0] = _SpecularR;
//	m_MaterialColor.Specular[1] = _SpecularG;
//	m_MaterialColor.Specular[2] = _SpecularB;
//	m_MaterialColor.Specular[3] = _SpecularAlpha;
//
//	m_MaterialColor.Emission[0] = _EmissionR;
//	m_MaterialColor.Emission[1] = _EmissionG;
//	m_MaterialColor.Emission[2] = _EmissionB;
//	m_MaterialColor.Emission[3] = _EmissionAlpha;
//
//	m_MaterialColor.Shininess	= _Shininess;
//};
//
//void rglMaterialColor::SetMeterialColor(	float _AmbientR,	float _AmbientG,	float _AmbientB,	float _AmbientAlpha,
//									   float _DiffuseR,	float _DiffuseG,	float _DiffuseB,	float _DiffuseAlpha,
//									   float _SpecularR,	float _SpecularG,	float _SpecularB,	float _SpecularAlpha,
//									   float _EmissionR,	float _EmissionG,	float _EmissionB,	float _EmissionAlpha,
//									   float _Shininess)
//{
//	m_MaterialColor.Ambient[0] = _AmbientR;
//	m_MaterialColor.Ambient[1] = _AmbientG;
//	m_MaterialColor.Ambient[2] = _AmbientB;
//	m_MaterialColor.Ambient[3] = _AmbientAlpha;
//
//	m_MaterialColor.Diffuse[0] = _DiffuseR;
//	m_MaterialColor.Diffuse[1] = _DiffuseG;
//	m_MaterialColor.Diffuse[2] = _DiffuseB;
//	m_MaterialColor.Diffuse[3] = _DiffuseAlpha;
//
//	m_MaterialColor.Specular[0] = _SpecularR;
//	m_MaterialColor.Specular[1] = _SpecularG;
//	m_MaterialColor.Specular[2] = _SpecularB;
//	m_MaterialColor.Specular[3] = _SpecularAlpha;
//
//	m_MaterialColor.Emission[0] = _EmissionR;
//	m_MaterialColor.Emission[1] = _EmissionG;
//	m_MaterialColor.Emission[2] = _EmissionB;
//	m_MaterialColor.Emission[3] = _EmissionAlpha;
//
//	m_MaterialColor.Shininess	= _Shininess;
//}
//
//void rglMaterialColor::SetMeterialColor(float _AmbientR, float _AmbientG, float _AmbientB,
//								 float _DiffuseR, float _DiffuseG, float _DiffuseB,
//								 float _SpecularR, float _SpecularG, float _SpecularB,
//								 float _EmissionR, float _EmissionG, float _EmissionB,
//								 float _Shininess)
//{
//	m_MaterialColor.Ambient[0] = _AmbientR;
//	m_MaterialColor.Ambient[1] = _AmbientG;
//	m_MaterialColor.Ambient[2] = _AmbientB;
//	m_MaterialColor.Ambient[3] = 1.0f;
//
//	m_MaterialColor.Diffuse[0] = _DiffuseR;
//	m_MaterialColor.Diffuse[1] = _DiffuseG;
//	m_MaterialColor.Diffuse[2] = _DiffuseB;
//	m_MaterialColor.Diffuse[3] = 1.0f;
//
//	m_MaterialColor.Specular[0] = _SpecularR;
//	m_MaterialColor.Specular[1] = _SpecularG;
//	m_MaterialColor.Specular[2] = _SpecularB;
//	m_MaterialColor.Specular[3] = 1.0f;
//
//	m_MaterialColor.Emission[0] = _EmissionR;
//	m_MaterialColor.Emission[1] = _EmissionG;
//	m_MaterialColor.Emission[2] = _EmissionB;
//	m_MaterialColor.Emission[3] = 1.0f;
//
//	m_MaterialColor.Shininess	= _Shininess;
//};


//void rglMaterialColor::SetMeterialColor(const sMaterialColor& _color)
//{
//	m_MaterialColor.Ambient[0] = _color.Ambient[0];
//	m_MaterialColor.Ambient[1] = _color.Ambient[1];
//	m_MaterialColor.Ambient[2] = _color.Ambient[2];
//	m_MaterialColor.Ambient[3] = _color.Ambient[3];
//	m_MaterialColor.Diffuse[0] = _color.Diffuse[0];
//	m_MaterialColor.Diffuse[1] = _color.Diffuse[1];
//	m_MaterialColor.Diffuse[2] = _color.Diffuse[2];
//	m_MaterialColor.Diffuse[3] = _color.Diffuse[3];
//	m_MaterialColor.Emission[0] = _color.Emission[0];
//	m_MaterialColor.Emission[1] = _color.Emission[1];
//	m_MaterialColor.Emission[2] = _color.Emission[2];
//	m_MaterialColor.Emission[3] = _color.Emission[3];
//	m_MaterialColor.Specular[0] = _color.Specular[0];
//	m_MaterialColor.Specular[1] = _color.Specular[1];
//	m_MaterialColor.Specular[2] = _color.Specular[2];
//	m_MaterialColor.Specular[3] = _color.Specular[3];
//	m_MaterialColor.Shininess = _color.Shininess;
//}
//
//sMaterialColor rglMaterialColor::GetMeterialColor(void)
//{
//	sMaterialColor color;
//
//	color.Ambient[0] = m_MaterialColor.Ambient[0];
//	color.Ambient[1] = m_MaterialColor.Ambient[1];     
//	color.Ambient[2] = m_MaterialColor.Ambient[2];
//	color.Ambient[3] = m_MaterialColor.Ambient[3];
//	color.Diffuse[0] = m_MaterialColor.Diffuse[0];
//	color.Diffuse[1] = m_MaterialColor.Diffuse[1];
//	color.Diffuse[2] = m_MaterialColor.Diffuse[2];
//	color.Diffuse[3] = m_MaterialColor.Diffuse[3];
//	color.Emission[0] = m_MaterialColor.Emission[0];
//	color.Emission[1] = m_MaterialColor.Emission[1];
//	color.Emission[2] = m_MaterialColor.Emission[2];
//	color.Emission[3] = m_MaterialColor.Emission[3];
//	color.Specular[0] = m_MaterialColor.Specular[0];
//	color.Specular[1] = m_MaterialColor.Specular[1];
//	color.Specular[2] = m_MaterialColor.Specular[2];
//	color.Specular[3] = m_MaterialColor.Specular[3];
//	color.Shininess = m_MaterialColor.Shininess;
//
//	return color;
//}

