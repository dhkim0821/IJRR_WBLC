/*!*****************************************************************************
[ SNU Robotics Graphic ]

author: JeongSeok Lee

Version information
v0.001 : 2008-11-11			Jeongseok Lee
*******************************************************************************/

#ifndef __SRG_COLOR__
#define __SRG_COLOR__

#include "srgL.h"

#define SRG_COLOR_DEFAULT_RED				(0.5f)
#define SRG_COLOR_DEFAULT_GREEN				(0.5f)
#define SRG_COLOR_DEFAULT_BLUE				(0.5f)
#define SRG_COLOR_DEFAULT_ALPHA				(1.0f)

#define SRG_COLOR_DEFAULT_AMBIENT_FACTOR	(1.0f)
#define SRG_COLOR_DEFAULT_DIFFUSE_FACTOR	(1.0f)
#define SRG_COLOR_DEFAULT_SPECULAR_FACTOR	(0.95f)
#define SRG_COLOR_DEFAULT_EMISSION_FACTOR	(0.08f)
#define SRG_COLOR_DEFAULT_SHININESS_FACTOR	(0.55f)

/*! \class srgString
	OpenGL Color without light factor.
*/
class srgColor
{
public:
	/*! Constructor. */
	srgColor();

	/*! Constructor. */
	srgColor(float Red, float Green, float Blue, float Alpha = 1.0f);

	/*! Constructor. */
	srgColor(const srgColor& rhs);

	/*! Destructor. */
	~srgColor();



	/*! Set Red factor. */
	void SetRed(float Red);

	/*! Set Green factor. */
	void SetGreen(float Green);

	/*! Set Blue factor. */
	void SetBlue(float Blue);

	/*! Set Alpha factor. */
	void SetAlpha(float Alpha);

	/*! Set RGB. */
	void SetRGB(float Red, float Green, float Blue);

	/*! Set RGBA. */
	void SetRGBA(float Red, float Green, float Blue, float Alpha);
	
	/*! Get Red factor. */
	float GetRed(void);

	/*! Get Green factor. */
	float GetGreen(void);

	/*! Get Blue factor. */
	float GetBlue(void);

	/*! Get Alpha factor. */
	float GetAlpha(void);

	/*! Get RGBA. */
	const float* const GetRGBA(void);



	/*! = operator */
	srgColor& operator = (const srgColor &rhs);

	/*! + operator */
	srgColor operator + (const srgColor &rhs);

	/*! - operator */
	srgColor operator - (const srgColor &rhs);

	/*! * operator */
	srgColor operator * (const float f);

	/*! == operator */
	bool operator == (const srgColor &rhs);



	/*! Use color */
	void PushAttrib(void) const;

	/*! Unuse color */
	void PopAttrib(void) const;

	/*! Apply color */
	void FadeIn(float fValue = 0.1f);

	/*! Release color */
	void FadeOut(float fValue = 0.1f);

	// RGBA struct
	typedef struct _sRGBA { float R; float G; float B; float A; } SRGBA;
	
	// union
	union { SRGBA sRGBA; float m_RGBA[4]; };

protected:
	void _Clamp();
};



/*! \class srgMaterialColor
	OpenGL color with light factors.
*/
class srgMaterialColor
{
public:
	/*! Constructor */
	srgMaterialColor();

	/*! Constructor */
	srgMaterialColor(float Red, float Green, float Blue, float Alpha = 1.0f);

	/*! Constructor */
	srgMaterialColor(float Red, float Green, float Blue,
		float Alpha,
		float Ambient,
		float Diffuse,
		float Specular,
		float Emission,
		float Shininess);

	/*! Destructor */
	virtual ~srgMaterialColor() {}



	/*! Set color */
	void	SetColor(float Red, float Green, float Blue,
		float Alpha,
		float Ambient,
		float Diffuse,
		float Specular,
		float Emission,
		float Shininess);

	/*! Set color */
	void	SetColor(float Red, float Green, float Blue, float Alpha = 1.0f);

	/*! Get color */
	const float* const	GetColor();

	/*! Set transparency */
	void SetTransp(float _alpha);

	/*! Get transparency */
	float GetTransp(void);

	/*! Set ambient color */
	void	SetAmbient(float fValue);

	/*! Get ambient color */
	const float* const	GetAmbient(void);
	
	/*! Set diffuse color */
	void	SetDiffuse(float fValue);

	/*! Get diffuse color */
	const float* const	GetDiffuse(void);

	/*! Set specular color */
	void	SetSpecular(float fValue);

	/*! Get specular color */
	const float* const	GetSpecular(void);

	/*! Set emission color */
	void	SetEmission(float fValue);

	/*! Get emission color */
	const float* const	GetEmission(void);

	/*! Get shininess */
	float	GetShininess(void);

	/*! Set shininess */
	void	SetShininess(float _shine);

	/*! Apply color */
	void	PushAttrib(void) const;

	/*! Release color */
	void	PopAttrib(void) const;

	/*! Apply color */
	void	PushAttrib_OnlyColor4(void) const;

	/*! Release color */
	void	PopAttrib_OnlyColor4(void) const;

	/*! = operator */
	srgMaterialColor& operator = (const srgMaterialColor &rhs);

	/*! == operator */
	bool operator == (const srgMaterialColor &rhs);

protected:
	srgColor _Color;
	float _fAmbientFactor;
	float _fDiffuseFactor;
	float _fSpecularFactor;
	float _fEmissionFactor;
	float _fShininessFactor;

	struct sMaterialColor
	{
		float Ambient[4];		// material property for ambient lighting	(∞£¡¢)
		float Diffuse[4];		// material property for diffuse lighting (∫–ªÍ)
		float Specular[4];		// material property for specular lighting (π›ªÁ)
		float Emission[4];		// material property for emission lighting (πﬂ±§)
		float Shininess;		// material property for shininess (±§≈√)
	};

	void _UpdateMaterialColor();
	sMaterialColor	_MaterialColor;
};



#endif // __SRG_COLOR__

