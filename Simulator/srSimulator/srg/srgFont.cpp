#include "srgFont.h"
#include <string.h>
#include <stdarg.h>
#include <fstream>
#include <algorithm>
#include <iostream>
#include <float.h>
//#using <mscorlib.dll>

srgString::srgString()
{
	_Mode = MODE_BITMAP;
	_FontIndex = 0;
	_StrokeScale = 1.0f;

	bitmap_fonts[0] = GLUT_BITMAP_9_BY_15;
	bitmap_fonts[1] = GLUT_BITMAP_8_BY_13;
	bitmap_fonts[2] = GLUT_BITMAP_TIMES_ROMAN_10;
	bitmap_fonts[3] = GLUT_BITMAP_TIMES_ROMAN_24;
	bitmap_fonts[4] = GLUT_BITMAP_HELVETICA_10;
	bitmap_fonts[5] = GLUT_BITMAP_HELVETICA_12;
	bitmap_fonts[6] = GLUT_BITMAP_HELVETICA_18;

	bitmap_font_names[0] = GLUT_BITMAP_9_BY_15;
	bitmap_font_names[1] = GLUT_BITMAP_8_BY_13;
	bitmap_font_names[2] = GLUT_BITMAP_TIMES_ROMAN_10;
	bitmap_font_names[3] = GLUT_BITMAP_TIMES_ROMAN_24;
	bitmap_font_names[4] = GLUT_BITMAP_HELVETICA_10;
	bitmap_font_names[5] = GLUT_BITMAP_HELVETICA_12;
	bitmap_font_names[6] = GLUT_BITMAP_HELVETICA_18;

	stroke_fonts[0] = GLUT_STROKE_ROMAN;
	stroke_fonts[1] = GLUT_STROKE_MONO_ROMAN;

	stroke_font_names[0] = GLUT_STROKE_ROMAN;
	stroke_font_names[1] = GLUT_STROKE_MONO_ROMAN;
}

srgString::~srgString()
{

}

void srgString::_PrintBitmapString( void* font, const char* s )
{
	if (s && strlen(s)) {
		while (*s) {
			glutBitmapCharacter(font, *s);
			s++;
		}
	}
}

void srgString::_PrintStrokeString( void* font, const char* s )
{
	if (s && strlen(s)) {
		while (*s) {
			glutStrokeCharacter(font, *s);
			s++;
		}
	}
}

void srgString::ChangFont( mode_type NewType, unsigned int FontIndex )
{
	_Mode = NewType;

	if (_Mode == MODE_BITMAP)
	{
		_FontIndex = FontIndex%7;
	} 
	else
	{
		_FontIndex = FontIndex%2;
	}
}

void srgString::Print( int x, int y, const char* string, ... )
{
	char text[1024];
	va_list va;

	if ( !string ) return;

	va_start(va, string);
#ifdef WIN32
	vsprintf_s(text, string, va);
#else
	vsprintf(text, string, va);
#endif
	va_end(va);

	if (_Mode == MODE_BITMAP)
	{
		//glWindowPos2i(x, y);
		glRasterPos2i(x, y);

		_PrintBitmapString(bitmap_fonts[_FontIndex], text);
	}
// 	else
// 	{
// 		//glMatrixMode(GL_MODELVIEW);
// 		//glPushMatrix();
// 		//{
// 		//	glTranslatef(x, y, 0.0);
// 		//	glScalef(stroke_scale, stroke_scale, stroke_scale);
// 		//	print_stroke_string(stroke_fonts[font_index], string);
// 		//}
// 		//glPopMatrix();
// 
// 		glWindowPos2i(x, y);
// 		//glScalef(stroke_scale, stroke_scale, stroke_scale);
// 		print_stroke_string(stroke_fonts[font_index], text);
// 	}
}

//srgFont::srgFont()
//{
//	//m_hDC = NULL;		// Private GDI Device Context
//}
//
//srgFont::~srgFont(void)
//{
//}
//
//GLvoid srgFont::BuildFont(HDC hDC, char *szFontName)								// Build Our Bitmap Font
//{
//	HFONT	hFont;										// Windows Font ID
//
//	m_uBase = glGenLists(96);								// Storage For 96 Characters
//	hFont = CreateFont(-24,								// Height Of Font
//		0,								// Width Of Font
//		0,								// Angle Of Escapement
//		0,								// Orientation Angle
//		FW_NORMAL,						// Font Weight
//		FALSE,							// Italic
//		FALSE,							// Underline
//		FALSE,							// Strikeout
//		ANSI_CHARSET,					// Character Set Identifier
//		OUT_TT_PRECIS,					// Output Precision
//		CLIP_DEFAULT_PRECIS,			// Clipping Precision
//		ANTIALIASED_QUALITY,			// Output Quality
//		FF_DONTCARE|DEFAULT_PITCH,		// Family And Pitch
//		szFontName);					// Font Name
//	SelectObject(hDC, szFontName);							// Selects The Font We Want
//	wglUseFontBitmaps(hDC, 32, 96, m_uBase);				// Builds 96 Characters Starting At Character 32
//}
//
//GLvoid srgFont::KillFont(GLvoid)									// Delete The Font
//{
//	glDeleteLists(m_uBase, 96);							// Delete All 96 Characters
//}
//
//GLvoid srgFont::glPrint(const char *text)
//{
//	glPushAttrib(GL_LIST_BIT);							// Pushes The Display List Bits
//	glListBase(m_uBase - 32);								// Sets The Base Character to 32
//	glCallLists(strlen(text), GL_UNSIGNED_BYTE, text);	// Draws The Display List Text
//	glPopAttrib();										// Pops The Display List Bits
//}



//unsigned int srgString::base = 0;
//
//void srgString::print(int x, int y, const char * string, ...)
//{
//	if ( !base )
//	{
//		base = glGenLists(255);
//
//		HFONT font = CreateFont(20, 0, 0, 0, FW_BOLD, FALSE, FALSE, FALSE, ANSI_CHARSET, OUT_TT_PRECIS, CLIP_DEFAULT_PRECIS, ANTIALIASED_QUALITY, FF_DONTCARE|DEFAULT_PITCH, "Arial");
//		HFONT oldfont = (HFONT)SelectObject(wglGetCurrentDC(), font);
//		wglUseFontBitmaps(wglGetCurrentDC(), 0, 255, base);
//		SelectObject(wglGetCurrentDC(), oldfont);
//		DeleteObject(font);
//	}
//
//	char text[1024];
//	va_list va;
//
//	if ( !string ) return;
//
//	va_start(va, string);
//	vsprintf(text, string, va);
//	va_end(va);
//
//	glWindowPos2i(x, y);
//
//	glPushAttrib(GL_LIST_BIT);
//	glListBase(base);
//	glCallLists((GLsizei)strlen(text), GL_UNSIGNED_BYTE, text);
//	glPopAttrib();
//}
//
//void srgString::setFont(const char font_name[], int font_height, int font_width)
//{
//	if ( base ) glDeleteLists(base, 255);
//	base = glGenLists(255);
//
//	HFONT font = CreateFont(font_height, font_width, 0, 0, FW_BOLD, 0, 0, 0, ANSI_CHARSET, OUT_TT_PRECIS, CLIP_DEFAULT_PRECIS, ANTIALIASED_QUALITY, FF_DONTCARE|DEFAULT_PITCH, font_name);
//	HFONT oldfont = (HFONT)SelectObject(wglGetCurrentDC(), font);
//	wglUseFontBitmaps(wglGetCurrentDC(), 0, 255, base);
//	SelectObject(wglGetCurrentDC(), oldfont);
//	DeleteObject(font);
//}
