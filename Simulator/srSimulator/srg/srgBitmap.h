/*
	BMPLoader - loads Microsoft .bmp format
    Copyright (C) 2006  Chris Backhouse

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along
    with this program; if not, write to the Free Software Foundation, Inc.,
    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.


  cjbackhouse@hotmail.com 		www.backhouse.tk
  
  I would appreciate it if anyone using this in something cool would tell me
  so I can see where it ends up.

  Takes a filename, returns an array of RGB pixel data
  Loads:
  24bit bitmaps
  256 colour bitmaps
  16 colour bitmaps
  2 colour bitmaps  (Thanks to Charles Rabier)

  This code is designed for use in openGL programs, so bitmaps not correctly padded will not
  load properly, I believe this only applies to: 
  256cols if width is not a multiple of 4
  16cols if width is not a multiple of 8
  2cols if width is not a multiple of 32

  Sample code:

	BMPClass bmp;
	BMPLoad(fname,bmp);
	glTexImage2D(GL_TEXTURE_2D,0,3,bmp.width,bmp.height,0,GL_RGB,GL_UNSIGNED_BYTE,bmp.bytes);


	Jaeyoung Haan, Apr 14, 2009
	Change class name BMPClass -> _Bitmap.
	Functions got to be member functions.
	Member variables got to be protected and "Get~" member functions added. 
*/

#ifndef BMPLOADER_H
#define BMPLOADER_H

#ifdef WIN32
	#define _CRT_SECURE_NO_DEPRECATE 
	#define _CRT_SECURE_NO_WARNINGS 
	#define _CRT_NONSTDC_NO_DEPRECATE 

	#pragma warning ( disable:4996 )
#endif

#include <string>

#define BMPError			char
#define BMPNOTABITMAP		'b'		//Possible error flags
#define BMPNOOPEN			'o'
#define BMPFILEERROR		'f'
#define BMPBADINT			'i'
#define BMPNOERROR			'\0'
#define BMPUNKNOWNFORMAT	'u'

typedef unsigned char BYTE;

using namespace std;

class _Bitmap
{
protected:
	int		m_Width;
	int		m_Height;
	BYTE*	m_Data;

protected:
	BYTE&	_pixel(int x, int y, int c);
	void	_allocateMem();

public:
			 _Bitmap();
			~_Bitmap();

	int		GetWidth();
	int		GetHeight();
	BYTE*	GetData();

	// Translates my error codes into English	
	string		TranslateBMPError(BMPError err);	

	// Load and select in OpenGL
	BMPError	BMPLoadGL(string fname);

	// Load bmp file.
	BMPError	BMPLoad(string fname);
	BMPError	BMPLoad(char* fname);
};

typedef	_Bitmap	srgBitmap;

#endif
