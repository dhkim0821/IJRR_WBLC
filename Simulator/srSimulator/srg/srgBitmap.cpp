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


#include "srgBitmap.h"
#include <stdio.h>

_Bitmap::_Bitmap()
{
	m_Width = 0;
	m_Height = 0;
	m_Data = 0;
}

_Bitmap::~_Bitmap()
{
	delete [] m_Data;
}

BYTE& _Bitmap::_pixel(int x,int y,int c)
{
	return m_Data[(y * m_Width + x) * 3 + c];
}

void _Bitmap::_allocateMem()
{
	delete [] m_Data;
	m_Data = new BYTE[m_Width * m_Height * 3];
}

int	_Bitmap::GetWidth()
{
	return m_Width;
}

int	_Bitmap::GetHeight()
{
	return m_Height;
}

BYTE* _Bitmap::GetData()
{
	return m_Data;
}

string _Bitmap::TranslateBMPError(BMPError err)
{
	switch(err)
	{
	case(BMPNOTABITMAP):
		return "This file is not a bitmap, specifically it doesn't start 'BM'";
	case(BMPNOOPEN):
		return "Failed to open the file, suspect it doesn't exist";
	case(BMPFILEERROR):
		return "ferror said we had an error. This error seems to not always mean anything, try ignoring it";
	case(BMPBADINT):
		return "sizeof(int)!=4 quite a lot of rewriting probably needs to be done on the code";
	case(BMPNOERROR):
		return "No errors detected";
	case(BMPUNKNOWNFORMAT):
		return "Unknown bmp format, ie not 24bit, 256,16 or 2 colour";
	default:
		return "Not a valid error code";
	}
}

BMPError _Bitmap::BMPLoad(string fname)
{
	return BMPLoad((char*)fname.c_str());
}

BMPError _Bitmap::BMPLoad(char* fname)
{
	if(sizeof(int) != 4) return BMPBADINT;
		
	FILE* f = fopen(fname, "rb");		//open for reading in binary mode

	if(!f) return BMPNOOPEN;

	char header[54];

	size_t ret = fread(header, 54, 1, f);			//read the 54bit main header

	if(header[0] != 'B' || header[1] != 'M') 
	{
		fclose(f);
		return BMPNOTABITMAP;		//all bitmaps should start "BM"
	}

	//it seems gimp sometimes makes its headers small, so we have to do this. hence all the fseeks
	int offset = *(unsigned int*)(header + 10);
	
	m_Width  = *(int*)(header + 18);
	m_Height = *(int*)(header + 22);

	//now the bitmap knows how big it is it can allocate its memory
	_allocateMem();

	int bits = int(header[28]);		//color depth

	int x, y, c;
	BYTE cols[256 * 4];				//color table

	switch(bits)
	{
	case(24):
		fseek(f, offset, SEEK_SET);
		ret = fread(m_Data, m_Width * m_Height * 3, 1, f);	//24bit is easy
		for(x = 0; x < m_Width * m_Height * 3; x += 3)	//except the format is BGR, grr
		{
			BYTE temp = m_Data[x];
			m_Data[x] = m_Data[x + 2];
			m_Data[x + 2] = temp;
		}
		break;

	case(8):
		ret = fread(cols, 256 * 4, 1, f);							//read color table
		fseek(f, offset, SEEK_SET);
		for(y = 0; y < m_Height; ++y)						//(Notice 4bytes/col for some reason)
			for(x = 0; x < m_Width; ++x)
			{
				BYTE byte;			
				fread(&byte, 1, 1, f);						//just read byte					
				for(int c = 0; c < 3; ++c)
					_pixel(x, y, c) = cols[byte * 4 + 2 - c];	//and look up in the table
			}
		break;

	case(4):
		ret = fread(cols, 16 * 4, 1, f);
		fseek(f, offset, SEEK_SET);
		for(y = 0; y < 256; ++y)
			for(x = 0; x < 256; x += 2)
			{
				BYTE byte;
				fread(&byte, 1, 1, f);						//as above, but need to extract two
				for(c = 0; c < 3; ++c)						//pixels from each byte
					_pixel(x, y, c) = cols[byte / 16 * 4 + 2 - c];
				for(c = 0; c < 3; ++c)
					_pixel(x + 1, y, c) = cols[byte % 16 * 4 + 2 - c];
			}
		break;

	case(1):
		ret = fread(cols, 8, 1, f);
		fseek(f, offset, SEEK_SET);
		for(y = 0; y < m_Height; ++y)
			for(x = 0; x < m_Width; x += 8)
			{
				BYTE byte;
				ret = fread(&byte, 1, 1, f);
				//Every byte is eight pixels
				//so I'm shifting the byte to the relevant position, then masking out
				//all but the lowest bit in order to get the index into the color table.
				for(int x2 = 0; x2 < 8; ++x2)
					for(int c = 0; c < 3; ++c)
						_pixel(x + x2, y, c) = cols[((byte >> (7 - x2))&1) * 4 + 2 - c];
			}
		break;

	default:
		fclose(f);
		return BMPUNKNOWNFORMAT;
	}

	if(ferror(f))
	{
		fclose(f);
		return BMPFILEERROR;
	}
	
	fclose(f);

	return BMPNOERROR;
}

#ifdef __gl_h
BMPError _Bitmap::BMPLoadGL(string fname)
{
	BMPError e = BMPLoad(fname);
	if(e!=BMPNOERROR) return e;
		
	glEnable(GL_TEXTURE_2D);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexImage2D(GL_TEXTURE_2D,0,3,m_Width,m_Height,0,GL_RGB,GL_UNSIGNED_BYTE,m_Data);

	return BMPNOERROR;
}
#endif
