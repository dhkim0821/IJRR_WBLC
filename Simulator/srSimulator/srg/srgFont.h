/*******************************************************************************
[ SNU Robotics Graphic ]

author: JeongSeok Lee

Version information
v0.001 : 2008-11-12			Jeongseok Lee
-
*******************************************************************************/

#ifndef __SRG_FONT__
#define __SRG_FONT__

#include "srgL.h"
#include "srgMath.h"

//#include <list>
//#include <vector>
//#include <string>
//#include <stdio.h>
//#include <string.h>

/** \class srgString
	OpenGL GULT Text.
*/
class srgString
{
public:
	/** Constructor. */
	srgString();

	/** Destructor. */
	~srgString();

	// 
	typedef enum {
		MODE_BITMAP,
		//MODE_STROKE
	} mode_type;

	/** Change font */
	void ChangFont(mode_type NewType, unsigned int FontIndex);

	/** Print 2d text on view.
	  *	x and y indicate the location of text, left-bottom.	*/
	void Print(int x, int y, const char* string, ...);

protected:
	void _PrintBitmapString(void* font, const char* s);
	void _PrintStrokeString(void* font, const char* s);

	mode_type _Mode;
	int _FontIndex;
	float _StrokeScale;

	void* bitmap_fonts[7];
	void* bitmap_font_names[7];
	void* stroke_fonts[2];
	void* stroke_font_names[2];
private:
};



#endif // __SRG_FONT__
