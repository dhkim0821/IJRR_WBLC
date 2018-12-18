//////////////////////////////////////////////////////////////////////
//
// OpenGL Texture Class
// by: Matthew Fairfax
//
// GLTexture.h: interface for the GLTexture class.
// This class loads a texture file and prepares it
// to be used in OpenGL. It can open a bitmap or a
// targa file. The min filter is set to mipmap b/c
// they look better and the performance cost on
// modern video cards in negligible. I leave all of
// the texture management to the application. I have
// included the ability to load the texture from a
// Visual Studio resource. The bitmap's id must be
// be surrounded by quotation marks (i.e. "Texture.bmp").
// The targa files must be in a resource type of "TGA"
// (including the quotes). The targa's id must be
// surrounded by quotation marks (i.e. "Texture.tga").
//
// Usage:
// GLTexture tex;
// GLTexture tex1;
// GLTexture tex3;
//
// tex.Load("texture.bmp"); // Loads a bitmap
// tex.Use();				// Binds the bitmap for use
// 
// tex1.LoadFromResource("texture.tga"); // Loads a targa
// tex1.Use();				 // Binds the targa for use
//
// // You can also build a texture with a single color and use it
// tex3.BuildColorTexture(255, 0, 0);	// Builds a solid red texture
// tex3.Use();				 // Binds the targa for use
//
//	Jaeyoung Haan, Apr 14, 2009
//	Change class name.
//	Make platform-independent.
//
//////////////////////////////////////////////////////////////////////

#ifndef TEXTURE_H
#define TEXTURE_H

#include "srg/srgL.h"

class Texture  
{
protected:
	unsigned int texture[1];	// OpenGL's number for the texture
	char*	m_TextureName;		// The textures name
	int		m_Width;			// Texture's width
	int		m_Height;			// Texture's height

public:
			 Texture();			// Constructor
	virtual ~Texture();			// Destructor

	void	Use();							// Binds the texture for use
	void	BuildColorTexture(unsigned char r, unsigned char g, unsigned char b);	// Sometimes we want a texture of uniform color
	void	LoadTGA(char *name);			// Loads a targa file
	void	LoadBMP(char *name);			// Loads a bitmap file
  void	LoadPNG(char *name);			// Loads a png file
	void	Load(char *name);				// Load the texture
  bool  loadPngImage(char *name, int &outWidth, int &outHeight, bool &outHasAlpha, GLubyte **outData);
};

#endif 

