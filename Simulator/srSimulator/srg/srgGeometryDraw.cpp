#include "srgGeometryDraw.h"
#include <math.h>
#include <stdio.h>

void srgDrawBox_FeatureLine(double HalfWidth, double HalfHeight, double HalfDepth)
{
	//double x = Width/2.0;
	//double y = Height/2.0;
	//double z = Depth/2.0;

	double x = HalfWidth;
	double y = HalfHeight;
	double z = HalfDepth;

	glBegin(GL_LINE_LOOP);
	{
		glVertex3d( x,  y, -z);
		glVertex3d(-x,  y, -z);
		glVertex3d(-x,  y,  z);
		glVertex3d( x,  y,  z);
	}
	glEnd();
	glBegin(GL_LINE_LOOP);
	{
		glVertex3d( x, -y, -z);
		glVertex3d( x,  y, -z);
		glVertex3d( x,  y,  z);
		glVertex3d( x, -y,  z);
	}
	glEnd();
	glBegin(GL_LINE_LOOP);
	{
		glVertex3d( x, -y,  z);
		glVertex3d(-x, -y,  z);
		glVertex3d(-x, -y, -z);
		glVertex3d( x, -y, -z);
	}
	glEnd();
	glBegin(GL_LINE_LOOP);
	{
		glVertex3d(-x, -y,  z);
		glVertex3d(-x,  y,  z);
		glVertex3d(-x,  y, -z);
		glVertex3d(-x, -y, -z);
	}
	glEnd();
	glBegin(GL_LINE_LOOP);
	{
		glVertex3d(-x, -y, -z);
		glVertex3d(-x,  y, -z);
		glVertex3d( x,  y, -z);
		glVertex3d( x, -y, -z);
	}
	glEnd();
	glBegin(GL_LINE_LOOP);
	{
		glVertex3d(-x, -y,  z);
		glVertex3d( x, -y,  z);
		glVertex3d( x,  y,  z);
		glVertex3d(-x,  y,  z);
	} //GL_QUADS
	glEnd();	
}

void srgDrawBox(double HalfWidth, double HalfHeight, double HalfDepth)
{
	//double x = Width/2.0;
	//double y = Height/2.0;
	//double z = Depth/2.0;

	double x = HalfWidth;
	double y = HalfHeight;
	double z = HalfDepth;

	glBegin(GL_QUADS);
	{
		glVertex3d( x,  y, -z);
		glVertex3d(-x,  y, -z);
		glVertex3d(-x,  y,  z);
		glVertex3d( x,  y,  z);

		glVertex3d( x, -y, -z);
		glVertex3d( x,  y, -z);
		glVertex3d( x,  y,  z);
		glVertex3d( x, -y,  z);

		glVertex3d( x, -y,  z);
		glVertex3d(-x, -y,  z);
		glVertex3d(-x, -y, -z);
		glVertex3d( x, -y, -z);

		glVertex3d(-x, -y,  z);
		glVertex3d(-x,  y,  z);
		glVertex3d(-x,  y, -z);
		glVertex3d(-x, -y, -z);

		glVertex3d(-x, -y, -z);
		glVertex3d(-x,  y, -z);
		glVertex3d( x,  y, -z);
		glVertex3d( x, -y, -z);

		glVertex3d(-x, -y,  z);
		glVertex3d( x, -y,  z);
		glVertex3d( x,  y,  z);
		glVertex3d(-x,  y,  z);
	} //GL_QUADS
	glEnd();	
}

void srgDrawBox_NV(double HalfWidth, double HalfHeight, double HalfDepth)
{
	double x = HalfWidth;
	double y = HalfHeight;
	double z = HalfDepth;

	glBegin(GL_QUADS);
	{
		glNormal3d( 0.0,  1.0,  0.0);
		glVertex3d( x,  y, -z);
		glVertex3d(-x,  y, -z);
		glVertex3d(-x,  y,  z);
		glVertex3d( x,  y,  z);

		glNormal3d( 1.0,  0.0,  0.0);
		glVertex3d( x, -y, -z);
		glVertex3d( x,  y, -z);
		glVertex3d( x,  y,  z);
		glVertex3d( x, -y,  z);

		glNormal3d( 0.0, -1.0,  0.0);
		glVertex3d( x, -y,  z);
		glVertex3d(-x, -y,  z);
		glVertex3d(-x, -y, -z);
		glVertex3d( x, -y, -z);

		glNormal3d(-1.0,  0.0,  0.0);
		glVertex3d(-x, -y,  z);
		glVertex3d(-x,  y,  z);
		glVertex3d(-x,  y, -z);
		glVertex3d(-x, -y, -z);

		glNormal3d( 0.0,  0.0, -1.0);
		glVertex3d(-x, -y, -z);
		glVertex3d(-x,  y, -z);
		glVertex3d( x,  y, -z);
		glVertex3d( x, -y, -z);

		glNormal3d( 0.0,  0.0,  1.0);
		glVertex3d(-x, -y,  z);
		glVertex3d( x, -y,  z);
		glVertex3d( x,  y,  z);
		glVertex3d(-x,  y,  z);
	} //GL_QUADS
	glEnd();	
}

void srgDrawBox_NV_TEX(double HalfWidth, double HalfHeight, double HalfDepth)
{
	double x = HalfWidth;
	double y = HalfHeight;
	double z = HalfDepth;

	glBegin(GL_QUADS);
	{
		glNormal3d( 0.0,  1.0,  0.0);
		glTexCoord2f(0, 0); glVertex3d( x,  y, -z);
		glTexCoord2f(1, 0); glVertex3d(-x,  y, -z);
		glTexCoord2f(1, 1); glVertex3d(-x,  y,  z);
		glTexCoord2f(0, 1); glVertex3d( x,  y,  z);

		glNormal3d( 1.0,  0.0,  0.0);
		glTexCoord2f(0, 0); glVertex3d( x, -y, -z);
		glTexCoord2f(1, 0); glVertex3d( x,  y, -z);
		glTexCoord2f(1, 1); glVertex3d( x,  y,  z);
		glTexCoord2f(0, 1); glVertex3d( x, -y,  z);

		glNormal3d( 0.0, -1.0,  0.0);
		glTexCoord2f(0, 0); glVertex3d( x, -y,  z);
		glTexCoord2f(1, 0); glVertex3d(-x, -y,  z);
		glTexCoord2f(1, 1); glVertex3d(-x, -y, -z);
		glTexCoord2f(0, 1); glVertex3d( x, -y, -z);

		glNormal3d(-1.0,  0.0,  0.0);
		glTexCoord2f(0, 0); glVertex3d(-x, -y,  z);
		glTexCoord2f(1, 0); glVertex3d(-x,  y,  z);
		glTexCoord2f(1, 1); glVertex3d(-x,  y, -z);
		glTexCoord2f(0, 1); glVertex3d(-x, -y, -z);

		glNormal3d( 0.0,  0.0, -1.0);
		glTexCoord2f(0, 0); glVertex3d(-x, -y, -z);
		glTexCoord2f(1, 0); glVertex3d(-x,  y, -z);
		glTexCoord2f(1, 1); glVertex3d( x,  y, -z);
		glTexCoord2f(0, 1); glVertex3d( x, -y, -z);

		glNormal3d( 0.0,  0.0,  1.0);
		glTexCoord2f(0, 0); glVertex3d(-x, -y,  z);
		glTexCoord2f(1, 0); glVertex3d( x, -y,  z);
		glTexCoord2f(1, 1); glVertex3d( x,  y,  z);
		glTexCoord2f(0, 1); glVertex3d(-x,  y,  z);
	} //GL_QUADS
	glEnd();	
}

void srgDrawOpenDome_FeatureLine(double Radius, int FeatureSlices, int FeatureStacks, int Slices, int Stacks)
{
	GLfloat drho	= (GLfloat)(M_PI / Stacks / 2.0);
	GLfloat dtheta	= (GLfloat)((M_PI*2.0) / Slices);
	GLfloat dFeature_rho	= (GLfloat)(M_PI / FeatureStacks / 2.0);
	GLfloat dFeature_theta	= (GLfloat)((M_PI*2.0) / FeatureSlices);

	GLint i, j;     // Looping variables

	for (i = FeatureStacks; i > 0; i--) 
	{
		GLfloat rho = (GLfloat)i * dFeature_rho;
		GLfloat srho = (GLfloat)(sin(rho));
		GLfloat crho = (GLfloat)(cos(rho));

		glBegin(GL_LINE_STRIP);
		{
			for ( j = 0; j <= Slices; j++)
			{
				GLfloat theta = (j == Slices) ? 0.0f : j * dtheta;
				GLfloat stheta = (GLfloat)(-sin(theta));
				GLfloat ctheta = (GLfloat)(cos(theta));

				GLfloat x = srho * stheta;
				GLfloat y = srho * ctheta;
				GLfloat z = crho;

				glVertex3f(x * (GLfloat)Radius, y * (GLfloat)Radius, z * (GLfloat)Radius);
			}
		}
		glEnd();
	}

	for ( j = 0; j < FeatureSlices; j++)
	{
		GLfloat theta = (GLfloat)j * dFeature_theta;
		GLfloat stheta = (GLfloat)(-sin(theta));
		GLfloat ctheta = (GLfloat)(cos(theta));

		glBegin(GL_LINE_STRIP);

		for (i = 0; i <= Stacks; i++) 
		{
			GLfloat rho = (GLfloat)i * drho;
			GLfloat srho = (GLfloat)(sin(rho));
			GLfloat crho = (GLfloat)(cos(rho));

			GLfloat x = srho * stheta;
			GLfloat y = srho * ctheta;
			GLfloat z = crho;

			glVertex3f(x * (GLfloat)Radius, y * (GLfloat)Radius, z * (GLfloat)Radius);
		}
		glEnd();
	}
}
void srgDrawOpenDome(double Radius, int Slices, int Stacks)
{
	// (2pi/Stacks)가 
	GLfloat drho = (GLfloat)(3.141592653589 / Stacks / 2.0);
	GLfloat dtheta = (GLfloat)(2.0f * 3.141592653589 / Slices);

	GLint i, j;     // Looping variables

	for (i = 0; i < Stacks; i++) 
	{
		GLfloat rho = (GLfloat)i * drho;
		GLfloat srho = (GLfloat)(sin(rho));
		GLfloat crho = (GLfloat)(cos(rho));
		GLfloat srhodrho = (GLfloat)(sin(rho + drho));
		GLfloat crhodrho = (GLfloat)(cos(rho + drho));

		// Many sources of OpenGL sphere drawing code uses a triangle fan
		// for the caps of the sphere. This however introduces texturing 
		// artifacts at the poles on some OpenGL implementations
		glBegin(GL_TRIANGLE_STRIP);

		for ( j = 0; j <= Slices; j++)
		{
			GLfloat theta = (j == Slices) ? 0.0f : j * dtheta;
			GLfloat stheta = (GLfloat)(-sin(theta));
			GLfloat ctheta = (GLfloat)(cos(theta));

			GLfloat x = srho * stheta;
			GLfloat y = srho * ctheta;
			GLfloat z = crho;

			glVertex3f(x * (GLfloat)Radius, y * (GLfloat)Radius, z * (GLfloat)Radius);

			x = srhodrho * stheta;
			y = srhodrho * ctheta;
			z = crhodrho;

			glVertex3f(x * (GLfloat)Radius, y * (GLfloat)Radius, z * (GLfloat)Radius);
		}
		glEnd();
	}
}

void srgDrawOpenDome_NV(double Radius, int Slices, int Stacks)
{
	// (2pi/Stacks)가 
	GLfloat drho = (GLfloat)(3.141592653589 / Stacks / 2.0);
	GLfloat dtheta = 2.0f * (GLfloat)(3.141592653589 / Slices);

	GLint i, j;     // Looping variables

	GLfloat rho = drho;
	GLfloat srho = (GLfloat)(sin(rho));
	GLfloat crho = (GLfloat)(cos(rho));
	GLfloat srhodrho = (GLfloat)(sin(rho + drho));
	GLfloat crhodrho = (GLfloat)(cos(rho + drho));

	// Many sources of OpenGL sphere drawing code uses a triangle fan
	// for the caps of the sphere. This however introduces texturing 
	// artifacts at the poles on some OpenGL implementations
	glBegin(GL_TRIANGLE_FAN);
	glNormal3f(0.0f, 0.0f, (GLfloat)Radius);
	glVertex3f(0.0f, 0.0f, (GLfloat)Radius);
	for ( j = 0; j <= Slices; j++)
	{
		GLfloat theta = (j == Slices) ? 0.0f : j * dtheta;
		GLfloat stheta = (GLfloat)(-sin(theta));
		GLfloat ctheta = (GLfloat)(cos(theta));

		GLfloat x = srho * stheta;
		GLfloat y = srho * ctheta;
		GLfloat z = crho;

		glNormal3f(x, y, z);
		glVertex3f(x * (GLfloat)Radius, y * (GLfloat)Radius, z * (GLfloat)Radius);
	}
	glEnd();

	for (i = 1; i < Stacks; i++) 
	{
		GLfloat rho = (GLfloat)i * drho;
		GLfloat srho = (GLfloat)(sin(rho));
		GLfloat crho = (GLfloat)(cos(rho));
		GLfloat srhodrho = (GLfloat)(sin(rho + drho));
		GLfloat crhodrho = (GLfloat)(cos(rho + drho));

		// Many sources of OpenGL sphere drawing code uses a triangle fan
		// for the caps of the sphere. This however introduces texturing 
		// artifacts at the poles on some OpenGL implementations
		glBegin(GL_TRIANGLE_STRIP);

		for ( j = 0; j <= Slices; j++)
		{
			GLfloat theta = (j == Slices) ? 0.0f : j * dtheta;
			GLfloat stheta = (GLfloat)(-sin(theta));
			GLfloat ctheta = (GLfloat)(cos(theta));

			GLfloat x = srho * stheta;
			GLfloat y = srho * ctheta;
			GLfloat z = crho;

			glNormal3f(x, y, z);
			glVertex3f(x * (GLfloat)Radius, y * (GLfloat)Radius, z * (GLfloat)Radius);

			x = srhodrho * stheta;
			y = srhodrho * ctheta;
			z = crhodrho;

			glNormal3f(x, y, z);
			glVertex3f(x * (GLfloat)Radius, y * (GLfloat)Radius, z * (GLfloat)Radius);
		}
		glEnd();
	}
}

void srgDrawOpenDome_NV_TEX(double Radius, int Slices, int Stacks)
{
	// (2pi/Stacks)가 
	GLfloat drho = (GLfloat)(3.141592653589 / Stacks / 2.0);
	GLfloat dtheta = 2.0f * (GLfloat)(3.141592653589) / (GLfloat) Slices;
	GLfloat ds = 1.0f / (GLfloat) Slices;
	GLfloat dt = 1.0f / (GLfloat) Stacks;
	GLfloat t = 1.0f;	
	GLfloat s = 0.0f;
	GLint i, j;     // Looping variables


	GLfloat rho = drho;
	GLfloat srho = (GLfloat)(sin(rho));
	GLfloat crho = (GLfloat)(cos(rho));
	GLfloat srhodrho = (GLfloat)(sin(rho + drho));
	GLfloat crhodrho = (GLfloat)(cos(rho + drho));

	// Many sources of OpenGL sphere drawing code uses a triangle fan
	// for the caps of the sphere. This however introduces texturing 
	// artifacts at the poles on some OpenGL implementations
	glBegin(GL_TRIANGLE_STRIP);
	s = 0.0f;
	for ( j = 0; j <= Slices; j+=2)
	{
		GLfloat theta = (j == Slices) ? 0.0f : j * dtheta;
		GLfloat stheta = (GLfloat)(-sin(theta));
		GLfloat ctheta = (GLfloat)(cos(theta));

		GLfloat x = srho * stheta;
		GLfloat y = srho * ctheta;
		GLfloat z = crho;

		glTexCoord2f(s, t);
		glNormal3f(x, y, z);
		glVertex3f(x * (GLfloat)Radius, y * (GLfloat)Radius, z * (GLfloat)Radius);

		x = srhodrho * stheta;
		y = srhodrho * ctheta;
		z = crhodrho;

		glTexCoord2f(s, t - dt);
		s += ds;
		glNormal3f(x, y, z);
		glVertex3f(x * (GLfloat)Radius, y * (GLfloat)Radius, z * (GLfloat)Radius);
	}
	glEnd();
	t -= dt;

	for (i = 1; i < Stacks; i++) 
	{
		GLfloat rho = (GLfloat)i * drho;
		GLfloat srho = (GLfloat)(sin(rho));
		GLfloat crho = (GLfloat)(cos(rho));
		GLfloat srhodrho = (GLfloat)(sin(rho + drho));
		GLfloat crhodrho = (GLfloat)(cos(rho + drho));

		// Many sources of OpenGL sphere drawing code uses a triangle fan
		// for the caps of the sphere. This however introduces texturing 
		// artifacts at the poles on some OpenGL implementations
		glBegin(GL_TRIANGLE_STRIP);
		s = 0.0f;
		for ( j = 0; j <= Slices; j++)
		{
			GLfloat theta = (j == Slices) ? 0.0f : j * dtheta;
			GLfloat stheta = (GLfloat)(-sin(theta));
			GLfloat ctheta = (GLfloat)(cos(theta));

			GLfloat x = srho * stheta;
			GLfloat y = srho * ctheta;
			GLfloat z = crho;

			glTexCoord2f(s, t);
			glNormal3f(x, y, z);
			glVertex3f(x * (GLfloat)Radius, y * (GLfloat)Radius, z * (GLfloat)Radius);

			x = srhodrho * stheta;
			y = srhodrho * ctheta;
			z = crhodrho;

			glTexCoord2f(s, t - dt);
			s += ds;
			glNormal3f(x, y, z);
			glVertex3f(x * (GLfloat)Radius, y * (GLfloat)Radius, z * (GLfloat)Radius);
		}
		glEnd();

		t -= dt;
	}
}

void srgDrawOpenCylinder_FeatureLine(double BaseRadius, double TopRadius, double Height, int FeatureSlices, int Slices)
{
	//GLfloat drho	= (GLfloat)(M_PI / Stacks / 2.0);
	GLfloat dtheta	= (GLfloat)((M_PI*2.0) / Slices);
	//GLfloat dFeature_rho	= (GLfloat)(M_PI / FeatureStacks / 2.0);
	GLfloat dFeature_theta	= (GLfloat)((M_PI*2.0) / FeatureSlices);

	GLint j;     // Looping variables

	// 윗 디스크
	glBegin(GL_LINE_STRIP);
	{
		for ( j = 0; j <= Slices; j++)
		{
			GLfloat theta = (j == Slices) ? 0.0f : j * dtheta;
			GLfloat stheta = (GLfloat)(-sin(theta));
			GLfloat ctheta = (GLfloat)(cos(theta));

			GLfloat x = (GLfloat)TopRadius*stheta;
			GLfloat y = (GLfloat)TopRadius*ctheta;
			GLfloat z = (GLfloat)Height/2.0f;

			glVertex3f(x, y, z);
		}
	}
	glEnd();

	// 아래 디스크
	glBegin(GL_LINE_STRIP);
	{
		for ( j = 0; j <= Slices; j++)
		{
			GLfloat theta = (j == Slices) ? 0.0f : j * dtheta;
			GLfloat stheta = (GLfloat)(-sin(theta));
			GLfloat ctheta = (GLfloat)(cos(theta));

			GLfloat x = (GLfloat)BaseRadius*stheta;
			GLfloat y = (GLfloat)BaseRadius*ctheta;
			GLfloat z = (GLfloat)-Height/2.0f;

			glVertex3f(x, y, z);
		}
	}
	glEnd();

	// 세로줄
	for ( j = 0; j < FeatureSlices; j++)
	{
		GLfloat theta = (GLfloat)j * dFeature_theta;
		GLfloat stheta = (GLfloat)(-sin(theta));
		GLfloat ctheta = (GLfloat)(cos(theta));

		GLfloat x = stheta;
		GLfloat y = ctheta;

		glBegin(GL_LINES);
		{
			glVertex3f(x * (GLfloat)TopRadius, y * (GLfloat)TopRadius, (GLfloat)Height/2.0f);
			glVertex3f(x * (GLfloat)BaseRadius, y * (GLfloat)BaseRadius, (GLfloat)-Height/2.0f);
		}
		glEnd();
	}
}

//void srgDrawOpenCylinder(double BaseRadius, double TopRadius, double Height, int Slices)
//{
//	//GLfloat drho	= (GLfloat)(M_PI / Stacks / 2.0);
//	GLfloat dtheta	= (GLfloat)((M_PI*2.0) / Slices);
//	//GLfloat dFeature_rho	= (GLfloat)(M_PI / FeatureStacks / 2.0);
//	GLfloat dFeature_theta	= (GLfloat)((M_PI*2.0) / FeatureSlices);
//
//	GLint j;     // Looping variables
//
//	// 윗 디스크
//	glBegin(GL_LINE_STRIP);
//	{
//		for ( j = 0; j <= Slices; j++)
//		{
//			GLfloat theta = (j == Slices) ? 0.0f : j * dtheta;
//			GLfloat stheta = (GLfloat)(-sin(theta));
//			GLfloat ctheta = (GLfloat)(cos(theta));
//
//			GLfloat x = (GLfloat)TopRadius*stheta;
//			GLfloat y = (GLfloat)TopRadius*ctheta;
//			GLfloat z = (GLfloat)Height/2.0f;
//
//			glVertex3f(x, y, z);
//		}
//	}
//	glEnd();
//
//	// 아래 디스크
//	glBegin(GL_LINE_STRIP);
//	{
//		for ( j = 0; j <= Slices; j++)
//		{
//			GLfloat theta = (j == Slices) ? 0.0f : j * dtheta;
//			GLfloat stheta = (GLfloat)(-sin(theta));
//			GLfloat ctheta = (GLfloat)(cos(theta));
//
//			GLfloat x = (GLfloat)BaseRadius*stheta;
//			GLfloat y = (GLfloat)BaseRadius*ctheta;
//			GLfloat z = (GLfloat)-Height/2.0f;
//
//			glVertex3f(x, y, z);
//		}
//	}
//	glEnd();
//
//	// 세로줄
//	for ( j = 0; j < FeatureSlices; j++)
//	{
//		GLfloat theta = (GLfloat)j * dFeature_theta;
//		GLfloat stheta = (GLfloat)(-sin(theta));
//		GLfloat ctheta = (GLfloat)(cos(theta));
//
//		GLfloat x = stheta;
//		GLfloat y = ctheta;
//
//		glBegin(GL_LINES);
//		{
//			glVertex3f(x * (GLfloat)TopRadius, y * (GLfloat)TopRadius, (GLfloat)Height/2.0f);
//			glVertex3f(x * (GLfloat)BaseRadius, y * (GLfloat)BaseRadius, (GLfloat)-Height/2.0f);
//		}
//		glEnd();
//	}
//}

void srgDrawSphere_FeatureLine(double Radius, int FeatureSlices, int FeatureStacks, int Slices, int Stacks)
{
	GLfloat drho	= (GLfloat)(M_PI / Stacks);
	GLfloat dtheta	= (GLfloat)((M_PI*2.0) / Slices);
	GLfloat dFeature_rho	= (GLfloat)(M_PI / FeatureStacks);
	GLfloat dFeature_theta	= (GLfloat)((M_PI*2.0) / FeatureSlices);

	GLint i, j;     // Looping variables

	for (i = 1; i < FeatureStacks; i++) 
	{
		GLfloat rho = (GLfloat)i * dFeature_rho;
		GLfloat srho = (GLfloat)(sin(rho));
		GLfloat crho = (GLfloat)(cos(rho));

		glBegin(GL_LINE_STRIP);
		{
			for ( j = 0; j <= Slices; j++)
			{
				GLfloat theta = (j == Slices) ? 0.0f : j * dtheta;
				GLfloat stheta = (GLfloat)(-sin(theta));
				GLfloat ctheta = (GLfloat)(cos(theta));

				GLfloat x = srho * stheta;
				GLfloat y = srho * ctheta;
				GLfloat z = crho;

				glVertex3f(x * (GLfloat)Radius, y * (GLfloat)Radius, z * (GLfloat)Radius);
			}
		}
		glEnd();
	}

	for ( j = 0; j < FeatureSlices; j++)
	{
		GLfloat theta = (GLfloat)j * dFeature_theta;
		GLfloat stheta = (GLfloat)(-sin(theta));
		GLfloat ctheta = (GLfloat)(cos(theta));

		glBegin(GL_LINE_STRIP);

		for (i = 0; i <= Stacks; i++) 
		{
			GLfloat rho = (GLfloat)i * drho;
			GLfloat srho = (GLfloat)(sin(rho));
			GLfloat crho = (GLfloat)(cos(rho));

			GLfloat x = srho * stheta;
			GLfloat y = srho * ctheta;
			GLfloat z = crho;

			glVertex3f(x * (GLfloat)Radius, y * (GLfloat)Radius, z * (GLfloat)Radius);
		}
		glEnd();
	}
}

void srgDrawLoop(float Radius, int Slices)
{
	float fX, fY;
	float dAngle = 0.0;
	float dAngleIncrease = (float)(M_PI*2.0) / Slices;
	//float dAngleIncrease = 360.0/ Slice;

	glBegin(GL_LINE_STRIP);
	{
		glVertex3f(Radius, 0.0f, 0.0f);
		for (int i = 0; i < Slices; i++)
		{
			dAngle += dAngleIncrease;
			//dAngle += .5;
			fX = Radius*cos(dAngle);
			fY = Radius*sin(dAngle);
			glVertex3f(fX, fY, 0.0f);
		}
		glVertex3f(Radius, 0.0f, 0.0f);
	}
	glEnd();
}

void srgDrawCircle(float Radius, int Slices)
{
	float fX, fY;
	float dAngle = 0.0;
	float dAngleIncrease = (float)(M_PI*2.0) / Slices;
	//float dAngleIncrease = 360.0/ Slice;

	glBegin(GL_POLYGON);
	{
		//glVertex3f(Radius, 0.0f, 0.0f);
		for (int i = 0; i < Slices; i++)
		{
			dAngle += dAngleIncrease;
			//dAngle += .5;
			fX = Radius*cos(dAngle);
			fY = Radius*sin(dAngle);
			glVertex3f(fX, fY, 0.0f);
		}
		//glVertex3f(Radius, 0.0f, 0.0f);
	}
	glEnd();
}

void srgDraw2DSphere(float Radius, int Slices, int Stacks)
{
	GLfloat drho = (GLfloat)(M_PI) / (GLfloat) Stacks;
	GLfloat dtheta = 2.0f * (GLfloat)(M_PI) / (GLfloat) Slices;
	GLfloat ds = 1.0f / (GLfloat) Slices;
	GLfloat dt = 1.0f / (GLfloat) Stacks;
	GLfloat t = 1.0f;	
	GLfloat s = 0.0f;
	GLint i, j;     // Looping variables

	for (i = 0; i < Stacks; i++) 
	{
		GLfloat rho = (GLfloat)i * drho;
		GLfloat srho = (GLfloat)(sin(rho));
		GLfloat crho = (GLfloat)(cos(rho));
		GLfloat srhodrho = (GLfloat)(sin(rho + drho));
		GLfloat crhodrho = (GLfloat)(cos(rho + drho));

		// Many sources of OpenGL sphere drawing code uses a triangle fan
		// for the caps of the sphere. This however introduces texturing 
		// artifacts at the poles on some OpenGL implementations
		glBegin(GL_TRIANGLE_STRIP);
		s = 0.0f;
		for ( j = 0; j <= Slices; j++) 
		{
			GLfloat theta = (j == Slices) ? 0.0f : j * dtheta;
			GLfloat stheta = (GLfloat)(-sin(theta));
			GLfloat ctheta = (GLfloat)(cos(theta));

			GLfloat x = stheta * srho;
			GLfloat y = ctheta * srho;
			GLfloat z = crho;

			//glTexCoord2f(s, t);
			//glNormal3f(x, y, z);
			glVertex3f(x * Radius, y * Radius, z * Radius);

			x = stheta * srhodrho;
			y = ctheta * srhodrho;
			z = crhodrho;
			//glTexCoord2f(s, t - dt);
			s += ds;
			//glNormal3f(x, y, z);
			glVertex3f(x * Radius, y * Radius, z * Radius);
		}
		glEnd();

		t -= dt;
	}
}

// TODO:
// 임시로 한 것임.
// 화살표 그릴 것.
void srgDrawArrow1(float Length)
{
	glBegin(GL_LINES);
	{
		glVertex3f(0.0f, 0.0f, 0.0f);
		glVertex3f(Length, 0.0f, 0.0f);
	}
	glEnd();
}

// For best results, put this in a display list
// Draw a torus (dough nut)  at z = fZVal... torus is in xy plane
void srgDrawTorus(GLfloat majorRadius, GLfloat minorRadius, GLint numMajor, GLint numMinor)
{
	Vec3 vNormal;
	double majorStep = 2.0f*M_PI / numMajor;
	double minorStep = 2.0f*M_PI / numMinor;
	int i, j;

	for (i=0; i<numMajor; ++i) 
	{
		double a0 = i * majorStep;
		double a1 = a0 + majorStep;
		GLfloat x0 = (GLfloat) cos(a0);
		GLfloat y0 = (GLfloat) sin(a0);
		GLfloat x1 = (GLfloat) cos(a1);
		GLfloat y1 = (GLfloat) sin(a1);

		glBegin(GL_TRIANGLE_STRIP);
		for (j=0; j<=numMinor; ++j) 
		{
			double b = j * minorStep;
			GLfloat c = (GLfloat) cos(b);
			GLfloat r = minorRadius * c + majorRadius;
			GLfloat z = minorRadius * (GLfloat) sin(b);

			// First point
			glTexCoord2f((float)(i)/(float)(numMajor), (float)(j)/(float)(numMinor));
			vNormal[0] = x0*c;
			vNormal[1] = y0*c;
			vNormal[2] = z/minorRadius;
			vNormal = Normalize(vNormal);
			glNormal3d(vNormal[0], vNormal[1], vNormal[2]);
			glVertex3f(x0*r, y0*r, z);

			glTexCoord2f((float)(i+1)/(float)(numMajor), (float)(j)/(float)(numMinor));
			vNormal[0] = x1*c;
			vNormal[1] = y1*c;
			vNormal[2] = z/minorRadius;
			//vNormal = Normalize(vNormal);
			glNormal3d(vNormal[0], vNormal[1], vNormal[2]);
			glVertex3f(x1*r, y1*r, z);
		}
		glEnd();
	}
}

//////////////////////////////////////////////////////////////////////////
// TODO: 하세요 -_-
//////////////////////////////////////////////////////////////////////////













//////////////////////////////////////////////////////////////////////
//
// Implementation of srgQuadric class.
//
/*** DESCRIPTION

This is actually a helper class which wraps the
use of quadric objects in OGL (see guide).
It must be used inside an GLEnabledView cause
a quadric object must refer to a Rendering Context.

****************************************/

//////////////////////////////////////////////////////////////////////
// Construction/Destruction

//srgQuadric::srgQuadric(GLenum drwStyle,GLenum normals,GLenum side,BOOL bGenerateTxtrCoords)
//{
//	// check validity of parameters
//	dAASSERT(normals==GLU_NONE || normals==GLU_FLAT || normals==GLU_SMOOTH);
//	dAASSERT(drwStyle==GLU_FILL || drwStyle==GLU_LINE || drwStyle==GLU_SILHOUETTE|| drwStyle==GLU_POINT);
//	dAASSERT(side==GLU_INSIDE || side==GLU_OUTSIDE);
//	// create quadric object
//	m_quadrObj = gluNewQuadric();
//	// set error callback function (shared with tesselators)
//	//gluQuadricCallback(m_quadrObj,GLU_ERROR,(void (CALLBACK*)())&ErrorCallback);
//	// set normal generation
//	gluQuadricNormals(m_quadrObj,normals);
//	// set Texture Coordinates generation
//	if(bGenerateTxtrCoords) gluQuadricTexture(m_quadrObj,GL_TRUE);
//	else gluQuadricTexture(m_quadrObj,GL_FALSE);
//	// set how the quadric will be generated
//	gluQuadricDrawStyle(m_quadrObj,drwStyle);
//	// set which side of the quadric is to be considered inside
//	gluQuadricOrientation(m_quadrObj,side);
//}
//
//srgQuadric::~srgQuadric()
//{
//	// remove quadric object
//	gluDeleteQuadric(m_quadrObj);	
//}
//
////////////////////////////////////////////////////////////////////////
//// Member functions
//
//void srgQuadric::SetNormals(GLenum type)
//{
//	// check validity of type parameter it must be one of these:
//	dAASSERT(type==GLU_NONE || type==GLU_FLAT || type==GLU_SMOOTH);
//	// issue corresponding GL command
//	gluQuadricNormals(m_quadrObj,type);
//}
//
//void srgQuadric::SetTextureCoordsGen(BOOL flag)
//{
//	// issue corresponding GL commands
//	if(flag) gluQuadricTexture(m_quadrObj,GL_TRUE);
//	else gluQuadricTexture(m_quadrObj,GL_FALSE);
//}
//
//void srgQuadric::SetOrientation(GLenum type)
//{
//	// check validity of type parameter it must be one of these:
//	dAASSERT(type==GLU_INSIDE || type==GLU_OUTSIDE);
//	// issue corresponding GL command
//	gluQuadricOrientation(m_quadrObj,type);
//}
//
//void srgQuadric::SetDrawStyle(GLenum style)
//{
//	// check validity of type parameter it must be one of these:
//	dAASSERT(style==GLU_FILL || style==GLU_LINE || style==GLU_SILHOUETTE|| style==GLU_POINT);
//	// issue corresponding GL command
//	gluQuadricDrawStyle(m_quadrObj,style);
//}
//
//void srgQuadric::DrawSphere(GLdouble radius, int longitudeSubdiv, int latitudeSubdiv)
//{
//	// issue corresponding GL command
//	gluSphere(m_quadrObj,radius,longitudeSubdiv,latitudeSubdiv);
//}
//
//void srgQuadric::DrawCylinder(GLdouble baseRadius,GLdouble topRadius,GLdouble height,int slices,int stacks)
//{
//	// issue corresponding GL command
//	gluCylinder(m_quadrObj,baseRadius,topRadius,height,slices,stacks);
//}
//
//void srgQuadric::DrawDisk(GLdouble innerRadius,GLdouble outerRadius,int slices,int loops)
//{
//	// issue corresponding GL command
//	gluDisk(m_quadrObj,innerRadius,outerRadius,slices,loops);
//}
//
//void srgQuadric::DrawPartialDisk(GLdouble innerRadius,GLdouble outerRadius,int slices,int loops,GLdouble startAngle,GLdouble sweepAngle)
//{
//	// issue corresponding GL command
//	gluPartialDisk(m_quadrObj,innerRadius,outerRadius,slices,loops,startAngle,sweepAngle);
//}
















































/*******************************************************************************
CAUTION
struct GLUquadric {}; -> For debug
don't remove this code
*******************************************************************************/
//struct GLUquadric {};
/******************************************************************************/

void ConvertXrQuadricToOpenglQuadric(DWORD dwFlags, GLUquadricObj* obj)
{
	DWORD DrawingStyleTestFlags = SRG_FIGURE_DRAWINGSTYLE_MASK;
	DWORD TextureTestFlags = SRG_FIGURE_TEXTURE_MASK;

	// Drawing style을 검출한다
	DrawingStyleTestFlags &= dwFlags;
	TextureTestFlags &= dwFlags;

	switch(DrawingStyleTestFlags)
	{
	case SRG_FIGURE_FILL:
		gluQuadricDrawStyle(obj, GLU_FILL);
		break;
	case SRG_FIGURE_WIREFRAME:
		gluQuadricDrawStyle(obj, GLU_LINE);
		break;
	case SRG_FIGURE_FEATURELINE:
		gluQuadricDrawStyle(obj, GLU_SILHOUETTE);
		break;
	case SRG_FIGURE_SILHOUETTE:
		gluQuadricDrawStyle(obj, GLU_SILHOUETTE);
		break;
	}

	switch(TextureTestFlags)
	{
	case SRG_FIGURE_TEXTURE:
		gluQuadricTexture(obj, GLU_TRUE);
		break;
	case 0x00000000:
		gluQuadricTexture(obj, GLU_FALSE);
		break;
	}

	gluQuadricOrientation(obj, GLU_OUTSIDE);
}

void ConvertXrQuadricToOpenglQuadric(QuadricState eQuadricState, GLUquadricObj* obj)
{
	switch(eQuadricState.DrawStyle)
	{
	case drawstyleFILL:
		gluQuadricDrawStyle(obj, GLU_FILL);
		break;
	case drawstyleLINE:
		gluQuadricDrawStyle(obj, GLU_LINE);
		break;
	case drawstyleSILHOUETTE:
		gluQuadricDrawStyle(obj, GLU_SILHOUETTE);
		break;
  default:
    break;
	}

	if(eQuadricState.HasTexture){
		gluQuadricTexture(obj, GLU_TRUE);
  } else{
		gluQuadricTexture(obj, GLU_FALSE);
  }
	gluQuadricOrientation(obj, GLU_OUTSIDE);
}

void srgDrawLine(float fX1, float fY1, float fZ1, float fX2, float fY2, float fZ2)
{
	glBegin(GL_LINES);
	{
		glVertex3f(fX1, fY1, fZ1);
		glVertex3f(fX2, fY2, fZ2);
	}
	glEnd();
}

//void srgDrawFrustumLine(float fFovy, float fAspect, float fzNear, float fzFar)
//{
//	float fNearTop = fzNear*tanf((float)srgL_DEG2RAD(fFovy)*0.5f);
//	float fFarTop = (fzFar*float(0.01f))*tanf((float)srgL_DEG2RAD(fFovy)*0.5f);
//	float fNearBottom = -fNearTop;
//	float fFarBottom = -fFarTop;
//
//	float fNearRight = fAspect*fNearTop;
//	float fFarRight = fAspect*fFarTop;
//	float fNearLeft = -fNearRight;
//	float fFarLeft = -fFarRight;
//
//	float fzNearView = -fzNear;
//	//float fzFarView = -fzFar;
//	float fzFarView = -fzFar*float(0.01f);
//
//	glBegin(GL_LINE_STRIP);
//	{
//		glVertex3f(fNearTop, fNearRight, fzNearView);
//		glVertex3f(fNearBottom, fNearRight, fzNearView);
//		glVertex3f(fNearBottom, fNearLeft, fzNearView);
//		glVertex3f(fNearTop, fNearLeft, fzNearView);
//		glVertex3f(fNearTop, fNearRight, fzNearView);
//	}
//	glEnd();
//	/*
//	glBegin(GL_POLYGON);
//	{
//	glVertex3f(fFarTop, fFarRight, fzFarView);
//	glVertex3f(fFarBottom, fFarRight, fzFarView);
//	glVertex3f(fFarBottom, fFarLeft, fzFarView);
//	glVertex3f(fFarTop, fFarLeft, fzFarView);
//	//glVertex3f(fFarTop, fFarRight, fzFarView);
//	}
//	glEnd();
//	*/
//	glBegin(GL_LINES);
//	{
//		glVertex3f(fNearTop, fNearRight, fzNearView);
//		glVertex3f(fFarTop, fFarRight, fzFarView);
//
//		glVertex3f(fNearBottom, fNearRight, fzNearView);
//		glVertex3f(fFarBottom, fFarRight, fzFarView);
//
//		glVertex3f(fNearBottom, fNearLeft, fzNearView);
//		glVertex3f(fFarBottom, fFarLeft, fzFarView);
//
//		glVertex3f(fNearTop, fNearLeft, fzNearView);
//		glVertex3f(fFarTop, fFarLeft, fzFarView);
//	}
//	glEnd();
//}

void srgDrawBox(QuadricState eQuadricState,
			   double X, double Y, double Z)
{
	switch(eQuadricState.DrawStyle)
	{
	case drawstyleFILL:
		if (eQuadricState.HasTexture)
			srgDrawBox_NV_TEX(X, Y, Z);
		else
			srgDrawBox_NV(X, Y, Z);
		break;
	case drawstyleLINE:
		glPushAttrib(GL_POLYGON_BIT);
		{
			glPolygonMode (GL_FRONT_AND_BACK, GL_LINE);

			if (eQuadricState.HasTexture)
				srgDrawBox_NV_TEX(X, Y, Z);
			else
				srgDrawBox_NV(X, Y, Z);
		}
		glPopAttrib();
		break;
	case drawstyleSILHOUETTE:
		{
			glPushAttrib(GL_POLYGON_BIT);
			{
				glPolygonMode (GL_BACK, GL_LINE);		// Draw Back facing Polygons As Wire frames

				glEnable(GL_CULL_FACE);
				glCullFace (GL_FRONT);				// Don't Draw Any Front-Facing Polygons

				glPushAttrib(GL_DEPTH_BUFFER_BIT);
				{
					//glDepthFunc (GL_ALWAYS);			// Change The Y Mode
					srgDrawBox_FeatureLine(X, Y, Z);
					glDepthFunc (GL_LEQUAL);			// Change The Y Mode
					//if (eQuadricState.HasTexture)
					//{
					//	srgDrawBox_NV_TEX(X, Y, Z);
					//}
					//else
					//{
					//	srgDrawBox_NV(X, Y, Z);
					//}
				}
				glPopAttrib();
			}
			glPopAttrib();
		}
		break;
	case drawstyleEDGE:
		srgDrawBox_FeatureLine(X, Y, Z);
		break;
	case drawstyleBEHIND_EDGE:
		glPushAttrib(GL_LINE_BIT);
		{
			glEnable(GL_LINE_STIPPLE);
			glLineStipple(LINE_STIPPLE_FACTOR, LINE_STIPPLE_PATTER);

			glPushAttrib(GL_DEPTH_BUFFER_BIT);
			{
				glDepthFunc (GL_GREATER);			// Change The Y Mode
				srgDrawBox_FeatureLine(X, Y, Z);
			}
			glPopAttrib();
		}
		glPopAttrib();
		break;
	default:
		break;
	}
}


void srgDrawBox(DWORD dwFlags,
			   double X, double Y, double Z)
{
	DWORD DrawingStyleTestFlags = SRG_FIGURE_DRAWINGSTYLE_MASK;
	DWORD TextureTestFlags = SRG_FIGURE_TEXTURE_MASK;

	// Drawing style을 검출한다
	DrawingStyleTestFlags &= dwFlags;
	TextureTestFlags &= dwFlags;

	switch(DrawingStyleTestFlags)
	{
	case SRG_FIGURE_FILL:
		if (TextureTestFlags)
			srgDrawBox_NV_TEX(X, Y, Z);
		else
			srgDrawBox_NV(X, Y, Z);
		break;
	case SRG_FIGURE_WIREFRAME:
		srgDrawBox_FeatureLine(X, Y, Z);
		// 		glPushAttrib(GL_POLYGON_BIT);
		// 		{
		// 			glPolygonMode (GL_FRONT_AND_BACK, GL_LINE);
		// 
		// 			if (TextureTestFlags)
		// 				srgDrawBox_NV_TEX(X, Y, Z);
		// 			else
		// 				srgDrawBox_NV(X, Y, Z);
		// 		}
		// 		glPopAttrib();
		break;
	case SRG_FIGURE_SILHOUETTE:
		{
			glPushAttrib(GL_POLYGON_BIT);
			{
				glPolygonMode (GL_BACK, GL_LINE);		// Draw Back facing Polygons As Wire frames

				glEnable(GL_CULL_FACE);
				glCullFace (GL_FRONT);				// Don't Draw Any Front-Facing Polygons

				glPushAttrib(GL_DEPTH_BUFFER_BIT);
				{
					//glDepthFunc (GL_ALWAYS);			// Change The Y Mode
					srgDrawBox_FeatureLine(X, Y, Z);
					glDepthFunc (GL_LEQUAL);			// Change The Y Mode
					//if (eQuadricState.HasTexture)
					//{
					//	srgDrawBox_NV_TEX(X, Y, Z);
					//}
					//else
					//{
					//	srgDrawBox_NV(X, Y, Z);
					//}
				}
				glPopAttrib();
			}
			glPopAttrib();
		}
		break;
	case SRG_FIGURE_FEATURELINE:
		srgDrawBox_FeatureLine(X, Y, Z);
		break;
	case SRG_FIGURE_BACK_FEATURELINE:
		glPushAttrib(GL_LINE_BIT);
		{
			glEnable(GL_LINE_STIPPLE);
			glLineStipple(LINE_STIPPLE_FACTOR, LINE_STIPPLE_PATTER);

			glPushAttrib(GL_DEPTH_BUFFER_BIT);
			{
				glDepthFunc (GL_GREATER);			// Change The Y Mode
				srgDrawBox_FeatureLine(X, Y, Z);
			}
			glPopAttrib();
		}
		glPopAttrib();
		break;
	default:
		break;
	}
}

void srgDrawDisk(double InnerRadius, double OuterRadius, int Slices, int Loops, DWORD dwFlag)
{
	GLUquadricObj *obj;
	obj = gluNewQuadric();

	ConvertXrQuadricToOpenglQuadric(dwFlag, obj);

	gluDisk(obj, InnerRadius, OuterRadius, Slices, Loops);

	gluDeleteQuadric(obj);
}
void srgDrawDisk(QuadricState	eQuadricState,
				double InnerRadius, double OuterRadius, int Slices, int Loops)
{
	GLUquadricObj *obj;
	obj = gluNewQuadric();

	ConvertXrQuadricToOpenglQuadric(eQuadricState, obj);

	gluDisk(obj, InnerRadius, OuterRadius, Slices, Loops);

	gluDeleteQuadric(obj);
}

void srgPartialDrawDisk(QuadricState	eQuadricState,
						double InnerRadius, double OuterRadius, int Slices, int Loops, double dStartAngle, double dSweepAngle)
{
	GLUquadricObj *obj;
	obj = gluNewQuadric();

	ConvertXrQuadricToOpenglQuadric(eQuadricState, obj);

	gluPartialDisk(obj, InnerRadius, OuterRadius, Slices, Loops, dStartAngle, dSweepAngle);

	gluDeleteQuadric(obj);
}

void srgDrawSphere(QuadricState	eQuadricState,
				  double Radius, int Slices, int Stacks)
{
	GLUquadricObj *obj;
	obj = gluNewQuadric();

	ConvertXrQuadricToOpenglQuadric(eQuadricState, obj);

	switch(eQuadricState.DrawStyle)
	{
	case drawstyleFILL:
		gluSphere(obj, Radius, Slices, Stacks);
		break;
	case drawstyleLINE:
		gluSphere(obj, Radius, Slices, Stacks);
		break;
	case drawstyleSILHOUETTE:
		glPushAttrib(GL_POLYGON_BIT);
		{
			glPolygonMode (GL_BACK, GL_LINE);		// Draw Back facing Polygons As Wire frames
			glEnable(GL_CULL_FACE);
			glCullFace (GL_FRONT);				// Don't Draw Any Front-Facing Polygons
			glPushAttrib(GL_DEPTH_BUFFER_BIT);
			{
				//glDepthFunc (GL_ALWAYS);			// Change The Depth Mode
				srgDrawSphere_FeatureLine(Radius, 2, 1, Slices, Stacks);
				glDepthFunc (GL_LEQUAL);			// Change The Depth Mode
				gluQuadricDrawStyle(obj, GLU_FILL);
				//gluSphere(obj, Radius, Slices, Stacks);
			}
			glPopAttrib();
		}
		glPopAttrib();
		break;
	case drawstyleEDGE:
		srgDrawSphere_FeatureLine(Radius, 4, 2, Slices, Stacks);
		break;
	case drawstyleBEHIND_EDGE:
		glPushAttrib(GL_LINE_BIT);
		{
			glEnable(GL_LINE_STIPPLE);
			glLineStipple(LINE_STIPPLE_FACTOR, LINE_STIPPLE_PATTER);

			glPushAttrib(GL_DEPTH_BUFFER_BIT);
			{
				glDepthFunc (GL_GREATER);			// Change The Y Mode
				srgDrawSphere_FeatureLine(Radius, 2, 1, Slices, Stacks);
			}
			glPopAttrib();
		}
		glPopAttrib();
		break;
	default:
		break;
	}

	gluDeleteQuadric(obj);
}

void srgDrawSphere(DWORD dwFlags,
				  double Radius, int Slices, int Stacks)
{
	GLUquadricObj *obj;
	obj = gluNewQuadric();

	ConvertXrQuadricToOpenglQuadric(dwFlags, obj);

	DWORD DrawingStyleTestFlags = SRG_FIGURE_DRAWINGSTYLE_MASK;

	// Drawing style을 검출한다
	DrawingStyleTestFlags &= dwFlags;

	switch(DrawingStyleTestFlags)
	{
	case SRG_FIGURE_FILL:
		gluSphere(obj, Radius, Slices, Stacks);
		break;
	case SRG_FIGURE_WIREFRAME:
		gluSphere(obj, Radius, Slices, Stacks);
		break;
	case SRG_FIGURE_SILHOUETTE:
		glPushAttrib(GL_POLYGON_BIT);
		{
			glPolygonMode (GL_BACK, GL_LINE);		// Draw Back facing Polygons As Wire frames
			glEnable(GL_CULL_FACE);
			glCullFace (GL_FRONT);				// Don't Draw Any Front-Facing Polygons
			glPushAttrib(GL_DEPTH_BUFFER_BIT);
			{
				//glDepthFunc (GL_ALWAYS);			// Change The Depth Mode
				srgDrawSphere_FeatureLine(Radius, 2, 1, Slices, Stacks);
				glDepthFunc (GL_LEQUAL);			// Change The Depth Mode
				gluQuadricDrawStyle(obj, GLU_FILL);
				//gluSphere(obj, Radius, Slices, Stacks);
			}
			glPopAttrib();
		}
		glPopAttrib();
		break;
	case SRG_FIGURE_FEATURELINE:
		srgDrawSphere_FeatureLine(Radius, 4, 2, Slices, Stacks);
		break;
	case SRG_FIGURE_BACK_FEATURELINE:
		glPushAttrib(GL_LINE_BIT);
		{
			glEnable(GL_LINE_STIPPLE);
			glLineStipple(LINE_STIPPLE_FACTOR, LINE_STIPPLE_PATTER);

			glPushAttrib(GL_DEPTH_BUFFER_BIT);
			{
				glDepthFunc (GL_GREATER);			// Change The Y Mode
				srgDrawSphere_FeatureLine(Radius, 2, 1, Slices, Stacks);
			}
			glPopAttrib();
		}
		glPopAttrib();
		break;
	default:
		break;
	}

	gluDeleteQuadric(obj);
}

void srgDrawOpenDome(QuadricState	eQuadricState,
					double Radius, int Slices, int Stacks)
{
	switch(eQuadricState.DrawStyle)
	{
	case drawstyleFILL:
		if (eQuadricState.HasTexture)
			srgDrawOpenDome_NV_TEX(Radius, Slices, Stacks);
		else
			srgDrawOpenDome_NV(Radius, Slices, Stacks);
		break;
	case drawstyleLINE:
		if (eQuadricState.HasTexture)
			srgDrawOpenDome_NV_TEX(Radius, Slices, Stacks);
		else
			srgDrawOpenDome_NV(Radius, Slices, Stacks);
		break;
	case drawstyleSILHOUETTE:
		glPushAttrib(GL_POLYGON_BIT);
		{
			glPolygonMode (GL_BACK, GL_LINE);		// Draw Back facing Polygons As Wire frames
			glEnable(GL_CULL_FACE);
			glCullFace (GL_FRONT);				// Don't Draw Any Front-Facing Polygons

			glPushAttrib(GL_DEPTH_BUFFER_BIT);
			{
				//glDepthFunc (GL_ALWAYS);			// Change The Depth Mode
				srgDrawOpenDome_FeatureLine(Radius, 2, 1, Slices, Stacks);
				glDepthFunc (GL_LEQUAL);			// Change The Depth Mode
				//if (eQuadricState.HasTexture)
				//srgDrawOpenDome_NV_TEX(Radius, Slices, Stacks);
				//else
				//srgDrawOpenDome_NV(Radius, Slices, Stacks);
			}
			glPopAttrib();
		}
		glPopAttrib();
		break;
	case drawstyleEDGE:
		srgDrawOpenDome_FeatureLine(Radius, 2, 1, Slices, Stacks);
		break;
	case drawstyleBEHIND_EDGE:
		glPushAttrib(GL_LINE_BIT);
		{
			glEnable(GL_LINE_STIPPLE);
			glLineStipple(LINE_STIPPLE_FACTOR, LINE_STIPPLE_PATTER);

			glPushAttrib(GL_DEPTH_BUFFER_BIT);
			{
				glDepthFunc (GL_GREATER);			// Change The Y Mode
				srgDrawOpenDome_FeatureLine(Radius, 2, 1, Slices, Stacks);
			}
			glPopAttrib();
		}
		glPopAttrib();
		break;
	default:
		break;
	}
}

void srgDrawOpenDome(DWORD dwFlags,
					double Radius, int Slices, int Stacks)
{
	DWORD DrawingStyleTestFlags = SRG_FIGURE_DRAWINGSTYLE_MASK;
	DWORD TextureTestFlags = SRG_FIGURE_TEXTURE_MASK;

	// Drawing style을 검출한다
	DrawingStyleTestFlags &= dwFlags;
	TextureTestFlags &= dwFlags;

	switch(DrawingStyleTestFlags)
	{
	case SRG_FIGURE_FILL:
		if (TextureTestFlags)
			srgDrawOpenDome_NV_TEX(Radius, Slices, Stacks);
		else
			srgDrawOpenDome_NV(Radius, Slices, Stacks);
		break;
	case SRG_FIGURE_WIREFRAME:
		if (TextureTestFlags)
			srgDrawOpenDome_NV_TEX(Radius, Slices, Stacks);
		else
			srgDrawOpenDome_NV(Radius, Slices, Stacks);
		break;
	case SRG_FIGURE_SILHOUETTE:
		glPushAttrib(GL_POLYGON_BIT);
		{
			glPolygonMode (GL_BACK, GL_LINE);		// Draw Back facing Polygons As Wire frames
			glEnable(GL_CULL_FACE);
			glCullFace (GL_FRONT);				// Don't Draw Any Front-Facing Polygons

			glPushAttrib(GL_DEPTH_BUFFER_BIT);
			{
				//glDepthFunc (GL_ALWAYS);			// Change The Depth Mode
				srgDrawOpenDome_FeatureLine(Radius, 2, 1, Slices, Stacks);
				glDepthFunc (GL_LEQUAL);			// Change The Depth Mode
				//if (eQuadricState.HasTexture)
				//srgDrawOpenDome_NV_TEX(Radius, Slices, Stacks);
				//else
				//srgDrawOpenDome_NV(Radius, Slices, Stacks);
			}
			glPopAttrib();
		}
		glPopAttrib();
		break;
	case SRG_FIGURE_FEATURELINE:
		srgDrawOpenDome_FeatureLine(Radius, 2, 1, Slices, Stacks);
		break;
	case SRG_FIGURE_BACK_FEATURELINE:
		glPushAttrib(GL_LINE_BIT);
		{
			glEnable(GL_LINE_STIPPLE);
			glLineStipple(LINE_STIPPLE_FACTOR, LINE_STIPPLE_PATTER);

			glPushAttrib(GL_DEPTH_BUFFER_BIT);
			{
				glDepthFunc (GL_GREATER);			// Change The Y Mode
				srgDrawOpenDome_FeatureLine(Radius, 2, 1, Slices, Stacks);
			}
			glPopAttrib();
		}
		glPopAttrib();
		break;
	default:
		break;
	}
}

void srgDrawDome(QuadricState	eQuadricState,
				double Radius, int Slices, int Stacks)
{
	GLUquadricObj *obj;
	obj = gluNewQuadric();

	ConvertXrQuadricToOpenglQuadric(eQuadricState, obj);	

	switch(eQuadricState.DrawStyle)
	{
	case drawstyleFILL:
		if (eQuadricState.HasTexture)
		{
			srgDrawOpenDome_NV_TEX(Radius, Slices, Stacks);
			gluQuadricOrientation(obj, GLU_INSIDE);
			gluDisk(obj, 0.0, Radius, Slices, Stacks);
		}
		else
		{
			srgDrawOpenDome_NV(Radius, Slices, Stacks);
			gluQuadricOrientation(obj, GLU_INSIDE);
			gluDisk(obj, 0.0, Radius, Slices, Stacks);
		}
		break;
	case drawstyleLINE:
		if (eQuadricState.HasTexture)
		{
			srgDrawOpenDome_NV_TEX(Radius, Slices, Stacks);
			gluQuadricOrientation(obj, GLU_INSIDE);
			gluDisk(obj, 0.0, Radius, Slices, Stacks);
		}
		else
		{
			srgDrawOpenDome_NV(Radius, Slices, Stacks);
			gluQuadricOrientation(obj, GLU_INSIDE);
			gluDisk(obj, 0.0, Radius, Slices, Stacks);
		}
		break;
	case drawstyleSILHOUETTE:
		glPushAttrib(GL_POLYGON_BIT);
		{
			glPolygonMode (GL_BACK, GL_LINE);		// Draw Back facing Polygons As Wire frames
			glEnable(GL_CULL_FACE);
			glCullFace (GL_FRONT);				// Don't Draw Any Front-Facing Polygons

			glPushAttrib(GL_DEPTH_BUFFER_BIT);
			{
				//glDepthFunc (GL_ALWAYS);			// Change The Depth Mode
				srgDrawOpenDome_FeatureLine(Radius, 2, 2, Slices, Stacks);
				glDepthFunc (GL_LEQUAL);			// Change The Depth Mode
				if (eQuadricState.HasTexture)
				{
					srgDrawOpenDome_NV_TEX(Radius, Slices, Stacks);
					gluQuadricOrientation(obj, GLU_INSIDE);
					gluQuadricDrawStyle(obj, GLU_FILL);
					//gluDisk(obj, 0.0, Radius, Slices, Stacks);
				}
				else
				{
					srgDrawOpenDome_NV(Radius, Slices, Stacks);
					gluQuadricOrientation(obj, GLU_INSIDE);
					gluQuadricDrawStyle(obj, GLU_FILL);
					//gluDisk(obj, 0.0, Radius, Slices, Stacks);
				}
			}
			glPopAttrib();
		}
		glPopAttrib();
		break;
	case drawstyleEDGE:
		// TODO: gluDisk를 이용하던가 Disk그리는 함수를 이용하는 것으로 변경할 것
		srgDrawOpenDome_FeatureLine(Radius, 2, 2, Slices, Stacks);
		break;
	case drawstyleBEHIND_EDGE:
		glPushAttrib(GL_LINE_BIT);
		{
			glEnable(GL_LINE_STIPPLE);
			glLineStipple(LINE_STIPPLE_FACTOR, LINE_STIPPLE_PATTER);

			glPushAttrib(GL_DEPTH_BUFFER_BIT);
			{
				glDepthFunc (GL_GREATER);			// Change The Y Mode
				srgDrawOpenDome_FeatureLine(Radius, 2, 2, Slices, Stacks);
			}
			glPopAttrib();
		}
		glPopAttrib();
		break;
	default:
		break;
	}
	gluDeleteQuadric(obj);
}

void srgDrawDome(DWORD dwFlags,
				double Radius, int Slices, int Stacks)
{
	DWORD DrawingStyleTestFlags = SRG_FIGURE_DRAWINGSTYLE_MASK;
	DWORD TextureTestFlags = SRG_FIGURE_TEXTURE_MASK;

	// Drawing style을 검출한다
	DrawingStyleTestFlags &= dwFlags;
	TextureTestFlags &= dwFlags;

	GLUquadricObj *obj;
	obj = gluNewQuadric();

	ConvertXrQuadricToOpenglQuadric(dwFlags, obj);	

	switch(DrawingStyleTestFlags)
	{
	case drawstyleFILL:
		if (TextureTestFlags)
		{
			srgDrawOpenDome_NV_TEX(Radius, Slices, Stacks);
			gluQuadricOrientation(obj, GLU_INSIDE);
			gluDisk(obj, 0.0, Radius, Slices, Stacks);
		}
		else
		{
			srgDrawOpenDome_NV(Radius, Slices, Stacks);
			gluQuadricOrientation(obj, GLU_INSIDE);
			gluDisk(obj, 0.0, Radius, Slices, Stacks);
		}
		break;
	case drawstyleLINE:
		if (TextureTestFlags)
		{
			srgDrawOpenDome_NV_TEX(Radius, Slices, Stacks);
			gluQuadricOrientation(obj, GLU_INSIDE);
			gluDisk(obj, 0.0, Radius, Slices, Stacks);
		}
		else
		{
			srgDrawOpenDome_NV(Radius, Slices, Stacks);
			gluQuadricOrientation(obj, GLU_INSIDE);
			gluDisk(obj, 0.0, Radius, Slices, Stacks);
		}
		break;
	case drawstyleSILHOUETTE:
		glPushAttrib(GL_POLYGON_BIT);
		{
			glPolygonMode (GL_BACK, GL_LINE);		// Draw Back facing Polygons As Wire frames
			glEnable(GL_CULL_FACE);
			glCullFace (GL_FRONT);				// Don't Draw Any Front-Facing Polygons

			glPushAttrib(GL_DEPTH_BUFFER_BIT);
			{
				//glDepthFunc (GL_ALWAYS);			// Change The Depth Mode
				srgDrawOpenDome_FeatureLine(Radius, 2, 2, Slices, Stacks);
				glDepthFunc (GL_LEQUAL);			// Change The Depth Mode
				if (TextureTestFlags)
				{
					srgDrawOpenDome_NV_TEX(Radius, Slices, Stacks);
					gluQuadricOrientation(obj, GLU_INSIDE);
					gluQuadricDrawStyle(obj, GLU_FILL);
					//gluDisk(obj, 0.0, Radius, Slices, Stacks);
				}
				else
				{
					srgDrawOpenDome_NV(Radius, Slices, Stacks);
					gluQuadricOrientation(obj, GLU_INSIDE);
					gluQuadricDrawStyle(obj, GLU_FILL);
					//gluDisk(obj, 0.0, Radius, Slices, Stacks);
				}
			}
			glPopAttrib();
		}
		glPopAttrib();
		break;
	case drawstyleEDGE:
		// TODO: gluDisk를 이용하던가 Disk그리는 함수를 이용하는 것으로 변경할 것
		srgDrawOpenDome_FeatureLine(Radius, 2, 2, Slices, Stacks);
		break;
	case drawstyleBEHIND_EDGE:
		glPushAttrib(GL_LINE_BIT);
		{
			glEnable(GL_LINE_STIPPLE);
			glLineStipple(LINE_STIPPLE_FACTOR, LINE_STIPPLE_PATTER);

			glPushAttrib(GL_DEPTH_BUFFER_BIT);
			{
				glDepthFunc (GL_GREATER);			// Change The Y Mode
				srgDrawOpenDome_FeatureLine(Radius, 2, 2, Slices, Stacks);
			}
			glPopAttrib();
		}
		glPopAttrib();
		break;
	default:
		break;
	}
	gluDeleteQuadric(obj);
}

void srgDrawOpenCylinder(QuadricState	eQuadricState,
						double BaseRadius, double TopRadius, double Height, int Slices, int /* Stacks */)
{
	GLUquadricObj *obj;
	obj = gluNewQuadric();

	ConvertXrQuadricToOpenglQuadric(eQuadricState, obj);

	switch(eQuadricState.DrawStyle)
	{
	case drawstyleFILL:
		// Check: Stack을 1로 고정
		glPushMatrix();
		{
			glTranslatef(0.0f, 0.0f, -(GLfloat)Height/2.0f);
			gluCylinder(obj, (GLfloat)BaseRadius, (GLfloat)TopRadius, (GLfloat)Height, Slices, 1);
		}
		glPopMatrix();

		break;
	case drawstyleLINE:
		// Check: Stack을 1로 고정
		glPushMatrix();
		{
			glTranslatef(0.0f, 0.0f, -(GLfloat)Height/2.0f);
			gluCylinder(obj, (GLfloat)BaseRadius, (GLfloat)TopRadius, (GLfloat)Height, Slices, 1);
		}
		glPopMatrix();
		break;
	case drawstyleSILHOUETTE:
		glPushAttrib(GL_POLYGON_BIT);
		{
			glPolygonMode (GL_BACK, GL_LINE);		// Draw Back facing Polygons As Wire frames
			glEnable(GL_CULL_FACE);
			glCullFace (GL_FRONT);				// Don't Draw Any Front-Facing Polygons

			glPushAttrib(GL_DEPTH_BUFFER_BIT);
			{
				//glDepthFunc (GL_ALWAYS);			// Change The Depth Mode
				srgDrawOpenCylinder_FeatureLine(BaseRadius, TopRadius, Height, 2, Slices);
				glDepthFunc (GL_LEQUAL);			// Change The Depth Mode
				// Check: Stack을 1로 고정
				glPushMatrix();
				{
					glTranslatef(0.0f, 0.0f, -(GLfloat)Height/2.0f);
					gluQuadricDrawStyle(obj, GLU_FILL);
					//gluCylinder(obj, BaseRadius, TopRadius, Height, Slices, 1);
				}
				glPopMatrix();
			}
			glPopAttrib();
		}
		glPopAttrib();
		break;
	case drawstyleEDGE:
		// TODO: gluDisk를 이용하던가 Disk그리는 함수를 이용하는 것으로 변경할 것
		srgDrawOpenCylinder_FeatureLine(BaseRadius, TopRadius, Height, 2, Slices);
		break;
	case drawstyleBEHIND_EDGE:
		glPushAttrib(GL_LINE_BIT);
		{
			glEnable(GL_LINE_STIPPLE);
			glLineStipple(LINE_STIPPLE_FACTOR, LINE_STIPPLE_PATTER);

			glPushAttrib(GL_DEPTH_BUFFER_BIT);
			{
				glDepthFunc (GL_GREATER);			// Change The Y Mode
				srgDrawOpenCylinder_FeatureLine(BaseRadius, TopRadius, Height, 2, Slices);
			}
			glPopAttrib();
		}
		glPopAttrib();
		break;
	default:
		break;
	}

	gluDeleteQuadric(obj);
}

void srgDrawOpenCylinder(DWORD dwFlags,
						double BaseRadius, double TopRadius, double Height, int Slices, int /* Stacks */)
{
	DWORD DrawingStyleTestFlags = SRG_FIGURE_DRAWINGSTYLE_MASK;
	DWORD TextureTestFlags = SRG_FIGURE_TEXTURE_MASK;

	// Drawing style을 검출한다
	DrawingStyleTestFlags &= dwFlags;
	TextureTestFlags &= dwFlags;

	GLUquadricObj *obj;
	obj = gluNewQuadric();

	ConvertXrQuadricToOpenglQuadric(dwFlags, obj);

	switch(DrawingStyleTestFlags)
	{
	case SRG_FIGURE_FILL:
		// Check: Stack을 1로 고정
		glPushMatrix();
		{
			glTranslatef(0.0f, 0.0f, -(GLfloat)Height/2.0f);
			gluCylinder(obj, (GLfloat)BaseRadius, (GLfloat)TopRadius, (GLfloat)Height, Slices, 1);
		}
		glPopMatrix();

		break;
	case SRG_FIGURE_WIREFRAME:
		// Check: Stack을 1로 고정
		glPushMatrix();
		{
			glTranslatef(0.0f, 0.0f, -(GLfloat)Height/2.0f);
			gluCylinder(obj, (GLfloat)BaseRadius, (GLfloat)TopRadius, (GLfloat)Height, Slices, 1);
		}
		glPopMatrix();
		break;
	case SRG_FIGURE_SILHOUETTE:
		glPushAttrib(GL_POLYGON_BIT);
		{
			glPolygonMode (GL_BACK, GL_LINE);		// Draw Back facing Polygons As Wire frames
			glEnable(GL_CULL_FACE);
			glCullFace (GL_FRONT);				// Don't Draw Any Front-Facing Polygons

			glPushAttrib(GL_DEPTH_BUFFER_BIT);
			{
				//glDepthFunc (GL_ALWAYS);			// Change The Depth Mode
				srgDrawOpenCylinder_FeatureLine(BaseRadius, TopRadius, Height, 2, Slices);
				glDepthFunc (GL_LEQUAL);			// Change The Depth Mode
				// Check: Stack을 1로 고정
				glPushMatrix();
				{
					glTranslatef(0.0f, 0.0f, -(GLfloat)Height/2.0f);
					gluQuadricDrawStyle(obj, GLU_FILL);
					//gluCylinder(obj, BaseRadius, TopRadius, Height, Slices, 1);
				}
				glPopMatrix();
			}
			glPopAttrib();
		}
		glPopAttrib();
		break;
	case SRG_FIGURE_FEATURELINE:
		// TODO: gluDisk를 이용하던가 Disk그리는 함수를 이용하는 것으로 변경할 것
		srgDrawOpenCylinder_FeatureLine(BaseRadius, TopRadius, Height, 2, Slices);
		break;
	case SRG_FIGURE_BACK_FEATURELINE:
		glPushAttrib(GL_LINE_BIT);
		{
			glEnable(GL_LINE_STIPPLE);
			glLineStipple(LINE_STIPPLE_FACTOR, LINE_STIPPLE_PATTER);

			glPushAttrib(GL_DEPTH_BUFFER_BIT);
			{
				glDepthFunc (GL_GREATER);			// Change The Y Mode
				srgDrawOpenCylinder_FeatureLine(BaseRadius, TopRadius, Height, 2, Slices);
			}
			glPopAttrib();
		}
		glPopAttrib();
		break;
	default:
		break;
	}

	gluDeleteQuadric(obj);
}

void srgDrawCylinder(QuadricState	eQuadricState,
					double BaseRadius, double TopRadius, double Height, int Slices, int /* Stacks */)
{
	GLUquadricObj *obj;
	obj = gluNewQuadric();

	ConvertXrQuadricToOpenglQuadric(eQuadricState, obj);

	switch(eQuadricState.DrawStyle)
	{
	case drawstyleFILL:
		glPushMatrix();
		{
			glTranslated(0.0, 0.0, -0.5 * Height);
			//if (eQuadricState.eNormal & orientationOUTSIDE)
			gluQuadricOrientation(obj, GLU_INSIDE);
			// Check: Stack을 1로 고정
			gluDisk(obj, 0.0, BaseRadius, Slices, 1);

			//if (eQuadricState.eNormal & orientationINSIDE)
			//gluQuadricOrientation(obj, GLU_OUTSIDE);
			// Check: Stack을 1로 고정
			gluQuadricOrientation(obj, GLU_OUTSIDE);
			gluCylinder(obj, BaseRadius, TopRadius, Height, Slices, 1);

			glTranslated(0.0, 0.0, Height);
			// Check: Stack을 1로 고정
			gluQuadricOrientation(obj, GLU_OUTSIDE);
			gluDisk(obj, 0.0, TopRadius, Slices, 1);
		}
		glPopMatrix();
		break;
	case drawstyleLINE:
		glPushMatrix();
		{
			glTranslated(0.0, 0.0, -0.5 * Height);
			//if (eQuadricState.eNormal & orientationOUTSIDE)
			gluQuadricOrientation(obj, GLU_INSIDE);
			// Check: Stack을 1로 고정
			gluDisk(obj, 0.0, BaseRadius, Slices, 1);

			//if (eQuadricState.eNormal & orientationINSIDE)
			//gluQuadricOrientation(obj, GLU_OUTSIDE);
			// Check: Stack을 1로 고정
			gluQuadricOrientation(obj, GLU_OUTSIDE);
			gluCylinder(obj, BaseRadius, TopRadius, Height, Slices, 1);

			glTranslated(0.0, 0.0, Height);
			// Check: Stack을 1로 고정
			gluQuadricOrientation(obj, GLU_OUTSIDE);
			gluDisk(obj, 0.0, TopRadius, Slices, 1);
		}
		glPopMatrix();
		break;
	case drawstyleSILHOUETTE:

		glPushAttrib(GL_POLYGON_BIT);
		{
			glPolygonMode (GL_BACK, GL_LINE);		// Draw Back facing Polygons As Wire frames
			glEnable(GL_CULL_FACE);
			glCullFace (GL_FRONT);				// Don't Draw Any Front-Facing Polygons

			glPushAttrib(GL_DEPTH_BUFFER_BIT);
			{
				//glDepthFunc (GL_ALWAYS);			// Change The Depth Mode
				srgDrawOpenCylinder_FeatureLine(BaseRadius, TopRadius, Height, 2, Slices);
				glDepthFunc (GL_LEQUAL);			// Change The Depth Mode

				glPushMatrix();
				{
					glTranslated(0.0, 0.0, -0.5 * Height);
					//if (eQuadricState.eNormal & orientationOUTSIDE)
					gluQuadricDrawStyle(obj, GLU_FILL);
					gluQuadricOrientation(obj, GLU_INSIDE);
					// Check: Stack을 1로 고정
					//gluDisk(obj, 0.0, BaseRadius, Slices, 1);

					//if (eQuadricState.eNormal & orientationINSIDE)
					//gluQuadricOrientation(obj, GLU_OUTSIDE);
					// Check: Stack을 1로 고정
					gluQuadricDrawStyle(obj, GLU_FILL);
					gluQuadricOrientation(obj, GLU_OUTSIDE);
					//gluCylinder(obj, BaseRadius, TopRadius, Height, Slices, 1);

					glTranslated(0.0, 0.0, Height);
					// Check: Stack을 1로 고정
					gluQuadricDrawStyle(obj, GLU_FILL);
					gluQuadricOrientation(obj, GLU_OUTSIDE);
					//gluDisk(obj, 0.0, TopRadius, Slices, 1);
				}
				glPopMatrix();
			}
			glPopAttrib();
		}
		glPopAttrib();
		break;
	case drawstyleEDGE:
		// TODO: gluDisk를 이용하던가 Disk그리는 함수를 이용하는 것으로 변경할 것
		srgDrawOpenCylinder_FeatureLine(BaseRadius, TopRadius, Height, 2, Slices);
		break;
	case drawstyleBEHIND_EDGE:
		glPushAttrib(GL_LINE_BIT);
		{
			glEnable(GL_LINE_STIPPLE);
			glLineStipple(LINE_STIPPLE_FACTOR, LINE_STIPPLE_PATTER);

			glPushAttrib(GL_DEPTH_BUFFER_BIT);
			{
				glDepthFunc (GL_GREATER);			// Change The Y Mode
				srgDrawOpenCylinder_FeatureLine(BaseRadius, TopRadius, Height, 2, Slices);
			}
			glPopAttrib();
		}
		glPopAttrib();
		break;
	default:
		break;
	}

	gluDeleteQuadric(obj);
}

void srgDrawCylinder(DWORD dwFlags,
					double BaseRadius, double TopRadius, double Height, int Slices, int /* Stacks */)
{
	GLUquadricObj *obj;
	obj = gluNewQuadric();

	ConvertXrQuadricToOpenglQuadric(dwFlags, obj);

	DWORD DrawingStyleTestFlags = SRG_FIGURE_DRAWINGSTYLE_MASK;

	// Drawing style을 검출한다
	DrawingStyleTestFlags &= dwFlags;

	switch(DrawingStyleTestFlags)
	{
	case SRG_FIGURE_FILL:
		glPushMatrix();
		{
			glTranslated(0.0, 0.0, -0.5 * Height);
			//if (eQuadricState.eNormal & orientationOUTSIDE)
			gluQuadricOrientation(obj, GLU_INSIDE);
			// Check: Stack을 1로 고정
			gluDisk(obj, 0.0, BaseRadius, Slices, 1);

			//if (eQuadricState.eNormal & orientationINSIDE)
			//gluQuadricOrientation(obj, GLU_OUTSIDE);
			// Check: Stack을 1로 고정
			gluQuadricOrientation(obj, GLU_OUTSIDE);
			gluCylinder(obj, BaseRadius, TopRadius, Height, Slices, 1);

			glTranslated(0.0, 0.0, Height);
			// Check: Stack을 1로 고정
			gluQuadricOrientation(obj, GLU_OUTSIDE);
			gluDisk(obj, 0.0, TopRadius, Slices, 1);
		}
		glPopMatrix();
		break;
	case SRG_FIGURE_WIREFRAME:
		glPushMatrix();
		{
			glTranslated(0.0, 0.0, -0.5 * Height);
			//if (eQuadricState.eNormal & orientationOUTSIDE)
			gluQuadricOrientation(obj, GLU_INSIDE);
			// Check: Stack을 1로 고정
			gluDisk(obj, 0.0, BaseRadius, Slices, 1);

			//if (eQuadricState.eNormal & orientationINSIDE)
			//gluQuadricOrientation(obj, GLU_OUTSIDE);
			// Check: Stack을 1로 고정
			gluQuadricOrientation(obj, GLU_OUTSIDE);
			gluCylinder(obj, BaseRadius, TopRadius, Height, Slices, 1);

			glTranslated(0.0, 0.0, Height);
			// Check: Stack을 1로 고정
			gluQuadricOrientation(obj, GLU_OUTSIDE);
			gluDisk(obj, 0.0, TopRadius, Slices, 1);
		}
		glPopMatrix();
		break;
	case SRG_FIGURE_SILHOUETTE:
		glPushAttrib(GL_POLYGON_BIT);
		{
			glPolygonMode (GL_BACK, GL_LINE);		// Draw Back facing Polygons As Wire frames
			glEnable(GL_CULL_FACE);
			glCullFace (GL_FRONT);				// Don't Draw Any Front-Facing Polygons

			glPushAttrib(GL_DEPTH_BUFFER_BIT);
			{
				//glDepthFunc (GL_ALWAYS);			// Change The Depth Mode
				srgDrawOpenCylinder_FeatureLine(BaseRadius, TopRadius, Height, 2, Slices);
				glDepthFunc (GL_LEQUAL);			// Change The Depth Mode

				glPushMatrix();
				{
					glTranslated(0.0, 0.0, -0.5 * Height);
					//if (eQuadricState.eNormal & orientationOUTSIDE)
					gluQuadricDrawStyle(obj, GLU_FILL);
					gluQuadricOrientation(obj, GLU_INSIDE);
					// Check: Stack을 1로 고정
					//gluDisk(obj, 0.0, BaseRadius, Slices, 1);

					//if (eQuadricState.eNormal & orientationINSIDE)
					//gluQuadricOrientation(obj, GLU_OUTSIDE);
					// Check: Stack을 1로 고정
					gluQuadricDrawStyle(obj, GLU_FILL);
					gluQuadricOrientation(obj, GLU_OUTSIDE);
					//gluCylinder(obj, BaseRadius, TopRadius, Height, Slices, 1);

					glTranslated(0.0, 0.0, Height);
					// Check: Stack을 1로 고정
					gluQuadricDrawStyle(obj, GLU_FILL);
					gluQuadricOrientation(obj, GLU_OUTSIDE);
					//gluDisk(obj, 0.0, TopRadius, Slices, 1);
				}
				glPopMatrix();
			}
			glPopAttrib();
		}
		glPopAttrib();
		break;
	case SRG_FIGURE_FEATURELINE:
		// TODO: gluDisk를 이용하던가 Disk그리는 함수를 이용하는 것으로 변경할 것
		srgDrawOpenCylinder_FeatureLine(BaseRadius, TopRadius, Height, 2, Slices);
		break;
	case SRG_FIGURE_BACK_FEATURELINE:
		glPushAttrib(GL_LINE_BIT);
		{
			glEnable(GL_LINE_STIPPLE);
			glLineStipple(LINE_STIPPLE_FACTOR, LINE_STIPPLE_PATTER);

			glPushAttrib(GL_DEPTH_BUFFER_BIT);
			{
				glDepthFunc (GL_GREATER);			// Change The Y Mode
				srgDrawOpenCylinder_FeatureLine(BaseRadius, TopRadius, Height, 2, Slices);
			}
			glPopAttrib();
		}
		glPopAttrib();
		break;
	default:
		break;
	}

	gluDeleteQuadric(obj);
}

void srgDrawCapsule(QuadricState	eQuadricState,
				   double BaseRadius, double TopRadius, double Height, int Slices, int Stacks)
{
	glPushMatrix();
	{
		//glTranslated(0.0, 0.0, -0.5 * Height);
		// Check: Stack을 1로 고정
		srgDrawOpenCylinder(eQuadricState, BaseRadius, TopRadius, Height, Slices, 1);

		glTranslated(0.0, 0.0, 0.5 * Height);
		//glTranslated(0.0, 0.0, Height);
		srgDrawOpenDome(eQuadricState, TopRadius, Slices, Stacks);

		glTranslated(0.0, 0.0, -Height);
		glRotated(180, 0.0, 1.0, 0.0);
		srgDrawOpenDome(eQuadricState, BaseRadius, Slices, Stacks);
		//srgDrawOpenDome(eQuadricState, Radius, Slices, Stacks);
	}
	glPopMatrix();
}

void srgDrawCapsule(DWORD dwFlags,
				   double BaseRadius, double TopRadius, double Height, int Slices, int Stacks)
{
	glPushMatrix();
	{
		//glTranslated(0.0, 0.0, -0.5 * Height);
		// Check: Stack을 1로 고정
		srgDrawOpenCylinder(dwFlags, BaseRadius, TopRadius, Height, Slices, 1);

		glTranslated(0.0f, 0.0f, 0.5f * Height);
		//glTranslated(0.0, 0.0, Height);
		srgDrawOpenDome(dwFlags, TopRadius, Slices, Stacks);

		glTranslated(0.0f, 0.0f, -Height);
		glRotated(180.0f, 0.0f, 1.0f, 0.0f);
		srgDrawOpenDome(dwFlags, BaseRadius, Slices, Stacks);
		//srgDrawOpenDome(eQuadricState, Radius, Slices, Stacks);
	}
	glPopMatrix();
}

void srgDrawCapsule(QuadricState	eQuadricState,
				   double Radius, double Height, int Slices, int Stacks)
{
	glPushMatrix();
	{

		// Check: Stack을 1로 고정
		srgDrawOpenCylinder(eQuadricState, Radius, Radius, Height, Slices, 1);

		glTranslated(0.0, 0.0, 0.5 * Height);
		//glTranslated(0.0, 0.0, Height);
		srgDrawOpenDome(eQuadricState, Radius, Slices, Stacks);

		glTranslated(0.0, 0.0, -Height);
		glRotated(180, 0.0, 1.0, 0.0);
		srgDrawOpenDome(eQuadricState, Radius, Slices, Stacks);
		//srgDrawOpenDome(eQuadricState, Radius, Slices, Stacks);
	}
	glPopMatrix();
}

void srgDrawCapsule(DWORD dwFlags,
				   double Radius, double Height, int Slices, int Stacks)
{
	glPushMatrix();
	{
		// Check: Stack을 1로 고정
		srgDrawOpenCylinder(dwFlags, Radius, Radius, Height, Slices, 1);

		glTranslated(0.0f, 0.0f, 0.5f * Height);
		//glTranslated(0.0, 0.0, Height);
		srgDrawOpenDome(dwFlags, Radius, Slices, Stacks);

		glTranslated(0.0f, 0.0f, -Height);
		glRotated(180.0f, 0.0f, 1.0f, 0.0f);
		srgDrawOpenDome(dwFlags, Radius, Slices, Stacks);
		//srgDrawOpenDome(eQuadricState, Radius, Slices, Stacks);
	}
	glPopMatrix();
}

//void srgDrawVRML(DWORD dwFlags, const std::string& VRMLFileName)
//{
//	DWORD DrawingStyleTestFlags = SRG_FIGURE_DRAWINGSTYLE_MASK;
//	DWORD TextureTestFlags = SRG_FIGURE_TEXTURE_MASK;
//
//	// Drawing style을 검출한다
//	DrawingStyleTestFlags &= dwFlags;
//	TextureTestFlags &= dwFlags;
//
//	switch(DrawingStyleTestFlags)
//	{
//	case SRG_FIGURE_FILL:
//		if (TextureTestFlags)
//		{
//			//srgDrawVRML_NV_TEX(X, Z, Y);
//		}
//		else
//			srgDrawVRML_NV(VRMLFileName);
//		break;
//	case SRG_FIGURE_WIREFRAME:
//		glPushAttrib(GL_POLYGON_BIT);
//		{
//			glPolygonMode (GL_FRONT_AND_BACK, GL_LINE);
//
//			if (TextureTestFlags)
//			{
//			}	
//			else
//				srgDrawVRML_NV(VRMLFileName);
//		}
//		glPopAttrib();
//		break;
//	case SRG_FIGURE_SILHOUETTE:
//		{
//			glPushAttrib(GL_POLYGON_BIT);
//			{
//				glPolygonMode (GL_BACK, GL_LINE);		// Draw Back facing Polygons As Wire frames
//
//				glEnable(GL_CULL_FACE);
//				glCullFace (GL_FRONT);				// Don't Draw Any Front-Facing Polygons
//
//				glPushAttrib(GL_DEPTH_BUFFER_BIT);
//				{
//					//glDepthFunc (GL_ALWAYS);			// Change The Y Mode
//
//					// TODO:
//					// FeatureLine함수를 만들것!!
//					srgDrawVRML_NV(VRMLFileName);
//
//					glDepthFunc (GL_LEQUAL);			// Change The Y Mode
//					//if (eQuadricState.HasTexture)
//					//{
//					//	srgDrawBox_NV_TEX(X, Z, Y);
//					//}
//					//else
//					//{
//					//	srgDrawBox_NV(X, Z, Y);
//					//}
//				}
//				glPopAttrib();
//			}
//			glPopAttrib();
//		}
//		break;
//	case SRG_FIGURE_FEATURELINE:
//		// TODO:
//		// FeatureLine함수를 만들것
//		srgDrawVRML_NV(VRMLFileName);
//		break;
//	case SRG_FIGURE_BACK_FEATURELINE:
//		glPushAttrib(GL_LINE_BIT);
//		{
//			glEnable(GL_LINE_STIPPLE);
//			glLineStipple(LINE_STIPPLE_FACTOR, LINE_STIPPLE_PATTER);
//
//			glPushAttrib(GL_DEPTH_BUFFER_BIT);
//			{
//				glDepthFunc (GL_GREATER);			// Change The Y Mode
//				// TODO:
//				// FeatureLine함수를 만들것
//				srgDrawVRML_NV(VRMLFileName);
//			}
//			glPopAttrib();
//		}
//		glPopAttrib();
//		break;
//	default:
//		break;
//	}
//}

//void srgDrawVRML(DWORD dwFlags, XVRML& xVRML)
//{
//	DWORD DrawingStyleTestFlags = SRG_FIGURE_DRAWINGSTYLE_MASK;
//	DWORD TextureTestFlags = SRG_FIGURE_TEXTURE_MASK;
//
//	// Drawing style을 검출한다
//	DrawingStyleTestFlags &= dwFlags;
//	TextureTestFlags &= dwFlags;
//
//	switch(DrawingStyleTestFlags)
//	{
//	case SRG_FIGURE_FILL:
//		if (TextureTestFlags)
//		{
//			//srgDrawVRML_NV_TEX(X, Z, Y);
//		}
//		else
//			srgDrawVRML_NV(xVRML);
//		break;
//	case SRG_FIGURE_WIREFRAME:
//		glPushAttrib(GL_POLYGON_BIT);
//		{
//			glPolygonMode (GL_FRONT_AND_BACK, GL_LINE);
//
//			if (TextureTestFlags)
//			{
//			}	
//			else
//				srgDrawVRML_NV(xVRML);
//		}
//		glPopAttrib();
//		break;
//	case SRG_FIGURE_SILHOUETTE:
//		{
//			glPushAttrib(GL_POLYGON_BIT);
//			{
//				glPolygonMode (GL_BACK, GL_LINE);		// Draw Back facing Polygons As Wire frames
//
//				glEnable(GL_CULL_FACE);
//				glCullFace (GL_FRONT);				// Don't Draw Any Front-Facing Polygons
//
//				glPushAttrib(GL_DEPTH_BUFFER_BIT);
//				{
//					//glDepthFunc (GL_ALWAYS);			// Change The Y Mode
//
//					// TODO:
//					// FeatureLine함수를 만들것!!
//					srgDrawVRML_NV(xVRML);
//
//					glDepthFunc (GL_LEQUAL);			// Change The Y Mode
//					//if (eQuadricState.HasTexture)
//					//{
//					//	srgDrawBox_NV_TEX(X, Z, Y);
//					//}
//					//else
//					//{
//					//	srgDrawBox_NV(X, Z, Y);
//					//}
//				}
//				glPopAttrib();
//			}
//			glPopAttrib();
//		}
//		break;
//	case SRG_FIGURE_FEATURELINE:
//		// TODO:
//		// FeatureLine함수를 만들것
//		srgDrawVRML_NV(xVRML);
//		break;
//	case SRG_FIGURE_BACK_FEATURELINE:
//		glPushAttrib(GL_LINE_BIT);
//		{
//			glEnable(GL_LINE_STIPPLE);
//			glLineStipple(LINE_STIPPLE_FACTOR, LINE_STIPPLE_PATTER);
//
//			glPushAttrib(GL_DEPTH_BUFFER_BIT);
//			{
//				glDepthFunc (GL_GREATER);			// Change The Y Mode
//				// TODO:
//				// FeatureLine함수를 만들것
//				srgDrawVRML_NV(xVRML);
//			}
//			glPopAttrib();
//		}
//		glPopAttrib();
//		break;
//	default:
//		break;
//	}
//}
//
///* VRML CODE */
//void srgDrawVRML(const std::string& VRMLFileName)
//{
//	//pxVRML.LoadVRMLFile("arm.wrl");
//	XVRML pxVRML;
//	pxVRML.LoadVRMLFile(VRMLFileName);
//
//	int i = 0;
//	int j = 0;
//	int k = 0;
//	int IFSCount = 0;
//	int CoordIndexCount = 0;
//	int IndexCount = 0;
//
//	glPushMatrix();
//	{
//		// Indexed Face Set
//		// NOTE: need at least 3 points
//		XVRMLIndexedFaceSets& IFSs = pxVRML.GetIndexedFaceSets();
//		IFSCount = (int)IFSs.GetCount();
//		for (i = 0; i < IFSCount; i++)
//		{
//			VRMLCoordIndexArray& CoordIndexes = IFSs[i]->GetCoordIndexArray();
//			VRMLCoordinateArray& Coords = IFSs[i]->GetCoordinateArray();
//			CoordIndexCount = (int)CoordIndexes.GetCount();
//			for (j = 0; j < CoordIndexCount; j++)
//			{
//				IndexCount = (int)CoordIndexes[j].GetCount();
//				if (IndexCount > 2)	// need at least 3 points
//				{
//					glBegin(GL_POLYGON);
//					{
//						for (k = 0; k < IndexCount; k++)
//						{
//							glVertex3f(
//								(GLfloat)(Coords[ (CoordIndexes[j])[k] ])[0],
//								(GLfloat)(Coords[ (CoordIndexes[j])[k] ])[1],
//								(GLfloat)(Coords[ (CoordIndexes[j])[k] ])[2]
//							);
//						}
//					}
//					glEnd();
//				}
//			}
//		}
//
//	}
//	glPopMatrix();
//}
//
///* VRML CODE */
//void srgDrawVRML_NV(const std::string& VRMLFileName)
//{
//	//pxVRML.LoadVRMLFile("arm.wrl");
//	XVRML pxVRML;
//	pxVRML.LoadVRMLFile(VRMLFileName);
//
//	int i = 0;
//	int j = 0;
//	int k = 0;
//	int IFSCount = 0;
//	int CoordIndexCount = 0;
//	int IndexCount = 0;
//
//	//glPushAttrib(GL_EVAL_BIT);
//	{
//		//glEnable(GL_AUTO_NORMAL);
//		glPushMatrix();
//		{
//			// Indexed Face Set
//			// NOTE: need at least 3 points
//			XVRMLIndexedFaceSets& IFSs = pxVRML.GetIndexedFaceSets();
//			IFSCount = (int)IFSs.GetCount();
//			for (i = 0; i < IFSCount; i++)
//			{
//				VRMLCoordIndexArray& CoordIndexes	= IFSs[i]->GetCoordIndexArray();
//				VRMLCoordinateArray& Coords			= IFSs[i]->GetCoordinateArray();
//				VRMLNormalArray& Normals			= IFSs[i]->GetNormalArray();
//
//				CoordIndexCount = (int)CoordIndexes.GetCount();
//
//				for (j = 0; j < CoordIndexCount; j++)
//				{
//					IndexCount = (CoordIndexes[j])[0];
//
//					if (IndexCount > 2)	// need at least 3 points
//					{
//						glBegin(GL_POLYGON);
//						{
//							for (k = 0; k < IndexCount; k++)
//							{
//								//int a = (CoordIndexes[j])[k + 1];
//								//double d1 = (Coords[ (CoordIndexes[j])[k + 1] ])[0]*0.001;
//								//double d2 = (Coords[ (CoordIndexes[j])[k + 1] ])[1]*0.001;
//								//double d3 = (Coords[ (CoordIndexes[j])[k + 1] ])[2]*0.001;
//								glNormal3f(
//									(GLfloat)(Normals[ (CoordIndexes[j])[k + 1] ])[0],
//									(GLfloat)(Normals[ (CoordIndexes[j])[k + 1] ])[1],
//									(GLfloat)(Normals[ (CoordIndexes[j])[k + 1] ])[2]
//								);
//								glVertex3f(
//									(GLfloat)(Coords[ (CoordIndexes[j])[k + 1] ])[0]*0.001f,
//									(GLfloat)(Coords[ (CoordIndexes[j])[k + 1] ])[1]*0.001f,
//									(GLfloat)(Coords[ (CoordIndexes[j])[k + 1] ])[2]*0.001f
//									);
//							}
//						}
//						glEnd();
//					}
//				}
//			}
//		}
//		glPopMatrix();
//	}
//	//glPopAttrib();
//}
///* VRML CODE */
//void srgDrawVRML_NV(XVRML& xVRML)
//{
//	//pxVRML.LoadVRMLFile("arm.wrl");
//	//XVRML pxVRML;
//	//pxVRML.LoadVRMLFile(VRMLFileName);
//	int i = 0;
//	int j = 0;
//	int k = 0;
//	int IFSCount = 0;
//	int CoordIndexCount = 0;
//	int IndexCount = 0;
//
//	//glPushAttrib(GL_EVAL_BIT);
//	{
//		//glEnable(GL_AUTO_NORMAL);
//		glPushMatrix();
//		{
//			// Indexed Face Set
//			// NOTE: need at least 3 points
//			XVRMLIndexedFaceSets& IFSs = xVRML.GetIndexedFaceSets();
//			IFSCount = (int)IFSs.GetCount();
//			for (i = 0; i < IFSCount; i++)
//			{
//				VRMLCoordIndexArray& CoordIndexes	= IFSs[i]->GetCoordIndexArray();
//				VRMLCoordinateArray& Coords			= IFSs[i]->GetCoordinateArray();
//				VRMLNormalArray& Normals			= IFSs[i]->GetNormalArray();
//
//				CoordIndexCount = (int)CoordIndexes.GetCount();
//
//				for (j = 0; j < CoordIndexCount; j++)
//				{
//					IndexCount = (CoordIndexes[j])[0];
//
//					if (IndexCount > 2)	// need at least 3 points
//					{
//						glBegin(GL_POLYGON);
//						{
//							for (k = 0; k < IndexCount; k++)
//							{
//								//int a = (CoordIndexes[j])[k + 1];
//								//double d1 = (Coords[ (CoordIndexes[j])[k + 1] ])[0]*0.001;
//								//double d2 = (Coords[ (CoordIndexes[j])[k + 1] ])[1]*0.001;
//								//double d3 = (Coords[ (CoordIndexes[j])[k + 1] ])[2]*0.001;
//								glNormal3f(
//									(GLfloat)(Normals[ (CoordIndexes[j])[k + 1] ])[0],
//									(GLfloat)(Normals[ (CoordIndexes[j])[k + 1] ])[1],
//									(GLfloat)(Normals[ (CoordIndexes[j])[k + 1] ])[2]
//								);
//								glVertex3f(
//									(GLfloat)(Coords[ (CoordIndexes[j])[k + 1] ])[0]*0.001f,
//									(GLfloat)(Coords[ (CoordIndexes[j])[k + 1] ])[1]*0.001f,
//									(GLfloat)(Coords[ (CoordIndexes[j])[k + 1] ])[2]*0.001f
//									);
//							}
//						}
//						glEnd();
//					}
//				}
//			}
//		}
//		glPopMatrix();
//	}
//	//glPopAttrib();
//}

void srgDrawPartialDisk(QuadricState eQuadricState, double InnerRadius, double OuterRadius, int Slices, int /* Loops */, double startAngle, double SweepAngle)
{
	GLUquadricObj *obj;
	obj = gluNewQuadric();

	ConvertXrQuadricToOpenglQuadric(eQuadricState, obj);

	// Check: Loop를 1로 고정
	gluPartialDisk(obj, InnerRadius, OuterRadius, Slices, 1, startAngle, SweepAngle);

	gluDeleteQuadric(obj);
}



//////////////////////////////////////////////////////////////////////////

void srgDrawCoordinateLoop(float Radius, int Slice, int LineWidth)
{
	glPushMatrix();
	{
		glPushAttrib(GL_LINE_BIT);
		{
			glLineWidth((GLfloat)LineWidth);

			glColor3d(0.0f, 0.0f, 1.0f);
			srgDrawLoop(Radius, Slice);
			glRotatef(90.0f, 1.0f, 0.0f, 0.0f);
			glColor3d(0.0f, 1.0f, 0.0f);
			srgDrawLoop(Radius, Slice);
			glRotatef(90.0f, 0.0f, 1.0f, 0.0f);
			glColor3d(1.0f, 0.0f, 0.0f);
			srgDrawLoop(Radius, Slice);
		}
		glPopAttrib();
	}
	glPopMatrix();
}

void srgDrawCameraLoop(float Radius, int Slice, int LineWidth)
{
	glPushMatrix();
	{
		glPushAttrib(GL_LINE_BIT);
		{
			glLineWidth((GLfloat)LineWidth);

			glColor3d(0.0f, 0.0f, 1.0f);
			srgDrawLoop(Radius, Slice);
			glRotatef(90.0f, 1.0f, 0.0f, 0.0f);
			glColor3d(0.0f, 1.0f, 0.0f);
			srgDrawLoop(Radius, Slice);
			glRotatef(90.0f, 0.0f, 1.0f, 0.0f);
			glColor3d(1.0f, 0.0f, 0.0f);
			srgDrawLoop(Radius, Slice);
		}
		glPopAttrib();
	}
	glPopMatrix();
}

void srgDrawUniAxisX(float fAxisRadius, float fAxisHeight, float fArrowRadius, float fArrowHeight)
{
	GLUquadricObj *pObj;	// Temporary, used for quadrics

	// Measurements

	// Setup the quadric object
	pObj = gluNewQuadric();
	gluQuadricDrawStyle(pObj, GLU_FILL);
	gluQuadricNormals(pObj, GLU_SMOOTH);
	gluQuadricOrientation(pObj, GLU_OUTSIDE);
	gluQuadricTexture(pObj, GLU_FALSE);

	///////////////////////////////////////////////////////
	// Draw the blue Z axis first, with arrowed head
	//glColor3f(0.0f, 0.0f, 1.0f);
	//gluCylinder(pObj, fAxisRadius, fAxisRadius, fAxisHeight, 10, 1);
	//glPushMatrix();
	//glTranslatef(0.0f, 0.0f, 1.0f);
	//gluCylinder(pObj, fArrowRadius, 0.0f, fArrowHeight, 10, 1);
	//glRotatef(180.0f, 1.0f, 0.0f, 0.0f);
	//gluDisk(pObj, fAxisRadius, fArrowRadius, 10, 1);
	//glPopMatrix();

	///////////////////////////////////////////////////////
	// Draw the Red X axis 2nd, with arrowed head
	//glColor3f(1.0f, 0.0f, 0.0f);
	glPushMatrix();
	glRotatef(90.0f, 0.0f, 1.0f, 0.0f);
	gluCylinder(pObj, fAxisRadius, fAxisRadius, fAxisHeight, 10, 1);
	glPushMatrix();
	glTranslatef(0.0f, 0.0f, 1.0f);
	gluCylinder(pObj, fArrowRadius, 0.0f, fArrowHeight, 10, 1);
	glRotatef(180.0f, 0.0f, 1.0f, 0.0f);
	gluDisk(pObj, fAxisRadius, fArrowRadius, 10, 1);
	glPopMatrix();
	glPopMatrix();

	///////////////////////////////////////////////////////
	// Draw the Green Y axis 3rd, with arrowed head
	//glColor3f(0.0f, 1.0f, 0.0f);
	//glPushMatrix();
	//glRotatef(-90.0f, 1.0f, 0.0f, 0.0f);
	//gluCylinder(pObj, fAxisRadius, fAxisRadius, fAxisHeight, 10, 1);
	//glPushMatrix();
	//glTranslatef(0.0f, 0.0f, 1.0f);
	//gluCylinder(pObj, fArrowRadius, 0.0f, fArrowHeight, 10, 1);
	//glRotatef(180.0f, 1.0f, 0.0f, 0.0f);
	//gluDisk(pObj, fAxisRadius, fArrowRadius, 10, 1);
	//glPopMatrix();
	//glPopMatrix();

	////////////////////////////////////////////////////////
	// White Sphere at origin
	//glColor3f(1.0f, 1.0f, 1.0f);
	//gluSphere(pObj, 0.05f, 15, 15);

	// Delete the quadric
	gluDeleteQuadric(pObj);
}

void srgDrawUniAxisY(float fAxisRadius, float fAxisHeight, float fArrowRadius, float fArrowHeight)
{
	GLUquadricObj *pObj;	// Temporary, used for quadrics

	// Measurements

	// Setup the quadric object
	pObj = gluNewQuadric();
	gluQuadricDrawStyle(pObj, GLU_FILL);
	gluQuadricNormals(pObj, GLU_SMOOTH);
	gluQuadricOrientation(pObj, GLU_OUTSIDE);
	gluQuadricTexture(pObj, GLU_FALSE);

	///////////////////////////////////////////////////////
	// Draw the blue Z axis first, with arrowed head
	//glColor3f(0.0f, 0.0f, 1.0f);
	//gluCylinder(pObj, fAxisRadius, fAxisRadius, fAxisHeight, 10, 1);
	//glPushMatrix();
	//glTranslatef(0.0f, 0.0f, 1.0f);
	//gluCylinder(pObj, fArrowRadius, 0.0f, fArrowHeight, 10, 1);
	//glRotatef(180.0f, 1.0f, 0.0f, 0.0f);
	//gluDisk(pObj, fAxisRadius, fArrowRadius, 10, 1);
	//glPopMatrix();

	///////////////////////////////////////////////////////
	// Draw the Red X axis 2nd, with arrowed head
	//glColor3f(1.0f, 0.0f, 0.0f);
	glPushMatrix();
	glRotatef(90.0f, 0.0f, 1.0f, 0.0f);
	gluCylinder(pObj, fAxisRadius, fAxisRadius, fAxisHeight, 10, 1);
	glPushMatrix();
	glTranslatef(0.0f, 0.0f, 1.0f);
	gluCylinder(pObj, fArrowRadius, 0.0f, fArrowHeight, 10, 1);
	glRotatef(180.0f, 0.0f, 1.0f, 0.0f);
	gluDisk(pObj, fAxisRadius, fArrowRadius, 10, 1);
	glPopMatrix();
	glPopMatrix();

	///////////////////////////////////////////////////////
	// Draw the Green Y axis 3rd, with arrowed head
	//glColor3f(0.0f, 1.0f, 0.0f);
	//glPushMatrix();
	//glRotatef(-90.0f, 1.0f, 0.0f, 0.0f);
	//gluCylinder(pObj, fAxisRadius, fAxisRadius, fAxisHeight, 10, 1);
	//glPushMatrix();
	//glTranslatef(0.0f, 0.0f, 1.0f);
	//gluCylinder(pObj, fArrowRadius, 0.0f, fArrowHeight, 10, 1);
	//glRotatef(180.0f, 1.0f, 0.0f, 0.0f);
	//gluDisk(pObj, fAxisRadius, fArrowRadius, 10, 1);
	//glPopMatrix();
	//glPopMatrix();

	////////////////////////////////////////////////////////
	// White Sphere at origin
	//glColor3f(1.0f, 1.0f, 1.0f);
	//gluSphere(pObj, 0.05f, 15, 15);

	// Delete the quadric
	gluDeleteQuadric(pObj);
}

void srgDrawUniAxisZ(float fAxisRadius, float fAxisHeight, float fArrowRadius, float fArrowHeight)
{
	GLUquadricObj *pObj;	// Temporary, used for quadrics

	// Measurements

	// Setup the quadric object
	pObj = gluNewQuadric();
	gluQuadricDrawStyle(pObj, GLU_FILL);
	gluQuadricNormals(pObj, GLU_SMOOTH);
	gluQuadricOrientation(pObj, GLU_OUTSIDE);
	gluQuadricTexture(pObj, GLU_FALSE);

	///////////////////////////////////////////////////////
	// Draw the blue Z axis first, with arrowed head
	//glColor3f(0.0f, 0.0f, 1.0f);
	//gluCylinder(pObj, fAxisRadius, fAxisRadius, fAxisHeight, 10, 1);
	//glPushMatrix();
	//glTranslatef(0.0f, 0.0f, 1.0f);
	//gluCylinder(pObj, fArrowRadius, 0.0f, fArrowHeight, 10, 1);
	//glRotatef(180.0f, 1.0f, 0.0f, 0.0f);
	//gluDisk(pObj, fAxisRadius, fArrowRadius, 10, 1);
	//glPopMatrix();

	///////////////////////////////////////////////////////
	// Draw the Red X axis 2nd, with arrowed head
	//glColor3f(1.0f, 0.0f, 0.0f);
	glPushMatrix();
	glRotatef(90.0f, 0.0f, 1.0f, 0.0f);
	gluCylinder(pObj, fAxisRadius, fAxisRadius, fAxisHeight, 10, 1);
	glPushMatrix();
	glTranslatef(0.0f, 0.0f, 1.0f);
	gluCylinder(pObj, fArrowRadius, 0.0f, fArrowHeight, 10, 1);
	glRotatef(180.0f, 0.0f, 1.0f, 0.0f);
	gluDisk(pObj, fAxisRadius, fArrowRadius, 10, 1);
	glPopMatrix();
	glPopMatrix();

	///////////////////////////////////////////////////////
	// Draw the Green Y axis 3rd, with arrowed head
	//glColor3f(0.0f, 1.0f, 0.0f);
	//glPushMatrix();
	//glRotatef(-90.0f, 1.0f, 0.0f, 0.0f);
	//gluCylinder(pObj, fAxisRadius, fAxisRadius, fAxisHeight, 10, 1);
	//glPushMatrix();
	//glTranslatef(0.0f, 0.0f, 1.0f);
	//gluCylinder(pObj, fArrowRadius, 0.0f, fArrowHeight, 10, 1);
	//glRotatef(180.0f, 1.0f, 0.0f, 0.0f);
	//gluDisk(pObj, fAxisRadius, fArrowRadius, 10, 1);
	//glPopMatrix();
	//glPopMatrix();

	////////////////////////////////////////////////////////
	// White Sphere at origin
	//glColor3f(1.0f, 1.0f, 1.0f);
	//gluSphere(pObj, 0.05f, 15, 15);

	// Delete the quadric
	gluDeleteQuadric(pObj);
}

void srgDrawUniAxisX2(float fAxisRadius, float fAxisHeight, float fArrowRadius, float fArrowHeight)
{
	GLUquadricObj *pObj;	// Temporary, used for quadrics
	QuadricState eQuadricState;

	eQuadricState.DrawStyle = drawstyleFILL;
	eQuadricState.HasTexture = false;

	// Measurements

	// Setup the quadric object
	pObj = gluNewQuadric();
	gluQuadricDrawStyle(pObj, GLU_FILL);
	gluQuadricNormals(pObj, GLU_SMOOTH);
	gluQuadricOrientation(pObj, GLU_OUTSIDE);
	gluQuadricTexture(pObj, GLU_FALSE);

	///////////////////////////////////////////////////////
	// Draw the blue Z axis first, with arrowed head
	//glColor3f(0.0f, 0.0f, 1.0f);
	//gluCylinder(pObj, fAxisRadius, fAxisRadius, fAxisHeight, 10, 1);
	//glPushMatrix();
	//glTranslatef(0.0f, 0.0f, 1.0f);
	//gluCylinder(pObj, fArrowRadius, 0.0f, fArrowHeight, 10, 1);
	//glRotatef(180.0f, 1.0f, 0.0f, 0.0f);
	//gluDisk(pObj, fAxisRadius, fArrowRadius, 10, 1);
	//glPopMatrix();

	///////////////////////////////////////////////////////
	// Draw the Red X axis 2nd, with arrowed head
	//glColor3f(1.0f, 0.0f, 0.0f);
	glPushMatrix();
	glRotatef(90.0f, 0.0f, 1.0f, 0.0f);
	srgDrawBox(eQuadricState, fAxisRadius, fAxisRadius, fAxisHeight);
	glPushMatrix();
	glTranslatef(0.0f, 0.0f, 0.7f);
	gluCylinder(pObj, fArrowRadius, 0.0f, fArrowHeight, 10, 1);
	glRotatef(180.0f, 0.0f, 1.0f, 0.0f);
	gluDisk(pObj, fAxisRadius, fArrowRadius, 10, 1);

	//glRotatef(180.0f, 0.0f, 1.0f, 0.0f);
	//glTranslatef(0.0f, 0.0f, 1.4f);
	//gluCylinder(pObj, fArrowRadius, 0.0f, fArrowHeight, 10, 1);
	//glRotatef(180.0f, 0.0f, 1.0f, 0.0f);
	//gluDisk(pObj, fAxisRadius, fArrowRadius, 10, 1);
	glPopMatrix();
	glPopMatrix();

	///////////////////////////////////////////////////////
	// Draw the Green Y axis 3rd, with arrowed head
	//glColor3f(0.0f, 1.0f, 0.0f);
	//glPushMatrix();
	//glRotatef(-90.0f, 1.0f, 0.0f, 0.0f);
	//gluCylinder(pObj, fAxisRadius, fAxisRadius, fAxisHeight, 10, 1);
	//glPushMatrix();
	//glTranslatef(0.0f, 0.0f, 1.0f);
	//gluCylinder(pObj, fArrowRadius, 0.0f, fArrowHeight, 10, 1);
	//glRotatef(180.0f, 1.0f, 0.0f, 0.0f);
	//gluDisk(pObj, fAxisRadius, fArrowRadius, 10, 1);
	//glPopMatrix();
	//glPopMatrix();

	////////////////////////////////////////////////////////
	// White Sphere at origin
	//glColor3f(1.0f, 1.0f, 1.0f);
	//gluSphere(pObj, 0.05f, 15, 15);

	// Delete the quadric
	gluDeleteQuadric(pObj);
}

void srgDrawUnitAxes(void)
{
	GLUquadricObj *pObj;	// Temporary, used for quadrics

	// Measurements
	float fAxisRadius = 0.025f;
	float fAxisHeight = 1.0f;
	float fArrowRadius = 0.06f;
	float fArrowHeight = 0.1f;

	// Setup the quadric object
	pObj = gluNewQuadric();
	gluQuadricDrawStyle(pObj, GLU_FILL);
	gluQuadricNormals(pObj, GLU_SMOOTH);
	gluQuadricOrientation(pObj, GLU_OUTSIDE);
	gluQuadricTexture(pObj, GLU_FALSE);

	///////////////////////////////////////////////////////
	// Draw the blue Z axis first, with arrowed head
	glColor3f(0.0f, 0.0f, 1.0f);
	gluCylinder(pObj, fAxisRadius, fAxisRadius, fAxisHeight, 10, 1);
	glPushMatrix();
	glTranslatef(0.0f, 0.0f, 1.0f);
	gluCylinder(pObj, fArrowRadius, 0.0f, fArrowHeight, 10, 1);
	glRotatef(180.0f, 1.0f, 0.0f, 0.0f);
	gluDisk(pObj, fAxisRadius, fArrowRadius, 10, 1);
	glPopMatrix();

	///////////////////////////////////////////////////////
	// Draw the Red X axis 2nd, with arrowed head
	glColor3f(1.0f, 0.0f, 0.0f);
	glPushMatrix();
	glRotatef(90.0f, 0.0f, 1.0f, 0.0f);
	gluCylinder(pObj, fAxisRadius, fAxisRadius, fAxisHeight, 10, 1);
	glPushMatrix();
	glTranslatef(0.0f, 0.0f, 1.0f);
	gluCylinder(pObj, fArrowRadius, 0.0f, fArrowHeight, 10, 1);
	glRotatef(180.0f, 0.0f, 1.0f, 0.0f);
	gluDisk(pObj, fAxisRadius, fArrowRadius, 10, 1);
	glPopMatrix();
	glPopMatrix();

	///////////////////////////////////////////////////////
	// Draw the Green Y axis 3rd, with arrowed head
	glColor3f(0.0f, 1.0f, 0.0f);
	glPushMatrix();
	glRotatef(-90.0f, 1.0f, 0.0f, 0.0f);
	gluCylinder(pObj, fAxisRadius, fAxisRadius, fAxisHeight, 10, 1);
	glPushMatrix();
	glTranslatef(0.0f, 0.0f, 1.0f);
	gluCylinder(pObj, fArrowRadius, 0.0f, fArrowHeight, 10, 1);
	glRotatef(180.0f, 1.0f, 0.0f, 0.0f);
	gluDisk(pObj, fAxisRadius, fArrowRadius, 10, 1);
	glPopMatrix();
	glPopMatrix();

	////////////////////////////////////////////////////////
	// White Sphere at origin
	glColor3f(1.0f, 1.0f, 1.0f);
	gluSphere(pObj, 0.05f, 15, 15);

	// Delete the quadric
	gluDeleteQuadric(pObj);
}

//void srgFig_UnitAxisX(srgColor Color, float fStartPt, float fArrowRadius, float fArrowHeight, int iSlices, int iStacks)
//{
//	GLUquadricObj *pObj = NULL;	// Temporary, used for quadrics
//
//	// Measurements
//	// 	float fAxisRadius = 0.025f;
//	// 	float fAxisHeight = 1.0f;
//	// 	float fArrowRadius = 0.06f;
//	// 	float fArrowHeight = 0.1f;
//
//	// Setup the quadric object
//	pObj = gluNewQuadric();
//	gluQuadricDrawStyle(pObj, GLU_FILL);
//	gluQuadricNormals(pObj, GLU_SMOOTH);
//	//gluQuadricOrientation(pObj, GLU_OUTSIDE); // default 이므로 할 필요 없다.
//	gluQuadricTexture(pObj, GLU_FALSE);
//
//	///////////////////////////////////////////////////////
//	// Draw the red X axis first, with arrowed head
//	//glColor3f(0.0f, 0.0f, 1.0f);
//
//	glPushMatrix();
//	{
//		// 그리는 순서 (블렌딩 때문에, 깊이 테스트를 하지 않으므로,)
//		// 1. 원뿔의 밑면
//		// 2. 라인
//		// 3. 원뿔
//		//gluCylinder(pObj, fAxisRadius, fAxisRadius, fAxisHeight, 10, 1);
//		//glTranslatef(0.0f, 0.0f, 1.0f);
//		glRotatef(270.0f, 0.0f, 1.0f, 0.0f);				// z축이 x축으로
//		glTranslatef(0.0f, 0.0f, -1.0f + fArrowHeight);	// z축으로 이동
//		srgColor DarkColor(Color);
//		DarkColor.FadeOut(0.5f);
//		DarkColor.PushAttrib();
//		gluDisk(pObj, 0.0f, fArrowRadius, iSlices, iStacks);
//		DarkColor.PopAttrib();
//
//		Color.PushAttrib();
//		glBegin(GL_LINES);
//		{
//			glVertex3f(0.0f, 0.0f, 1.0f - fArrowHeight - fStartPt);
//			glVertex3f(0.0f, 0.0f, 0.0f);
//		}
//		glEnd();
//
//		glRotatef(180.0f, 0.0f, 1.0f, 0.0f);
//		gluCylinder(pObj, fArrowRadius, 0.0, fArrowHeight, iSlices, iStacks);
//		Color.PopAttrib();
//	}
//	glPopMatrix();
//
//	// Delete the quadric
//	gluDeleteQuadric(pObj);
//}
//
//void srgFig_UnitAxisY(srgColor Color, float fStartPt, float fArrowRadius, float fArrowHeight, int iSlices, int iStacks)
//{
//	GLUquadricObj *pObj = NULL;	// Temporary, used for quadrics
//
//	// Measurements
//	// 	float fAxisRadius = 0.025f;
//	// 	float fAxisHeight = 1.0f;
//	// 	float fArrowRadius = 0.06f;
//	// 	float fArrowHeight = 0.1f;
//
//	// Setup the quadric object
//	pObj = gluNewQuadric();
//	gluQuadricDrawStyle(pObj, GLU_FILL);
//	gluQuadricNormals(pObj, GLU_SMOOTH);
//	gluQuadricOrientation(pObj, GLU_OUTSIDE);
//	gluQuadricTexture(pObj, GLU_FALSE);
//
//	///////////////////////////////////////////////////////
//	// Draw the red X axis first, with arrowed head
//	//glColor3f(0.0f, 0.0f, 1.0f);
//
//	glPushMatrix();
//	{
//		//gluCylinder(pObj, fAxisRadius, fAxisRadius, fAxisHeight, 10, 1);
//		//glTranslatef(0.0f, 0.0f, 1.0f);
//		Color.PushAttrib();
//		glBegin(GL_LINES);
//		{
//			glVertex3f(0.0f, fStartPt, 0.0f);
//			glVertex3f(0.0f, 1.0f - fArrowHeight, 0.0f);
//		}
//		glEnd();
//		glRotatef(-90.0f, 1.0f, 0.0f, 0.0f);
//		glTranslatef(0.0f, 0.0f, 1.0f - fArrowHeight);
//		gluCylinder(pObj, fArrowRadius, 0.0, fArrowHeight, iSlices, iStacks);
//		Color.PopAttrib();
//		glRotatef(180.0f, 0.0f, 1.0f, 0.0f);
//		Color.FadeOut(0.5f);
//		Color.PushAttrib();
//		gluDisk(pObj, 0.0f, fArrowRadius, iSlices, iStacks);
//		Color.PopAttrib();
//	}
//	glPopMatrix();
//
//	// Delete the quadric
//	gluDeleteQuadric(pObj);
//}
//
//void srgFig_UnitAxisZ(srgColor Color, float fStartPt, float fArrowRadius, float fArrowHeight, int iSlices, int iStacks)
//{
//	GLUquadricObj *pObj = NULL;	// Temporary, used for quadrics
//
//	// Measurements
//	// 	float fAxisRadius = 0.025f;
//	// 	float fAxisHeight = 1.0f;
//	// 	float fArrowRadius = 0.06f;
//	// 	float fArrowHeight = 0.1f;
//
//	// Setup the quadric object
//	pObj = gluNewQuadric();
//	gluQuadricDrawStyle(pObj, GLU_FILL);
//	gluQuadricNormals(pObj, GLU_SMOOTH);
//	gluQuadricOrientation(pObj, GLU_OUTSIDE);
//	gluQuadricTexture(pObj, GLU_FALSE);
//
//	///////////////////////////////////////////////////////
//	// Draw the red X axis first, with arrowed head
//	//glColor3f(0.0f, 0.0f, 1.0f);
//
//	glPushMatrix();
//	{
//		//gluCylinder(pObj, fAxisRadius, fAxisRadius, fAxisHeight, 10, 1);
//		//glTranslatef(0.0f, 0.0f, 1.0f);
//		Color.PushAttrib();
//		glBegin(GL_LINES);
//		{
//			glVertex3f(0.0f, 0.0f, fStartPt);
//			glVertex3f(0.0f, 0.0f, 1.0f - fArrowHeight);
//		}
//		glEnd();
//		//glRotatef(90.0f, 0.0f, 1.0f, 0.0f);
//		glTranslatef(0.0f, 0.0f, 1.0f - fArrowHeight);
//		gluCylinder(pObj, fArrowRadius, 0.0, fArrowHeight, iSlices, iStacks);
//		Color.PopAttrib();
//		glRotatef(180.0f, 0.0f, 1.0f, 0.0f);
//		Color.FadeOut(0.5f);
//		Color.PushAttrib();
//		gluDisk(pObj, 0.0f, fArrowRadius, iSlices, iStacks);
//		Color.PopAttrib();
//	}
//	glPopMatrix();
//
//	// Delete the quadric
//	gluDeleteQuadric(pObj);
//}
//
//void srgFig_UnitPlanXY_2(srgColor ColorZ, float fStartPt)
//{
//	float fHalfStartPt = fStartPt*0.5f;
//
//	//glPushMatrix();
//	{
//		glPushAttrib(GL_COLOR_BUFFER_BIT);
//		{
//			glEnable(GL_BLEND);
//			// SRC,, DST,,
//			glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
//
//			glPushAttrib(GL_DEPTH_BUFFER_BIT);
//			{
//				glDepthFunc(GL_LEQUAL);
//
//				ColorZ.PushAttrib();
//				glBegin(GL_QUADS);
//				{
//					glVertex3f( fHalfStartPt,  fHalfStartPt, fStartPt);
//					glVertex3f(-fHalfStartPt,  fHalfStartPt, fStartPt);
//					glVertex3f(-fHalfStartPt, -fHalfStartPt, fStartPt);
//					glVertex3f( fHalfStartPt, -fHalfStartPt, fStartPt);
//				}
//				glEnd();
//				ColorZ.PopAttrib();
//			}
//			glPopAttrib();
//		}
//		glPopAttrib();
//	}
//	//glPopMatrix();
//}
//
//void srgFig_UnitPlanYZ_2(srgColor ColorX, float fStartPt)
//{
//	float fHalfStartPt = fStartPt*0.5f;
//
//	//glPushMatrix();
//	{
//		glPushAttrib(GL_COLOR_BUFFER_BIT);
//		{
//			glEnable(GL_BLEND);
//			// SRC,, DST,,
//			glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
//
//			glPushAttrib(GL_DEPTH_BUFFER_BIT);
//			{
//				glDepthFunc(GL_LEQUAL);
//
//				ColorX.PushAttrib();
//				glBegin(GL_QUADS);
//				{
//					glVertex3f(fStartPt,  fHalfStartPt,  fHalfStartPt);
//					glVertex3f(fStartPt, -fHalfStartPt,  fHalfStartPt);
//					glVertex3f(fStartPt, -fHalfStartPt, -fHalfStartPt);
//					glVertex3f(fStartPt,  fHalfStartPt, -fHalfStartPt);
//				}
//				glEnd();
//				ColorX.PopAttrib();
//			}
//			glPopAttrib();
//		}
//		glPopAttrib();
//	}
//	//glPopMatrix();
//}
//
//void srgFig_UnitPlanZX_2(srgColor ColorY, float fStartPt)
//{
//	float fHalfStartPt = fStartPt*0.5f;
//
//	//glPushMatrix();
//	{
//		glPushAttrib(GL_COLOR_BUFFER_BIT);
//		{
//			glEnable(GL_BLEND);
//			// SRC,, DST,,
//			glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
//
//			glPushAttrib(GL_DEPTH_BUFFER_BIT);
//			{
//				glDepthFunc(GL_LEQUAL);
//
//				ColorY.PushAttrib();
//				glBegin(GL_QUADS);
//				{
//					glVertex3f( fHalfStartPt,  fStartPt,  fHalfStartPt);
//					glVertex3f( fHalfStartPt,  fStartPt, -fHalfStartPt);
//					glVertex3f(-fHalfStartPt,  fStartPt, -fHalfStartPt);
//					glVertex3f(-fHalfStartPt,  fStartPt,  fHalfStartPt);
//				}
//				glEnd();
//				ColorY.PopAttrib();
//			}
//			glPopAttrib();
//		}
//		glPopAttrib();
//	}
//	//glPopMatrix();
//}
//
//void srgFig_UnitPlanXY_3(srgColor ColorZ, float fStartPt)
//{
//	//glPushMatrix();
//	{
//		glPushAttrib(GL_COLOR_BUFFER_BIT);
//		{
//			glEnable(GL_BLEND);
//			// SRC,, DST,,
//			glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
//
//			glPushAttrib(GL_DEPTH_BUFFER_BIT);
//			{
//				glDepthFunc(GL_LEQUAL);
//
//				ColorZ.PushAttrib();
//				glBegin(GL_QUADS);
//				{
//					glVertex3f( fStartPt,  fStartPt, fStartPt);
//					glVertex3f(-fStartPt,  fStartPt, fStartPt);
//					glVertex3f(-fStartPt, -fStartPt, fStartPt);
//					glVertex3f( fStartPt, -fStartPt, fStartPt);
//				}
//				glEnd();
//				ColorZ.PopAttrib();
//			}
//			glPopAttrib();
//		}
//		glPopAttrib();
//	}
//	//glPopMatrix();
//}
//
//void srgFig_UnitPlanYZ_3(srgColor ColorX, float fStartPt)
//{
//	//glPushMatrix();
//	{
//		glPushAttrib(GL_COLOR_BUFFER_BIT);
//		{
//			glEnable(GL_BLEND);
//			// SRC,, DST,,
//			glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
//
//			glPushAttrib(GL_DEPTH_BUFFER_BIT);
//			{
//				glDepthFunc(GL_LEQUAL);
//
//				ColorX.PushAttrib();
//				glBegin(GL_QUADS);
//				{
//					glVertex3f(fStartPt,  fStartPt,  fStartPt);
//					glVertex3f(fStartPt, -fStartPt,  fStartPt);
//					glVertex3f(fStartPt, -fStartPt, -fStartPt);
//					glVertex3f(fStartPt,  fStartPt, -fStartPt);
//				}
//				glEnd();
//				ColorX.PopAttrib();
//			}
//			glPopAttrib();
//		}
//		glPopAttrib();
//	}
//	//glPopMatrix();
//}
//
//void srgFig_UnitPlanZX_3(srgColor ColorY, float fStartPt)
//{
//	//glPushMatrix();
//	{
//		glPushAttrib(GL_COLOR_BUFFER_BIT);
//		{
//			glEnable(GL_BLEND);
//			// SRC,, DST,,
//			glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
//
//			glPushAttrib(GL_DEPTH_BUFFER_BIT);
//			{
//				glDepthFunc(GL_LEQUAL);
//
//				ColorY.PushAttrib();
//				glBegin(GL_QUADS);
//				{
//					glVertex3f( fStartPt,  fStartPt,  fStartPt);
//					glVertex3f( fStartPt,  fStartPt, -fStartPt);
//					glVertex3f(-fStartPt,  fStartPt, -fStartPt);
//					glVertex3f(-fStartPt,  fStartPt,  fStartPt);
//				}
//				glEnd();
//				ColorY.PopAttrib();
//			}
//			glPopAttrib();
//		}
//		glPopAttrib();
//	}
//	//glPopMatrix();
//}
//
//void srgFig_UnitPlanXY(srgColor ColorX, srgColor ColorY, float fStartPt)
//{
//	//glPushMatrix();
//	{
//		ColorX.PushAttrib();
//		glBegin(GL_LINES);
//		{
//			glVertex3f(fStartPt, 0.0f, 0.0f);
//			glVertex3f(fStartPt, fStartPt, 0.0f);
//		}
//		glEnd();
//		ColorX.PopAttrib();
//
//		ColorY.PushAttrib();
//		glBegin(GL_LINES);
//		{
//			glVertex3f(0.0f, fStartPt, 0.0f);
//			glVertex3f(fStartPt, fStartPt, 0.0f);
//		}
//		glEnd();
//		ColorY.PopAttrib();
//	}
//	//glPopMatrix();
//}
//
//void srgFig_UnitPlanYZ(srgColor ColorY, srgColor ColorZ, float fStartPt)
//{
//	//glPushMatrix();
//	{
//		ColorY.PushAttrib();
//		glBegin(GL_LINES);
//		{
//			glVertex3f(0.0f, fStartPt, 0.0f);
//			glVertex3f(0.0f, fStartPt, fStartPt);
//		}
//		glEnd();
//		ColorY.PopAttrib();
//
//		ColorZ.PushAttrib();
//		glBegin(GL_LINES);
//		{
//			glVertex3f(0.0f, 0.0f, fStartPt);
//			glVertex3f(0.0f, fStartPt, fStartPt);
//		}
//		glEnd();
//		ColorZ.PopAttrib();
//	}
//	//glPopMatrix();
//}
//
//void srgFig_UnitPlanZX(srgColor ColorZ, srgColor ColorX, float fStartPt)
//{
//	//glPushMatrix();
//	{
//		ColorZ.PushAttrib();
//		glBegin(GL_LINES);
//		{
//			glVertex3f(0.0f, 0.0f, fStartPt);
//			glVertex3f(fStartPt, 0.0f, fStartPt);
//		}
//		glEnd();
//		ColorZ.PopAttrib();
//
//		ColorX.PushAttrib();
//		glBegin(GL_LINES);
//		{
//			glVertex3f(fStartPt, 0.0f, 0.0f);
//			glVertex3f(fStartPt, 0.0f, fStartPt);
//		}
//		glEnd();
//		ColorX.PopAttrib();
//	}
//	//glPopMatrix();
//}
//
//void srgFig_Origin(srgColor ColorOrigin, float fRadius, int iSlices, int iStacks)
//{
//	GLUquadricObj *pObj = NULL;	// Temporary, used for quadrics
//
//	// Setup the quadric object
//	pObj = gluNewQuadric();
//	gluQuadricDrawStyle(pObj, GLU_FILL);
//	gluQuadricNormals(pObj, GLU_SMOOTH);
//	gluQuadricOrientation(pObj, GLU_OUTSIDE);
//	gluQuadricTexture(pObj, GLU_FALSE);
//
//	glPushMatrix();
//	{
//		ColorOrigin.PushAttrib();
//		gluSphere(pObj, fRadius, iSlices, iStacks);
//		ColorOrigin.PopAttrib();
//	}
//	glPopMatrix();
//
//	// Delete the quadric
//	gluDeleteQuadric(pObj);
//}
//
//void srgDrawCoordinateSimple(void)
//{
//	glBegin(GL_LINES);
//	{
//		glColor3f(1.0f, 0.0f, 0.0f);
//		glVertex3f(0.0f, 0.0f, 0.0f);
//		glVertex3f(1.0f, 0.0f, 0.0f);
//
//		glColor3f(0.0f, 1.0f, 0.0f);
//		glVertex3f(0.0f, 0.0f, 0.0f);
//		glVertex3f(0.0f, 1.0f, 0.0f);
//
//		glColor3f(0.0f, 0.0f, 1.0f);
//		glVertex3f(0.0f, 0.0f, 0.0f);
//		glVertex3f(0.0f, 0.0f, 1.0f);
//	}
//	glEnd();
//}








void _draw_box(const sr_real _sz[3])
{
	float sz[3]		= { (float)_sz[0], (float)_sz[1], (float)_sz[2] };

	float data[][8] = {	{  0.0f,  0.0f,  0.0f,  0.0f,  1.0f, -sz[0], -sz[1],  sz[2] },
	{  1.0f,  0.0f,  0.0f,  0.0f,  1.0f,  sz[0], -sz[1],  sz[2] },
	{  1.0f,  1.0f,  0.0f,  0.0f,  1.0f,  sz[0],  sz[1],  sz[2] },
	{  0.0f,  1.0f,  0.0f,  0.0f,  1.0f, -sz[0],  sz[1],  sz[2] },
	{  1.0f,  0.0f,  0.0f,  0.0f, -1.0f, -sz[0], -sz[1], -sz[2] },
	{  1.0f,  1.0f,  0.0f,  0.0f, -1.0f, -sz[0],  sz[1], -sz[2] },
	{  0.0f,  1.0f,  0.0f,  0.0f, -1.0f,  sz[0],  sz[1], -sz[2] },
	{  0.0f,  0.0f,  0.0f,  0.0f, -1.0f,  sz[0], -sz[1], -sz[2] },
	{  0.0f,  1.0f,  0.0f,  1.0f,  0.0f, -sz[0],  sz[1], -sz[2] },
	{  0.0f,  0.0f,  0.0f,  1.0f,  0.0f, -sz[0],  sz[1],  sz[2] },
	{  1.0f,  0.0f,  0.0f,  1.0f,  0.0f,  sz[0],  sz[1],  sz[2] },
	{  1.0f,  1.0f,  0.0f,  1.0f,  0.0f,  sz[0],  sz[1], -sz[2] },
	{  1.0f,  1.0f,  0.0f, -1.0f,  0.0f, -sz[0], -sz[1], -sz[2] },
	{  0.0f,  1.0f,  0.0f, -1.0f,  0.0f,  sz[0], -sz[1], -sz[2] },
	{  0.0f,  0.0f,  0.0f, -1.0f,  0.0f,  sz[0], -sz[1],  sz[2] },
	{  1.0f,  0.0f,  0.0f, -1.0f,  0.0f, -sz[0], -sz[1],  sz[2] },
	{  1.0f,  0.0f,  1.0f,  0.0f,  0.0f,  sz[0], -sz[1], -sz[2] },
	{  1.0f,  1.0f,  1.0f,  0.0f,  0.0f,  sz[0],  sz[1], -sz[2] },
	{  0.0f,  1.0f,  1.0f,  0.0f,  0.0f,  sz[0],  sz[1],  sz[2] },
	{  0.0f,  0.0f,  1.0f,  0.0f,  0.0f,  sz[0], -sz[1],  sz[2] },
	{  0.0f,  0.0f, -1.0f,  0.0f,  0.0f, -sz[0], -sz[1], -sz[2] },
	{  1.0f,  0.0f, -1.0f,  0.0f,  0.0f, -sz[0], -sz[1],  sz[2] },
	{  1.0f,  1.0f, -1.0f,  0.0f,  0.0f, -sz[0],  sz[1],  sz[2] },
	{  0.0f,  1.0f, -1.0f,  0.0f,  0.0f, -sz[0],  sz[1], -sz[2] }	};

	glInterleavedArrays(GL_T2F_N3F_V3F, 0, data);
	glDrawArrays(GL_QUADS, 0, 24);
}

void srgDrawGrid( void )
{
	glPushAttrib(GL_LINE_BIT);
	//glPushAttrib(GL_CURRENT_BIT);
	{
		const GLint RANGE = 10;

		glColor4f(0.5f, 0.5f, 0.5f, 1.0f);
		glBegin(GL_LINES);
		for (int i = -RANGE; i <= RANGE; i += 1)
		{
			if (i == 0 || (i % 100) == 0) continue;

			glVertex3i( RANGE, i, 0);
			glVertex3i(-RANGE, i, 0);

			glVertex3i(i, -RANGE, 0);
			glVertex3i(i,  RANGE, 0);
		}
		glEnd();

		glColor4f(0.5f, 0.5f, 0.5f, 1.0f);
		glBegin(GL_LINES);
		for(int i = -RANGE; i <= RANGE; i += 10)
		{
			if (i == 0) continue;

			glVertex3i( RANGE, i, 0);
			glVertex3i(-RANGE, i, 0);

			glVertex3i(i, -RANGE, 0);
			glVertex3i(i,  RANGE, 0);
		}
		glEnd();

		glLineWidth(2);
		glColor4f(0.5f, 0.5f, 0.5f, 1.0f);
		glBegin(GL_LINES);
		glVertex3i( RANGE, 0, 0);
		glVertex3i(-RANGE, 0, 0);
		glEnd();

		glBegin(GL_LINES);
		glVertex3i(0, -RANGE, 0);
		glVertex3i(0,  RANGE, 0);
		glEnd();
		glLineWidth(1);
	}
	//glPopAttrib();
	glPopAttrib();
}

void srgDrawGrid_Flexible( int ViewportHeight, float SightHeight )
{
	glPushAttrib(GL_LINE_BIT);
	//glPushAttrib(GL_CURRENT_BIT);
	{
		const int UNIT_PIXEL = 10;
		double RANGE = (int)(SightHeight);
		if (RANGE > 3000.0)
			return;
		double STEP_SIZE = (int)((double)UNIT_PIXEL*(double)SightHeight/(double)ViewportHeight);

		int Order = 0;
		double dTest_Num = STEP_SIZE;

		while (int(dTest_Num) > 0)
		{
			dTest_Num *= 0.1;
			Order++;
		}

		STEP_SIZE = pow(10.0, Order);

		int NUM_LINE = int(RANGE/STEP_SIZE);

		glColor4f(0.5f, 0.5f, 0.5f, 1.0f);
		
		if (STEP_SIZE > 0)
		{
      glLineWidth(1);
			glBegin(GL_LINES);

			for (int i = 0; i < NUM_LINE; ++i)
			{
				if (i == 0 || (i % 10) == 0) continue;

				glVertex3f( RANGE, i*STEP_SIZE, 0);
				glVertex3f(-RANGE, i*STEP_SIZE, 0);

				glVertex3f( RANGE, -i*STEP_SIZE, 0);
				glVertex3f(-RANGE, -i*STEP_SIZE, 0);

				glVertex3f(i*STEP_SIZE, -RANGE, 0);
				glVertex3f(i*STEP_SIZE,  RANGE, 0);

				glVertex3f(-i*STEP_SIZE, -RANGE, 0);
				glVertex3f(-i*STEP_SIZE,  RANGE, 0);
			}
			glEnd();
		}
		
		if (STEP_SIZE > 0)
		{
			glColor4f(0.5f, 0.5f, 0.5f, 1.0f);
			glLineWidth(2);
			glBegin(GL_LINES);
			for (int i = 0; i < NUM_LINE; i += 10)
			{
				if (i == 0 /*|| (i % 100) == 0*/) continue;

				glVertex3f( RANGE, i*STEP_SIZE, 0);
				glVertex3f(-RANGE, i*STEP_SIZE, 0);

				glVertex3f( RANGE, -i*STEP_SIZE, 0);
				glVertex3f(-RANGE, -i*STEP_SIZE, 0);

				glVertex3f(i*STEP_SIZE, -RANGE, 0);
				glVertex3f(i*STEP_SIZE,  RANGE, 0);

				glVertex3f(-i*STEP_SIZE, -RANGE, 0);
				glVertex3f(-i*STEP_SIZE,  RANGE, 0);
			}
			glEnd();
		}

		//glColor4f(0.5f, 0.5f, 0.5f, 1.0f);
		//glBegin(GL_LINES);
		//for(int i = -RANGE; i <= RANGE; i += 10*STEP)
		//{
		//	if (i == 0) continue;

		//	glVertex3i( RANGE, i, 0);
		//	glVertex3i(-RANGE, i, 0);

		//	glVertex3i(i, -RANGE, 0);
		//	glVertex3i(i,  RANGE, 0);
		//}
		//glEnd();

		glLineWidth(2);
		glColor4f(0.5f, 0.5f, 0.5f, 1.0f);
		glBegin(GL_LINES);
		glVertex3i( RANGE, 0, 0);
		glVertex3i(-RANGE, 0, 0);
		glEnd();

		glBegin(GL_LINES);
		glVertex3i(0, -RANGE, 0);
		glVertex3i(0,  RANGE, 0);
		glEnd();
		glLineWidth(1);
	}
	//glPopAttrib();
	glPopAttrib();
}

void srgDrawCoordinate_axis_basic(void)
{
	// ..1..1..    ..1..1..    ..1111..     
	// ..1..1..    ..1..1..    .....1..     
	// ...11...    ..1..1..    ....1...     
	// ...11...    ..1..1..    ...1....     
	// ..1..1..    ...11...    ..1.....     
	// ..1..1..    ...1....    ..1111..     
	// ........    ...1....    ........     
	// ........    .11.....    ........     

	GLubyte x[] = {0x00, 0x00, 0x24, 0x24, 0x18, 0x18, 0x24, 0x24};
	GLubyte y[] = {0x60, 0x10, 0x10, 0x18, 0x24, 0x24, 0x24, 0x24};
	GLubyte z[] = {0x00, 0x00, 0x3C, 0x20, 0x10, 0x08, 0x04, 0x3C};

	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

	glColor4d(0.9, 0, 0, 0.7);
	glRasterPos3i(24, 2, 2);
	glBitmap(8, 8, 0.0, 0.0, 0.0, 0.0, x);
	glBegin(GL_LINES);
	glVertex3f(  0,  0,  0);
	glVertex3f( 24,  0,  0);
	glEnd();

	glColor4d(0, 0.9, 0, 0.7);
	glRasterPos3i(2, 24, 2);
	glBitmap(8, 8, 0.0, 0.0, 0.0, 0.0, y);
	glBegin(GL_LINES);
	glVertex3f(  0,  0,  0);
	glVertex3f(  0, 24,  0);
	glEnd();

	glColor4d(0, 0, 0.9, 0.7);
	glRasterPos3i(2, 2, 24);
	glBitmap(8, 8, 0.0, 0.0, 0.0, 0.0, z);
	glBegin(GL_LINES);
	glVertex3f(  0,  0,  0);
	glVertex3f(  0,  0, 24);
	glEnd();
}

void srgDrawCoordinate_axis_basic(float size)
{
	// ..1..1..    ..1..1..    ..1111..     
	// ..1..1..    ..1..1..    .....1..     
	// ...11...    ..1..1..    ....1...     
	// ...11...    ..1..1..    ...1....     
	// ..1..1..    ...11...    ..1.....     
	// ..1..1..    ...1....    ..1111..     
	// ........    ...1....    ........     
	// ........    .11.....    ........     

	GLubyte x[] = {0x00, 0x00, 0x24, 0x24, 0x18, 0x18, 0x24, 0x24};
	GLubyte y[] = {0x60, 0x10, 0x10, 0x18, 0x24, 0x24, 0x24, 0x24};
	GLubyte z[] = {0x00, 0x00, 0x3C, 0x20, 0x10, 0x08, 0x04, 0x3C};

	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

	glColor4d(0.9, 0, 0, 0.7);
	glRasterPos3f(size, 0.1f, 0.1f);
	glBitmap(8, 8, 0.0, 0.0, 0.0, 0.0, x);
	glBegin(GL_LINES);
	glVertex3f(  0,  0,  0);
	glVertex3f( size,  0,  0);
	glEnd();

	glColor4d(0, 0.9, 0, 0.7);
	glRasterPos3f(0.1f, size, 0.1f);
	glBitmap(8, 8, 0.0, 0.0, 0.0, 0.0, y);
	glBegin(GL_LINES);
	glVertex3f(  0,  0,  0);
	glVertex3f(  0, size,  0);
	glEnd();

	glColor4d(0, 0, 0.9, 0.7);
	glRasterPos3f(0.1f, 0.1f, size);
	glBitmap(8, 8, 0.0, 0.0, 0.0, 0.0, z);
	glBegin(GL_LINES);
	glVertex3f(  0,  0,  0);
	glVertex3f(  0,  0, size);
	glEnd();
}

void srgDrawCameraFocus( void )
{
	glutSolidSphere(1.0f, 8, 8);
}

void srgDrawRevoluteJoint(DWORD dwFlags, double Radius, int Slices, int Stacks)
{
	
}

void srgDrawPrismaticJoint(DWORD dwFlags, double Radius, int Slices, int Stacks)
{

}

void srgDrawUniversalJoint(DWORD dwFlags, double Radius, int Slices, int Stacks)
{

}

void srgDrawWeldJoint(DWORD dwFlags, double Radius, int Slices, int Stacks)
{
	srgDrawSphere(dwFlags, Radius, Slices, Stacks);
}

