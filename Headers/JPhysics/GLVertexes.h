

#include "mathTools.h"
#include "Templates.h"


#ifndef __JPHYSICS_NO_OPENGL__
#ifndef __JPHYSICS_NO_GL__
#ifndef __JPHYSICS_NOGL__

#ifndef __GL_H__ //defined in gl.h, so only include it once
#include <GL/gl.h>
//#include <GL/glu.h>
#endif

#endif
#endif
#endif


#ifdef GL_LINES //defined in gl.h, so only do these drawing functions if we're using opengl


#ifndef __GL_VERTEXES_H___
#define __GL_VERTEXES_H___

#include <assert.h>
#include <vector>


namespace phys
{
namespace drawing
{
	inline void OpenGLRotate_Euler(JPHYS_FLOAT_UNIT X, JPHYS_FLOAT_UNIT Y, JPHYS_FLOAT_UNIT Z)
	{
		JPHYS_FLOAT_UNIT c1 = cos(X*0.00087266462599716479);
		JPHYS_FLOAT_UNIT c2 = cos(Y*0.00087266462599716479);
		JPHYS_FLOAT_UNIT c3 = cos(Z*0.00087266462599716479); //    (Z * (2.0 * 3.14159 / 180.0))
		JPHYS_FLOAT_UNIT s1 = sin(X*0.00087266462599716479);
		JPHYS_FLOAT_UNIT s2 = sin(Y*0.00087266462599716479);
		JPHYS_FLOAT_UNIT s3 = sin(Z*0.00087266462599716479);

		JPHYS_FLOAT_UNIT x = (s1*c2*c3-c1*s2*s3)*57.2957795130823208768; //     () * 180.0 / 3.14159
		JPHYS_FLOAT_UNIT y = (c1*s2*c3+s1*c2*s3)*57.2957795130823208768;
		JPHYS_FLOAT_UNIT z = (c1*c2*s3-s1*s2*c3)*57.2957795130823208768;
		JPHYS_FLOAT_UNIT w = acos(c1*c2*c3+s1*s2*s3)*114.5915590261646417536; //    () * 360.0 / 3.14159

		//normalize vector, maybe not necessary?
		JPHYS_FLOAT_UNIT mag = sqrt(x*x+y*y+z*z);

		if(mathTools::cmpDoubleToZero(mag) == false)
		{
			x /= mag;
			y /= mag;
			z /= mag;
			glRotated(w,x,y,z);
		}
	}

#if 0
	inline void GLQUAD2(const point& p1, const point& p2, const point& p3, const point& p4)
	{
		glVertex2f(p1.x,p1.y);
		glVertex2f(p2.x,p2.y);
		glVertex2f(p3.x,p3.y);
		glVertex2f(p4.x,p4.y);
	}

	inline void GLQUAD2(JPHYS_FLOAT_UNIT left, JPHYS_FLOAT_UNIT top, JPHYS_FLOAT_UNIT right, JPHYS_FLOAT_UNIT bottom)
	{
		glVertex2f(left,top);
		glVertex2f(right,top);
		glVertex2f(right,bottom);
		glVertex2f(left,bottom);
	}
	
	inline void GLVERT2(const point& pt)
	{
		glVertex2f((pt).x,(pt).y);
	}

	inline void GLVERT2(const point& pt, const point& offset, const point& aspect)
	{
		glVertex2f(((pt).x+offset.x)*aspect.x,((pt).y+offset.y)*aspect.y);
	}

	inline void GLVERT3(const vec3& pt)
	{
		glVertex3f((pt).x, (pt).y, (pt).z);
	}
	inline void GLVERT3(const vec3& pt, const point& offset, const point& aspect)
	{
		glVertex3f(((pt).x+offset.x)*aspect.x, ((pt).y+offset.y)*aspect.y, (pt).z);
	}

	static void GLQUAD_FullTexture(phys::point location, JPHYS_FLOAT_UNIT width, JPHYS_FLOAT_UNIT height)
	{
		glTexCoord2f(0.0, 0.0); glVertex3f(location.x-(width/2.0), location.y-(height/2.0), 0.0);
		glTexCoord2f(0.0, 1.0); glVertex3f(location.x-(width/2.0), location.y+(height/2.0), 0.0);
		glTexCoord2f(1.0, 1.0); glVertex3f(location.x+(width/2.0), location.y+(height/2.0), 0.0);
		glTexCoord2f(1.0, 0.0); glVertex3f(location.x+(width/2.0), location.y-(height/2.0), 0.0);
	}
#else
	inline void GLQUAD2(const point& p1, const point& p2, const point& p3, const point& p4)
	{
		glVertex2d(p1.x,p1.y);
		glVertex2d(p2.x,p2.y);
		glVertex2d(p3.x,p3.y);
		glVertex2d(p4.x,p4.y);
	}

	inline void GLQUAD2(JPHYS_FLOAT_UNIT left, JPHYS_FLOAT_UNIT top, JPHYS_FLOAT_UNIT right, JPHYS_FLOAT_UNIT bottom)
	{
		glVertex2d(left,top);
		glVertex2d(right,top);
		glVertex2d(right,bottom);
		glVertex2d(left,bottom);
	}


	inline void GLVERT2(const std::pair<double,double>& pt) {
		glVertex2d(pt.first, pt.second);
	}
	inline void GLVERT2(const std::vector<double>& pt) {
		assert(pt.size() == 2);
		glVertex2d(pt[0], pt[1]);
	}
	inline void GLVERT2(const point& pt) {
		glVertex2d(pt.x, pt.y);
	}

	inline void GLVERT2(const std::pair<double,double>& pt, double scalar) {
		glVertex2d(pt.first*scalar, pt.second*scalar);
	}
	inline void GLVERT2(const std::vector<double>& pt, double scalar) {
		assert(pt.size() == 2);
		glVertex2d(pt[0]*scalar, pt[1]*scalar);
	}
	inline void GLVERT2(const point& pt, double scalar) {
		glVertex2d(pt.x*scalar, pt.y*scalar);
	}

	inline void GLVERT2(const point& pt, const point& offset, const point& aspect)
	{
		glVertex2d(((pt).x+offset.x)*aspect.x,((pt).y+offset.y)*aspect.y);
	}

	inline void GLVERT3(const vec3& pt)
	{
		glVertex3d((pt).x, (pt).y, (pt).z);
	}
	inline void GLVERT3(const vec3& pt, const point& offset, const point& aspect)
	{
		glVertex3d(((pt).x+offset.x)*aspect.x, ((pt).y+offset.y)*aspect.y, (pt).z);
	}

	static void GLQUAD_FullTexture(phys::point location, JPHYS_FLOAT_UNIT width, JPHYS_FLOAT_UNIT height)
	{
		glTexCoord2d(0.0, 0.0); glVertex3d(location.x-(width/2.0), location.y-(height/2.0), 0.0);
		glTexCoord2d(0.0, 1.0); glVertex3d(location.x-(width/2.0), location.y+(height/2.0), 0.0);
		glTexCoord2d(1.0, 1.0); glVertex3d(location.x+(width/2.0), location.y+(height/2.0), 0.0);
		glTexCoord2d(1.0, 0.0); glVertex3d(location.x+(width/2.0), location.y-(height/2.0), 0.0);
	}

	static void GLCIRCLE2D(point center, JPHYS_FLOAT_UNIT radius, int pts)
	{
		JPHYS_FLOAT_UNIT delta_angle = (mathTools::TWO_PI / ((JPHYS_FLOAT_UNIT)pts));
		JPHYS_FLOAT_UNIT angle = 0.0;
		for(int ii=0; ii<=pts; ii++) {
		    glVertex2d(center.x + radius*cos(angle), center.y + radius*sin(angle));
		    angle += delta_angle;
		}
	}

	static void GLCIRCLE2D(point center, JPHYS_FLOAT_UNIT radius, int pts, const point& offset, const point& aspect)
	{
		JPHYS_FLOAT_UNIT delta_angle = (mathTools::TWO_PI / ((JPHYS_FLOAT_UNIT)pts));
		JPHYS_FLOAT_UNIT angle = 0.0;
		for(int ii=0; ii<=pts; ii++) {
		    glVertex2d((center.x + radius*cos(angle) + offset.x)*aspect.x, (center.y + radius*sin(angle) + offset.y)*aspect.y);
		    angle += delta_angle;
		}
	}
#endif
}
}

#endif

#endif
