//-----------------------------------------------------------------
// September 2, 2012
// Jason Bunk
// Fluid_Old.h
//
// A fluid, defined by a large collection of individual point masses,
// each independently moving but influenced by other points.
//-----------------------------------------------------------------

#ifndef __JPHYSICS__FLUID_H__
#define __JPHYSICS__FLUID_H__

#include "exports.h"
#include "Templates.h"
#include "Entities.h"
#include "EntitySystem.h"
#include <vector>


namespace phys
{

class ent_fluid : public IEntity	//don't export the entire class (stl vectors won't work), just its functions
{
protected:
		int N;

		char checkBounds;

public:

		JPHYS_FLOAT_UNIT kcohesion, krepulsion, massEach, distanceFactor;
		

	JPHYSICS_API INLINE int GetN() {return N;}

	JPHYSICS_API ent_fluid();

	JPHYSICS_API ent_fluid(SpawnParams_Fluid params);

	JPHYSICS_API void add_element(SpawnParams params);
	
	JPHYSICS_API void deleteAllElements();

//-- IEntity required overrides:
	
	//updates positions and velocities
	JPHYSICS_API virtual void update(JPHYS_FLOAT_UNIT frametime);

	//draws object using OpenGl
	JPHYSICS_API virtual void draw();

	//draws positions with an offset and stretch. since it has to calculate the offset each time, this is less efficient.
	JPHYSICS_API virtual void drawWithOffset(const point& offset, const point& aspectStretch);

	//applies an impulse to any nearby objects (e.g. for mouse/finger touches)
	JPHYSICS_API virtual void applyImpulse(const point& Location, const point& Impulse);
	
	//Check collisions, esp. against another moving object nearby
	JPHYSICS_API virtual void checkCollisions(const line &hitSurface, const point& offset);

};

}

#endif
