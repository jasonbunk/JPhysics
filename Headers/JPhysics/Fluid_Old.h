//-----------------------------------------------------------------
// September 2, 2012
// Jason Bunk
// Fluid_Old.h
//
// A fluid, defined by a large collection of individual point masses,
// each independently moving but influenced by other points.
//-----------------------------------------------------------------

#ifndef __JPHYSICS_FLUID_DEPRECATED_H__
#define __JPHYSICS_FLUID_DEPRECATED_H__

#include "exports.h"
#include "Templates.h"
#include "Entities.h"
#include "EntitySystem.h"
#include <vector>


namespace phys
{

class ent_fluid_old : public IEntityOld	//don't export the entire class (stl vectors won't work), just its functions
{
protected:
		int N;
public:

		JPHYS_FLOAT_UNIT kpos, kcohesion, krepulsion, massEach, distanceFactor;
		
//////////////////////////////////////////////////////////////////
		
		std::vector<point> startTracePos;
		std::vector<point> currentEdgePos;
		std::vector<bool> tracking;

		JPHYS_FLOAT_UNIT gooThresh, gooSize;
	
public:
	//draws gooey metaballs
	JPHYSICS_API void draw_metaballs(JPHYS_FLOAT_UNIT step, JPHYS_FLOAT_UNIT tolerance);

protected:
	//Return the metaball field's force at point 'pos'.
    JPHYS_FLOAT_UNIT calcForce(point pos);
	
	//Return a normalized (magnitude = 1) normal at point 'pos'.
    point calcNormal(point pos);
	
	//Return a normalized (magnitude = 1) tangent at point 'pos'.
    point calcTangent(point pos);

	 //Step once towards the border of the metaball field, return new coordinates and force at old coordinates.
    JPHYS_FLOAT_UNIT stepOnceTowardsBorder(point &pos);

	//Track the border of the metaball field and return new coordinates.
    point trackTheBorder(point pos);

    point eulerTangent(point pos, JPHYS_FLOAT_UNIT h);

    point rungeKutta2Tangent(point pos, JPHYS_FLOAT_UNIT h);

//////////////////////////////////////////////////////////////////
		

	JPHYSICS_API INLINE int GetN() {return N;}

	JPHYSICS_API ent_fluid_old();

	JPHYSICS_API ent_fluid_old(SpawnParams_Fluid params);

	JPHYSICS_API void add_element(SpawnParams params);
	
	JPHYSICS_API void deleteAllElements();

//-- IEntityOld required overrides:
	
	JPHYSICS_API virtual void draw();
	
	JPHYSICS_API virtual void drawWithOffset(const point& offset, const point& aspectStretch);

	JPHYSICS_API virtual bool update(JPHYS_FLOAT_UNIT frametime);

	JPHYSICS_API virtual void applyImpulse(const point& Location, const point& Impulse);

	JPHYSICS_API virtual void f( std::vector<point> *POS, std::vector<point> *VEL, JPHYS_FLOAT_UNIT deltaT );

};

}

#endif
