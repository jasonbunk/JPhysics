//-----------------------------------------------------------------
// July 27, 2012
// Jason Bunk
// Rectangles.h
//
// Rectangular prisms, grids of points held in place by linear springs
// to keep them at proper distance, and diagonally-connecting springs
// to prevent twisting (to a point; then permanent deformation).
//-----------------------------------------------------------------

#ifndef ___RECTANGLE_NEW_H__
#define ___RECTANGLE_NEW_H__

#include "exports.h"
#include "Templates.h"
#include "Entities.h"
#include "EntitySystem.h"
#include <vector>

namespace phys
{

class ent_rectangle_new : public IEntity	//don't export the entire class (stl vectors won't work), just its functions
{
private:
		int N,M; //N horizontal, M vertical
		
		char checkBounds;

		std::vector<point> temp_positions;
		std::vector<point> temp_velocities;

		INLINE void UpdateCenterPos();
public:

		JPHYS_FLOAT_UNIT mass;
		JPHYS_FLOAT_UNIT kpos, eqdist;

		char drawHollow;

	//N horizontal
	JPHYSICS_API inline int GetN() {return N;}
	//M vertical
	JPHYSICS_API inline int GetM() {return M;}
		
	JPHYSICS_API ent_rectangle_new();

	JPHYSICS_API ent_rectangle_new(JPHYS_FLOAT_UNIT NN, JPHYS_FLOAT_UNIT MM, EntitySystem* entSys, SpawnParams_Soft params);
	
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

	
	void integRK4(std::vector<point> & POSITIONS, std::vector<point> & VELOCITIES, JPHYS_FLOAT_UNIT hhh);

	inline void motionFunc(std::vector<point> & POSITIONS, std::vector<point> & VELOCITIES);

	/*inline*/ void checkVertexCollisions(int vertex1, int vertex2);

};

}

#endif
