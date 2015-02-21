//-----------------------------------------------------------------
// July 27, 2012
// Jason Bunk
// Rectangles.h
//
// Rectangular prisms, grids of points held in place by linear springs
// to keep them at proper distance, and diagonally-connecting springs
// to prevent twisting (to a point; then permanent deformation).
//-----------------------------------------------------------------

#ifndef __JPHYSICS__RECTANGLE_H__
#define __JPHYSICS__RECTANGLE_H__

#include "exports.h"
#include "Templates.h"
#include "Entities.h"
#include "EntitySystem.h"
#include <vector>

namespace phys
{

class ent_rectangle : public IEntityOld	//don't export the entire class (stl vectors won't work), just its functions
{
private:
		int N,M;

		std::vector<point> TempVelocities;

		void UpdateCenterPos();
public:

		JPHYS_FLOAT_UNIT mass;
		JPHYS_FLOAT_UNIT kpos, eqdist;
		JPHYS_FLOAT_UNIT friction;

		bool drawHollow;
		bool draw_corner_dots;
		bool calculate_diagonal_forces;
		bool edges_are_fixed;
		bool left_edge_only_fixed;
		bool move_only_on_x_axis;
		bool affected_by_gravity;


	JPHYSICS_API inline int GetN() {return N;}
	JPHYSICS_API inline int GetM() {return M;}
		
	JPHYSICS_API ent_rectangle();
	JPHYSICS_API ent_rectangle(JPHYS_FLOAT_UNIT NN, JPHYS_FLOAT_UNIT MM, EntitySystem* entSys, SpawnParams_Soft params);
	JPHYSICS_API ent_rectangle(JPHYS_FLOAT_UNIT NN, JPHYS_FLOAT_UNIT MM, JPHYS_FLOAT_UNIT framETime, EntitySystem* entSys, SpawnParams_Soft params);
	
//-- IEntityOld required overrides:

	JPHYSICS_API virtual void draw();
	
	JPHYSICS_API virtual void drawWithOffset(const point& offset, const point& aspectStretch);

	JPHYSICS_API virtual bool update(JPHYS_FLOAT_UNIT frametime);

	JPHYSICS_API virtual void applyImpulse(const point& Location, const point& Impulse);

	virtual void f( std::vector<point> *POS, std::vector<point> *VEL, JPHYS_FLOAT_UNIT deltaT );

};

}

#endif
