//-----------------------------------------------------------------
// December 29, 2013
// Jason Bunk
// RectangleWithTorque.h
//
// Rectangular prisms, grids of points held in place by linear springs
// to keep them at proper distance, and torqueing angular joints to
// maintain the original shape of the rectangular structure.
//-----------------------------------------------------------------

#ifndef ___RECTANGLE_WITH_TORQUE_H____
#define ___RECTANGLE_WITH_TORQUE_H____

#include "exports.h"
#include "Templates.h"
#include "Entities.h"
#include "EntitySystem.h"
#include <vector>

namespace phys
{

class ent_rectangle_torque : public IEntityOld	//don't export the entire class (stl vectors won't work), just its functions
{
protected:
		int N,M;

		void UpdateCenterPos();
		
		std::vector<point> TempVelocities; //for applying impulses

public:
	JPHYS_FLOAT_UNIT mass, kpos, kang, kvel;	//m = mass; kpos F = -kx; kang T = -k*theta; kvel = constant of drag (b in damping equation F = bv)
	JPHYS_FLOAT_UNIT eqdist, eqang;		//equilibrium distance, equilibrium angle
	
	bool drawHollow;

		
	JPHYSICS_API int GetN() {return N;}
	JPHYSICS_API int GetM() {return M;}
	
	JPHYSICS_API ent_rectangle_torque();
	JPHYSICS_API ent_rectangle_torque(JPHYS_FLOAT_UNIT NN, JPHYS_FLOAT_UNIT MM, EntitySystem* entSys, SpawnParams_Soft params);

	JPHYSICS_API void GiveStartingFlick(JPHYS_FLOAT_UNIT kick_vel);
	
//-- IEntityOld required overrides:

	JPHYSICS_API virtual void draw();
	
	JPHYSICS_API virtual void drawWithOffset(const point& offset, const point& aspectStretch);

	JPHYSICS_API virtual bool update(JPHYS_FLOAT_UNIT frametime);

	JPHYSICS_API virtual void applyImpulse(const point& Location, const point& Impulse);

	virtual void f( std::vector<point> *POS, std::vector<point> *VEL, JPHYS_FLOAT_UNIT deltaT );
};


}
#endif