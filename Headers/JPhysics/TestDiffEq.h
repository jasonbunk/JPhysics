#ifndef __TEST_DIFF_EQ_H__
#define __TEST_DIFF_EQ_H__

#include "exports.h"
#include "Templates.h"
#include "Entities.h"
#include "EntitySystem.h"
#include <vector>


namespace phys
{

class ent_TestDiffEq : public IEntityOld	//don't export the entire class (stl vectors won't work), just its functions
{
public:
	
	JPHYS_FLOAT_UNIT mass;
	JPHYS_FLOAT_UNIT kvel;

	JPHYS_FLOAT_UNIT lastY;

	JPHYSICS_API ent_TestDiffEq();
	
	JPHYSICS_API ent_TestDiffEq(JPHYS_FLOAT_UNIT framETime, EntitySystem* entSys, SpawnParams params);
	
//-- IEntityOld required overrides:
	
	JPHYSICS_API virtual void draw();
	
	JPHYSICS_API virtual void drawWithOffset(const point& offset, const point& aspectStretch);

	JPHYSICS_API virtual bool update(JPHYS_FLOAT_UNIT frametime);

	JPHYSICS_API virtual void applyImpulse(const point& Location, const point& Impulse);

	virtual void f( std::vector<point> *POS, std::vector<point> *VEL, JPHYS_FLOAT_UNIT deltaT );

};

}

#endif