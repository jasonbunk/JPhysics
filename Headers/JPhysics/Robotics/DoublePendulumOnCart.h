#ifndef __JPHYSICS_DOUBLE_PENDULUM_ON_CART_H__
#define __JPHYSICS_DOUBLE_PENDULUM_ON_CART_H__

#include "../exports.h"
#include "../Templates.h"
#include "../Entities.h"
#include "../EntitySystem.h"
#include <vector>


namespace phys
{


class double_pendulum_on_cart : public IEntityOld	//don't export the entire class (stl vectors won't work), just its functions
{
public:
	JPHYS_FLOAT_UNIT add_this_vel_to_cart_next_frame;

    JPHYS_FLOAT_UNIT cart_bounds_plusminus_x;

	JPHYS_FLOAT_UNIT cart_mass;
	JPHYS_FLOAT_UNIT cart_kvel;
	
	JPHYS_FLOAT_UNIT p1_len;
	JPHYS_FLOAT_UNIT p1_bob_m;
	JPHYS_FLOAT_UNIT p1_kang;
	
	JPHYS_FLOAT_UNIT p2_len;
	JPHYS_FLOAT_UNIT p2_bob_m;
	JPHYS_FLOAT_UNIT p2_kang;

	JPHYS_FLOAT_UNIT drawn_cart_width;
	JPHYS_FLOAT_UNIT drawn_cart_height;
	JPHYS_FLOAT_UNIT drawn_bob1_diameter;
	JPHYS_FLOAT_UNIT drawn_bob2_diameter;


	JPHYSICS_API double_pendulum_on_cart();


//-- IEntityOld required overrides:

	JPHYSICS_API virtual void draw();

	JPHYSICS_API virtual void drawWithOffset(const point& offset, const point& aspectStretch);

	JPHYSICS_API virtual bool update(JPHYS_FLOAT_UNIT frametime);

	JPHYSICS_API virtual void applyImpulse(const point& Location, const point& Impulse);

	virtual void f( std::vector<point> *POS, std::vector<point> *VEL, JPHYS_FLOAT_UNIT deltaT );

};




}

#endif
