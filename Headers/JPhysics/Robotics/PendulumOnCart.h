#ifndef __PENDULUM_ON_CART_H__
#define __PENDULUM_ON_CART_H__

#include "../exports.h"
#include "../Templates.h"
#include "../Entities.h"
#include "../EntitySystem.h"
#include <vector>


namespace phys
{


class pendulum_on_cart : public IEntityOld	//don't export the entire class (stl vectors won't work), just its functions
{
protected:
    point last_drawn_pendulum_bob_pos_cartesian;
public:
	JPHYS_FLOAT_UNIT add_this_vel_to_cart_next_frame;

    JPHYS_FLOAT_UNIT cart_bounds_plusminus_x;

	JPHYS_FLOAT_UNIT cart_mass;
	JPHYS_FLOAT_UNIT cart_kvel;
	JPHYS_FLOAT_UNIT pendulum_length;
	JPHYS_FLOAT_UNIT pendulum_mass_of_bob;
	JPHYS_FLOAT_UNIT pendulum_kang;

	JPHYS_FLOAT_UNIT drawn_cart_width;
	JPHYS_FLOAT_UNIT drawn_cart_height;
	JPHYS_FLOAT_UNIT drawn_bob_diameter;


	JPHYSICS_API pendulum_on_cart();


//-- IEntityOld required overrides:

	JPHYSICS_API virtual void draw();

	JPHYSICS_API virtual void drawWithOffset(const point& offset, const point& aspectStretch);

	JPHYSICS_API virtual bool update(JPHYS_FLOAT_UNIT frametime);

	JPHYSICS_API virtual void applyImpulse(const point& Location, const point& Impulse);

	virtual void f( std::vector<point> *POS, std::vector<point> *VEL, JPHYS_FLOAT_UNIT deltaT );

};




}

#endif
