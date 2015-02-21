#ifndef ____JPHYSICS_DCMOTOR_CART_H_____
#define ____JPHYSICS_DCMOTOR_CART_H_____



#include "../exports.h"
#include "../Templates.h"
#include "../Entities.h"
#include "../EntitySystem.h"
#include <vector>


namespace phys
{


class dcmotor_with_cart : public IEntityOld	//don't export the entire class (stl vectors won't work), just its functions
{
public:
	JPHYS_FLOAT_UNIT given_pwm_delta;
	JPHYS_FLOAT_UNIT Imot;
	JPHYS_FLOAT_UNIT stalltorque;
	JPHYS_FLOAT_UNIT motorMaxRadialSpeedUnencumbered;
	JPHYS_FLOAT_UNIT motorBeltDriverRadius;

    JPHYS_FLOAT_UNIT cart_bounds_plusminus_x;

	JPHYS_FLOAT_UNIT cart_mass;
	JPHYS_FLOAT_UNIT cart_kvel;
	
	JPHYS_FLOAT_UNIT drawn_motor_position;
	JPHYS_FLOAT_UNIT drawn_cart_width;
	JPHYS_FLOAT_UNIT drawn_cart_height;


	JPHYSICS_API dcmotor_with_cart();


//-- IEntityOld required overrides:

	JPHYSICS_API virtual void draw();

	JPHYSICS_API virtual void drawWithOffset(const point& offset, const point& aspectStretch);

	JPHYSICS_API virtual bool update(JPHYS_FLOAT_UNIT frametime);

	JPHYSICS_API virtual void applyImpulse(const point& Location, const point& Impulse);

	virtual void f( std::vector<point> *POS, std::vector<point> *VEL, JPHYS_FLOAT_UNIT deltaT );

};




}



#endif
