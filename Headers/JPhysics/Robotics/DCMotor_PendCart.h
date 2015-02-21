#ifndef ____JPHYSICS_DCMOTOR_PENDULUM_AND_CART_H_____
#define ____JPHYSICS_DCMOTOR_PENDULUM_AND_CART_H_____



#include "../exports.h"
#include "../Templates.h"
#include "../Entities.h"
#include "../EntitySystem.h"
#include <vector>


namespace phys
{


class dcmotor_pendcart : public IEntityOld	//don't export the entire class (stl vectors won't work), just its functions
{
	bool hasCrashedIntoSides;
	unsigned char originalPrecrashColorSaved[3];
public:
	JPHYS_FLOAT_UNIT given_pwm_delta;
	
	JPHYS_FLOAT_UNIT Imot;
	JPHYS_FLOAT_UNIT stalltorque;
	JPHYS_FLOAT_UNIT motorBeltDriverRadius;

    JPHYS_FLOAT_UNIT cart_bounds_plusminus_x;

	JPHYS_FLOAT_UNIT cart_mass;
	JPHYS_FLOAT_UNIT cart_kvel;
	JPHYS_FLOAT_UNIT pendulum_length_center_of_mass;
	JPHYS_FLOAT_UNIT pendulum_I_about_CM;
	JPHYS_FLOAT_UNIT pendulum_mass;
	JPHYS_FLOAT_UNIT pendulum_kang;
	
	JPHYS_FLOAT_UNIT drawn_motor_position;
	JPHYS_FLOAT_UNIT drawn_cart_width;
	JPHYS_FLOAT_UNIT drawn_cart_height;
	JPHYS_FLOAT_UNIT drawn_bob_diameter;
	
	
	JPHYS_FLOAT_UNIT get__theta() {return positions[0].y;}
	JPHYS_FLOAT_UNIT get__omega() {return velocities[0].y;}
	JPHYS_FLOAT_UNIT get__cartx() {return positions[0].x;}
	JPHYS_FLOAT_UNIT get__cartvel() {return velocities[0].x;}

	JPHYS_FLOAT_UNIT set__theta(JPHYS_FLOAT_UNIT newval) {positions[0].y = newval;}
	JPHYS_FLOAT_UNIT set__omega(JPHYS_FLOAT_UNIT newval) {velocities[0].y = newval;}
	JPHYS_FLOAT_UNIT set__cartx(JPHYS_FLOAT_UNIT newval) {positions[0].x = newval;}
	JPHYS_FLOAT_UNIT set__cartvel(JPHYS_FLOAT_UNIT newval) {velocities[0].x = newval;}
	
	
	JPHYSICS_API dcmotor_pendcart();


//-- IEntityOld required overrides:

	JPHYSICS_API virtual void draw();

	JPHYSICS_API virtual void drawWithOffset(const point& offset, const point& aspectStretch);

	JPHYSICS_API virtual bool update(JPHYS_FLOAT_UNIT frametime);

	JPHYSICS_API virtual void applyImpulse(const point& Location, const point& Impulse);

	virtual void f( std::vector<point> *POS, std::vector<point> *VEL, JPHYS_FLOAT_UNIT deltaT );

};




}



#endif
