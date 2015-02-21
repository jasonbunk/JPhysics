#ifndef ____JPHYSICS_DCMOTOR222_PENDULUM_AND_CART_H_____
#define ____JPHYSICS_DCMOTOR222_PENDULUM_AND_CART_H_____



#include "../exports.h"
#include "../Templates.h"
#include "../Entities.h"
#include "../EntitySystem.h"
#include <vector>


namespace phys
{


class iphys_dcmotor22_pendcart : public IPhysObject
{
public:
	bool hasCrashedIntoSides;
	
	
	//pendulum params
	JPHYS_FLOAT_UNIT  m;   //total mass of pendulum
	JPHYS_FLOAT_UNIT  Ipc; //rotational inertia of pendulum about its center-of-mass
	JPHYS_FLOAT_UNIT  l;   //distance from attachment point to pendulum center-of-mass
	JPHYS_FLOAT_UNIT  kp;  //angular drag constant (~force -kp*omega)
	
	//cart params
	JPHYS_FLOAT_UNIT  MC;   //mass of cart + Imotor/rmotor^2
	JPHYS_FLOAT_UNIT  kc;  //linear drag constant (includes both cart and motor friction)
	
	//global params
	JPHYS_FLOAT_UNIT g;
	
	JPHYS_FLOAT_UNIT  given_control_force_u;
	
    JPHYS_FLOAT_UNIT  cart_bounds_plusminus_x;
    
	JPHYS_FLOAT_UNIT  drawn_motorBeltDriverRadius;
	JPHYS_FLOAT_UNIT  drawn_motor_position;
	JPHYS_FLOAT_UNIT  drawn_cart_width;
	JPHYS_FLOAT_UNIT  drawn_cart_height;
	JPHYS_FLOAT_UNIT  drawn_bob_diameter;
	
	
	JPHYS_FLOAT_UNIT  get__theta() {return positions[0].y;}
	JPHYS_FLOAT_UNIT  get__omega() {return velocities[0].y;}
	JPHYS_FLOAT_UNIT  get__cartx() {return positions[0].x;}
	JPHYS_FLOAT_UNIT  get__cartvel() {return velocities[0].x;}
	JPHYS_FLOAT_UNIT  set__theta(JPHYS_FLOAT_UNIT newval) {positions[0].y = newval;}
	JPHYS_FLOAT_UNIT  set__omega(JPHYS_FLOAT_UNIT newval) {velocities[0].y = newval;}
	JPHYS_FLOAT_UNIT  set__cartx(JPHYS_FLOAT_UNIT newval) {positions[0].x = newval;}
	JPHYS_FLOAT_UNIT  set__cartvel(JPHYS_FLOAT_UNIT newval) {velocities[0].x = newval;}
	
	
	JPHYSICS_API iphys_dcmotor22_pendcart();
	
	JPHYSICS_API void direct_update(JPHYS_FLOAT_UNIT frametime);
	
	virtual void f( std::vector<point> *POS, std::vector<point> *VEL, JPHYS_FLOAT_UNIT deltaT );
};




class dcmotor22_pendcart : public IEntityOld
{
	bool hasChangedColors;
	unsigned char originalPrecrashColorSaved[3];
public:
	iphys_dcmotor22_pendcart * myPhysics;
	
	
	JPHYSICS_API dcmotor22_pendcart();
	
	
	
	JPHYS_FLOAT_UNIT  get__theta() {return myPhysics->positions[0].y;}
	JPHYS_FLOAT_UNIT  get__omega() {return myPhysics->velocities[0].y;}
	JPHYS_FLOAT_UNIT  get__cartx() {return myPhysics->positions[0].x;}
	JPHYS_FLOAT_UNIT  get__cartvel() {return myPhysics->velocities[0].x;}
	JPHYS_FLOAT_UNIT  set__theta(JPHYS_FLOAT_UNIT newval) {myPhysics->positions[0].y = newval;}
	JPHYS_FLOAT_UNIT  set__omega(JPHYS_FLOAT_UNIT newval) {myPhysics->velocities[0].y = newval;}
	JPHYS_FLOAT_UNIT  set__cartx(JPHYS_FLOAT_UNIT newval) {myPhysics->positions[0].x = newval;}
	JPHYS_FLOAT_UNIT  set__cartvel(JPHYS_FLOAT_UNIT newval) {myPhysics->velocities[0].x = newval;}
	
	
//-- IEntityOld required overrides:

	JPHYSICS_API virtual void draw();

	JPHYSICS_API virtual void drawWithOffset(const point& offset, const point& aspectStretch);

	JPHYSICS_API virtual bool update(JPHYS_FLOAT_UNIT frametime);

	JPHYSICS_API virtual void applyImpulse(const point& Location, const point& Impulse);
	
	virtual void f( std::vector<point> *POS, std::vector<point> *VEL, JPHYS_FLOAT_UNIT deltaT );
};




}



#endif
