#include "stdafx.h"
#include "Robotics/DCMotor2_PendCart.h"
#include "Integrators.h"
#include "mathTools.h"
#include "GLVertexes.h"


namespace phys
{
using namespace mathTools; //from math_constants.h
using namespace drawing; //from GLVertexes.h

//=======================================================================================================================================
//=======================================================================================================================================
//=======================================================================================================================================
	/*
	positions[0].x == cart x in cartesian (cart y == 0)
	positions[0].y == pendulum.theta
	
	velocities[0].x == cart.vel.x
	velocities[0].y == pendulum.omega == theta-dot
	*/

	iphys_dcmotor22_pendcart::iphys_dcmotor22_pendcart()
	{
		positions.resize(1);
		velocities.resize(1);
		hasCrashedIntoSides = false;
		
		
		m  =-100.0;   //total mass of pendulum
		Ipc=-100.0; //rotational inertia of pendulum about its center-of-mass
		l  =-100.0;   //distance from attachment point to pendulum center-of-mass
		kp =-100.0;  //angular drag constant (~force -kp*omega)
		MC =-100.0;   //mass of cart + Imotor/rmotor^2
		kc =-100.0;  //linear drag constant (includes both cart and motor friction)
		g  = -100.0; //gravitational constant (usually +9.81)
		
		given_control_force_u = 0.0;
		
		cart_bounds_plusminus_x=-1.0;
		
		drawn_motorBeltDriverRadius=-1.0;
		drawn_motor_position=-1.0;
		drawn_cart_width=-1.0;
		drawn_cart_height=-1.0;
		drawn_bob_diameter=-1.0;
	}
	
	dcmotor22_pendcart::dcmotor22_pendcart()
	{
		myEntSystem = NULL;
		myPhysics = new iphys_dcmotor22_pendcart();
		
		hasChangedColors = false;
		originalPrecrashColorSaved[0] = 0;
		originalPrecrashColorSaved[1] = 0;
		originalPrecrashColorSaved[2] = 0;
	}

	void dcmotor22_pendcart::draw()
	{
		phys::point last_drawn_pendulum_bob_pos_cartesian;
		last_drawn_pendulum_bob_pos_cartesian.x = get__cartx() + (myPhysics->l * cos(get__theta() + ONE_HALF_PI));
		last_drawn_pendulum_bob_pos_cartesian.y =                (myPhysics->l * sin(get__theta() + ONE_HALF_PI));
		
		glColor3ub(color[0],color[1],color[2]);
		
		glBegin(GL_LINES);
		//pendulum
		GLVERT2(point(get__cartx(), 0.0));
		GLVERT2(last_drawn_pendulum_bob_pos_cartesian);
		
		GLVERT2(point(get__cartx() + myPhysics->drawn_cart_width*0.5, myPhysics->drawn_cart_height*0.5));
		GLVERT2(point(get__cartx() + myPhysics->drawn_cart_width*(-0.5), myPhysics->drawn_cart_height*0.5));
		
		GLVERT2(point(get__cartx() + myPhysics->drawn_cart_width*(-0.5), myPhysics->drawn_cart_height*0.5));
		GLVERT2(point(get__cartx() + myPhysics->drawn_cart_width*(-0.5), myPhysics->drawn_cart_height*(-0.5)));
		
		GLVERT2(point(get__cartx() + myPhysics->drawn_cart_width*(-0.5), myPhysics->drawn_cart_height*(-0.5)));
		GLVERT2(point(get__cartx() + myPhysics->drawn_cart_width*0.5, myPhysics->drawn_cart_height*(-0.5)));
		
		GLVERT2(point(get__cartx() + myPhysics->drawn_cart_width*0.5, myPhysics->drawn_cart_height*(-0.5)));
		GLVERT2(point(get__cartx() + myPhysics->drawn_cart_width*0.5, myPhysics->drawn_cart_height*0.5));
		glEnd();
		
		glBegin(GL_LINE_STRIP);
		GLCIRCLE2D(last_drawn_pendulum_bob_pos_cartesian, myPhysics->drawn_bob_diameter*0.5, 12);
		glEnd();
		
		glColor3ub(color[0],color[1],color[2]);
		glBegin(GL_LINE_STRIP);
		GLCIRCLE2D(point(myPhysics->drawn_motor_position, 0.0), myPhysics->drawn_motorBeltDriverRadius, 5);
		glEnd();
		
		glBegin(GL_LINES);
		GLVERT2(point(myPhysics->drawn_motor_position, 0.0));
		GLVERT2(point(myPhysics->drawn_motor_position + myPhysics->drawn_bob_diameter*cos(get__cartx() / myPhysics->drawn_motorBeltDriverRadius), myPhysics->drawn_bob_diameter*sin(get__cartx() / myPhysics->drawn_motorBeltDriverRadius)));
		glEnd();
		
		glBegin(GL_LINES);
		GLVERT2(point(myPhysics->cart_bounds_plusminus_x + myPhysics->drawn_cart_width*0.5, -1.0));
		GLVERT2(point(myPhysics->cart_bounds_plusminus_x + myPhysics->drawn_cart_width*0.5,  1.0));
		GLVERT2(point(-myPhysics->cart_bounds_plusminus_x - myPhysics->drawn_cart_width*0.5,-1.0));
		GLVERT2(point(-myPhysics->cart_bounds_plusminus_x - myPhysics->drawn_cart_width*0.5, 1.0));
		glEnd();
	}

	void dcmotor22_pendcart::drawWithOffset(const point& offset, const point& aspectStretch)
	{
		std::cout<<"drawWithOffset() --- dont use this!!!!!!!!!"<<std::endl;
	}
	
	void iphys_dcmotor22_pendcart::direct_update(JPHYS_FLOAT_UNIT frametime)
	{
		integrate::RK4(this, frametime, &positions, &velocities);
		
		if(cart_bounds_plusminus_x > 0.0) {
			if(positions[0].x > cart_bounds_plusminus_x) {
				positions[0].x = cart_bounds_plusminus_x;
				velocities[0].x = 0.0;
				hasCrashedIntoSides = true;
			} else if(positions[0].x < (-1.0*cart_bounds_plusminus_x)) {
				positions[0].x = (-1.0*cart_bounds_plusminus_x);
				velocities[0].x = 0.0;
				hasCrashedIntoSides = true;
			}
		}
	}
	
	bool dcmotor22_pendcart::update(JPHYS_FLOAT_UNIT frametime)
	{
		if(myEntSystem==NULL)
			return true;
		
		myPhysics->g = (-1.0*myEntSystem->gravity.y);
		myPhysics->direct_update(frametime);
		
		if(myPhysics->hasCrashedIntoSides && hasChangedColors == false) {
			hasChangedColors = true;
			originalPrecrashColorSaved[0] = color[0];
			originalPrecrashColorSaved[1] = color[1];
			originalPrecrashColorSaved[2] = color[2];
			color[0] = (unsigned char)(((int)255) - ((int)color[0]));
			color[1] = (unsigned char)(((int)255) - ((int)color[1]));
			color[2] = (unsigned char)(((int)255) - ((int)color[2]));
		}
		
		//CenterPos = positions[0];
		return false;
	}
	
	void dcmotor22_pendcart::applyImpulse(const point& Location, const point& Impulse)
	{
		std::cout<<"dcmotor22_pendcart::applyImpulse() does nothing"<<std::endl;
	}
	
	/*
	positions[0].x == cart x in cartesian (cart y == 0)
	positions[0].y == pendulum.theta
	
	velocities[0].x == cart.vel.x
	velocities[0].y == pendulum.omega == theta-dot
	*/
	
	void iphys_dcmotor22_pendcart::f( std::vector<point> *POS, std::vector<point> *VEL, JPHYS_FLOAT_UNIT deltaT )
	{
		double _ST_THETA = (*POS)[0].y;
		double _ST_OMEGA = (*VEL)[0].y;
		double _ST_CARTXDOT = (*VEL)[0].x;
		
		(*POS)[0] = (*VEL)[0];
		
		const double IpcPl2m = (Ipc+l*l*m);
		const double denompart = 2.0*Ipc*(m+MC) + l*l*m*(m + 2.0*MC - m*cos(2.0*_ST_THETA));
		const double kcrvmPWMtsplmrw2sinth = kc*_ST_CARTXDOT + l*m*_ST_OMEGA*_ST_OMEGA*sin(_ST_THETA) - given_control_force_u;
		
		double thetadotdot_accel = -2.0*l*(-kp*m*_ST_OMEGA*cos(_ST_THETA)*cos(_ST_THETA) + (m+MC)*(kp*_ST_OMEGA - g*m*sin(_ST_THETA)) + m*cos(_ST_THETA)*kcrvmPWMtsplmrw2sinth) / denompart;
		double xdotdot_accel = 2.0*(cos(_ST_THETA)*(Ipc*kp*_ST_OMEGA + g*l*l*m*m*sin(_ST_THETA)) - IpcPl2m*kcrvmPWMtsplmrw2sinth) / denompart;
		
		(*VEL)[0].x = xdotdot_accel;
		(*VEL)[0].y = thetadotdot_accel;
		
		(*POS)[0] *= deltaT;
		(*VEL)[0] *= deltaT;
	}
	
	
	void dcmotor22_pendcart::f( std::vector<point> *POS, std::vector<point> *VEL, JPHYS_FLOAT_UNIT deltaT )
	{
		myPhysics->f(POS, VEL, deltaT);
	}

}
