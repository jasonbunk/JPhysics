#include "stdafx.h"
#include "Robotics/DCMotor_Cart.h"
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
	positions[0] == cart in cartesian (y is fixed)
	velocities[0].x == cart.vel.x
	*/

	dcmotor_with_cart::dcmotor_with_cart()
	{
		myEntSystem = NULL;
		given_pwm_delta = 0.0;
		positions.resize(1);
		velocities.resize(1);
		
		Imot = 0.5*0.01*(0.01*0.01);
		stalltorque = 1.0; // ???
		motorMaxRadialSpeedUnencumbered = 10.0; // ????
		motorBeltDriverRadius = 0.01;
		
		drawn_motor_position = -5.0;
		cart_bounds_plusminus_x = -1.0;

		cart_kvel = 0.0;
		cart_mass = 2.0;

		drawn_cart_width = 2.0;
		drawn_cart_height = 1.0;

	}

	void dcmotor_with_cart::draw()
	{
		glBegin(GL_LINES);
		glColor3ub(color[0],color[1],color[2]);
		
		GLVERT2(positions[0] + point(drawn_cart_width*0.5, drawn_cart_height*0.5));
		GLVERT2(positions[0] + point(drawn_cart_width*(-0.5), drawn_cart_height*0.5));
		
		GLVERT2(positions[0] + point(drawn_cart_width*(-0.5), drawn_cart_height*0.5));
		GLVERT2(positions[0] + point(drawn_cart_width*(-0.5), drawn_cart_height*(-0.5)));
		
		GLVERT2(positions[0] + point(drawn_cart_width*(-0.5), drawn_cart_height*(-0.5)));
		GLVERT2(positions[0] + point(drawn_cart_width*0.5, drawn_cart_height*(-0.5)));
		
		GLVERT2(positions[0] + point(drawn_cart_width*0.5, drawn_cart_height*(-0.5)));
		GLVERT2(positions[0] + point(drawn_cart_width*0.5, drawn_cart_height*0.5));
		glEnd();
		
		glBegin(GL_LINE_STRIP);
		glColor3ub(color[0],color[1],color[2]);
		GLCIRCLE2D(phys::point(drawn_motor_position, 0.0), motorBeltDriverRadius, 5);
		glEnd();
	}

	void dcmotor_with_cart::drawWithOffset(const point& offset, const point& aspectStretch)
	{
		std::cout<<"drawWithOffset() --- dont use this!!!!!!!!!"<<std::endl;
	}
	
	bool dcmotor_with_cart::update(JPHYS_FLOAT_UNIT frametime)
	{
		if(myEntSystem==NULL)
			return true;
		
		integrate::RK4(this, frametime, &positions, &velocities);
		
		if(cart_bounds_plusminus_x > 0.0) {
			if(positions[0].x > cart_bounds_plusminus_x) {
				positions[0].x = cart_bounds_plusminus_x;
				velocities[0].x = 0.0;
			} else if(positions[0].x < (-1.0*cart_bounds_plusminus_x)) {
				positions[0].x = (-1.0*cart_bounds_plusminus_x);
				velocities[0].x = 0.0;
			}
		}
		
		CenterPos = positions[0];
		return false;
	}

	void dcmotor_with_cart::applyImpulse(const point& Location, const point& Impulse)
	{
		std::cout<<"dcmotor_with_cart::applyImpulse() does nothing"<<std::endl;
	}
	
	/*
	positions[0] == cart in cartesian (y is fixed)
	velocities[0].x == cart.vel.x
	*/
	
	void dcmotor_with_cart::f( std::vector<point> *POS, std::vector<point> *VEL, JPHYS_FLOAT_UNIT deltaT )
	{
		(*POS)[0].y = 0.0;
		(*VEL)[0].y = 0.0;
		
		double originalPosX = (*POS)[0].x;
		
		(*POS)[0].x = (*VEL)[0].x;
		
		(*VEL)[0].x = (1.0/(cart_mass + (Imot/(motorBeltDriverRadius*motorBeltDriverRadius))))
				* (
				(-(*POS)[0].x*(cart_kvel + stalltorque/(motorBeltDriverRadius*motorBeltDriverRadius*motorMaxRadialSpeedUnencumbered)))
				+ (stalltorque * given_pwm_delta / motorBeltDriverRadius)
				);
		
		/*(*VEL)[0].x = pendulum_mass_of_bob*pendulum_length*((*POS)[1].x*(*POS)[1].x)*sinth
			+ pendulum_mass_of_bob*(-1.0*myEntSystem->gravity.y)*sinth*costh
			- cart_kvel*(*POS)[0].x
			+ (pendulum_kang/pendulum_length)*(*POS)[1].x*costh
			+ add_this_vel_to_cart_next_frame;
		(*VEL)[0].x /= denompart;

		(*VEL)[1].x = -pendulum_mass_of_bob*pendulum_length*((*POS)[1].x*(*POS)[1].x)*sinth*costh
				- (cart_mass+pendulum_mass_of_bob)*(-1.0*myEntSystem->gravity.y)*sinth
				+ cart_kvel*(*POS)[0].x*costh
				- (1.0 + (cart_mass / pendulum_mass_of_bob))*(pendulum_kang/pendulum_length)*(*POS)[1].x
				- add_this_vel_to_cart_next_frame*costh;
		(*VEL)[1].x /= (denompart * pendulum_length);*/
		
		
		(*POS)[0] *= deltaT;
		(*VEL)[0] *= deltaT;
	}




}
