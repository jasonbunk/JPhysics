
#include "stdafx.h"
#include "Robotics/DoublePendulumOnCart.h"
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
	positions[1].x == p1.theta
	positions[1].y == p2.theta

	velocities[0].x == cart.vel.x
	velocities[1].x == p1.omega == theta-dot
	velocities[1].y == p2.omega == theta-dot
	*/

	double_pendulum_on_cart::double_pendulum_on_cart()
	{
		myEntSystem = NULL;
		add_this_vel_to_cart_next_frame = 0.0;
		positions.resize(2);
		velocities.resize(2);

		cart_bounds_plusminus_x = -1.0;

		cart_kvel = 0.0;
		cart_mass = 2.0;
		p1_len = 2.5;
		p1_kang = 0.0;
		p2_len = 3.5;
		p1_kang = 0.0;

		drawn_cart_width = 2.0;
		drawn_cart_height = 1.0;
		drawn_bob1_diameter = 0.45;
		drawn_bob2_diameter = 0.55;

		p1_bob_m = ONE_FOURTH_PI*drawn_bob1_diameter*drawn_bob1_diameter*(cart_mass/(drawn_cart_width*drawn_cart_height));
		p1_bob_m = ONE_FOURTH_PI*drawn_bob2_diameter*drawn_bob2_diameter*(cart_mass/(drawn_cart_width*drawn_cart_height));
	}

	void double_pendulum_on_cart::draw()
	{
		point bob1_cart, bob2_cart;
		bob1_cart.x = positions[0].x + (p1_len * cos(positions[1].x));
		bob1_cart.y = positions[0].y + (p1_len * sin(positions[1].x));
		bob2_cart.x = bob1_cart.x + (p2_len * cos(positions[1].y));
		bob2_cart.y = bob1_cart.y + (p2_len * sin(positions[1].y));

		glBegin(GL_LINES);
		glColor3ub(color[0],color[1],color[2]);
		//pendulum1
		GLVERT2(positions[0]);
		GLVERT2(bob1_cart);
		//pendulum2
		GLVERT2(bob1_cart);
		GLVERT2(bob2_cart);

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
		GLCIRCLE2D(bob1_cart, drawn_bob1_diameter*0.5, 12);
		glEnd();
		glBegin(GL_LINE_STRIP);
		glColor3ub(color[0],color[1],color[2]);
		GLCIRCLE2D(bob2_cart, drawn_bob2_diameter*0.5, 12);
		glEnd();
	}

	void double_pendulum_on_cart::drawWithOffset(const point& offset, const point& aspectStretch)
	{
		std::cout<<"double_pendulum_on_cart -- don't use drawWithOffset()"<<std::endl;
	}

	bool double_pendulum_on_cart::update(JPHYS_FLOAT_UNIT frametime)
	{
		if(myEntSystem==NULL)
			return true;

		integrate::RK4(this, frametime, &positions, &velocities);
		add_this_vel_to_cart_next_frame = 0.0;

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

	void double_pendulum_on_cart::applyImpulse(const point& Location, const point& Impulse)
	{
		std::cout<<"not for gaming"<<std::endl;
	}

	/*
	positions[0] == cart in cartesian (y is fixed)
	positions[1].x == p1.theta
	positions[1].y == p2.theta

	velocities[0].x == cart.vel.x
	velocities[1].x == p1.omega == theta-dot
	velocities[1].y == p2.omega == theta-dot
	*/

	void double_pendulum_on_cart::f( std::vector<point> *POS, std::vector<point> *VEL, JPHYS_FLOAT_UNIT deltaT )
	{
		JPHYS_FLOAT_UNIT th1 = ((*POS)[1].x + ONE_HALF_PI);
		JPHYS_FLOAT_UNIT th2 = ((*POS)[1].y + ONE_HALF_PI);
		
		//depends on th1 and th2 but not on cartx
		JPHYS_FLOAT_UNIT denompart = p1_bob_m*p1_bob_m + 2*cart_mass*p1_bob_m + cart_mass*p2_bob_m + p1_bob_m*p2_bob_m
									- cart_mass*p2_bob_m*cos(2*(th1-th2)) - p1_bob_m*p2_bob_m*cos(2*th1) - p1_bob_m*p1_bob_m*cos(2*th1);
		
		(*VEL)[0].y = 0;
		(*POS)[0] = (*VEL)[0];
		(*POS)[1] = (*VEL)[1];
		
		double gggg = (-1*myEntSystem->gravity.y);

		/*(*VEL)[0].x = (-2*p1_len*p1_bob_m*sin(th1))*(p1_bob_m + p2_bob_m)*
						;
		(*VEL)[0].x /= denompart;


		(*VEL)[1].x = -p1_bob_m*p1_len*((*POS)[1].x*(*POS)[1].x)*sinth*costh
				- (cart_mass+p1_bob_m)*(-1.0*myEntSystem->gravity.y)*sinth
				+ cart_kvel*(*POS)[0].x*costh
				- (1.0 + (cart_mass / p1_bob_m))*(pendulum_kang/p1_len)*(*POS)[1].x
				- add_this_vel_to_cart_next_frame*costh;
		(*VEL)[1].x /= (denompart * p1_len);*/




		(*POS)[0] *= deltaT;
		(*VEL)[0] *= deltaT;
		(*POS)[1] *= deltaT;
		(*VEL)[1] *= deltaT;
	}




}
