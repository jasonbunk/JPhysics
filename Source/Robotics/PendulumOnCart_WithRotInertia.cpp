/*
 * Author: Jason Bunk
 * Web page: http://sites.google.com/site/jasonbunk
 * License: The MIT License (MIT) (Expat)
 * Copyright (c) 2015 Jason Bunk
 */

#include "stdafx.h"
#include "Robotics/PendulumOnCart_WithRotInertia.h"
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
	positions[1].x == pendulum.theta

	velocities[0].x == cart.vel.x
	velocities[1].x == pendulum.omega == theta-dot
	*/

	pendulum_on_cart_with_rot_inertia::pendulum_on_cart_with_rot_inertia()
	{
		myEntSystem = NULL;
		add_this_vel_to_cart_next_frame = 0.0;
		positions.resize(2);
		velocities.resize(2);

		cart_bounds_plusminus_x = -1.0;

		cart_kvel = 0.0;
		cart_mass = 2.0;
		pendulum_kang = 0.0;
		
		pendulum_length_center_of_mass = 2.0;
		pendulum_mass = cart_mass;
		pendulum_I_about_CM = ((1.0/3.0) * pendulum_mass * pendulum_length_center_of_mass * pendulum_length_center_of_mass);
		
		drawn_cart_width = 2.0;
		drawn_cart_height = 1.0;
		drawn_bob_diameter = 0.5;
	}

	void pendulum_on_cart_with_rot_inertia::draw()
	{
		last_drawn_pendulum_bob_pos_cartesian.x = positions[0].x + (pendulum_length_center_of_mass * cos(positions[1].x));
		last_drawn_pendulum_bob_pos_cartesian.y = positions[0].y + (pendulum_length_center_of_mass * sin(positions[1].x));

		glBegin(GL_LINES);
		glColor3ub(color[0],color[1],color[2]);
		//pendulum
		GLVERT2(positions[0]);
		GLVERT2(last_drawn_pendulum_bob_pos_cartesian);

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
		GLCIRCLE2D(last_drawn_pendulum_bob_pos_cartesian, drawn_bob_diameter*0.5, 12);
		glEnd();
	}

	void pendulum_on_cart_with_rot_inertia::drawWithOffset(const point& offset, const point& aspectStretch)
	{
		std::cout<<"DONT USE THIS"<<std::endl;
	}

	bool pendulum_on_cart_with_rot_inertia::update(JPHYS_FLOAT_UNIT frametime)
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

		//integrate::EulerBackwardsFromRK4(this, frametime, &positions, &velocities);

		CenterPos = positions[0];

		return false;
	}

	void pendulum_on_cart_with_rot_inertia::applyImpulse(const point& Location, const point& Impulse)
	{
		std::cout<<"not for gaming"<<std::endl;
	}

	/*
	positions[0] == cart in cartesian (y is fixed)
	positions[1].x == pendulum.theta

	velocities[0].x == cart.vel.x
	velocities[1].x == pendulum.omega == theta-dot
	*/

	void pendulum_on_cart_with_rot_inertia::f( std::vector<point> *POS, std::vector<point> *VEL, JPHYS_FLOAT_UNIT deltaT )
	{
		JPHYS_FLOAT_UNIT sinth = sin((*POS)[1].x + ONE_HALF_PI);
		JPHYS_FLOAT_UNIT costh = cos((*POS)[1].x + ONE_HALF_PI);
		JPHYS_FLOAT_UNIT denompart = ((4.0/3.0)*(cart_mass+pendulum_mass)) - pendulum_mass*costh*costh;
		
		(*VEL)[0].y = 0.0;
		(*VEL)[1].y = 0.0;
		(*POS)[0] = (*VEL)[0];
	//	(*POS)[0].x += add_this_vel_to_cart_next_frame;
		(*POS)[1] = (*VEL)[1];
	//	(*POS)[1].x -= (add_this_vel_to_cart_next_frame*costh);
		
		double IIpL2m = pendulum_I_about_CM + pendulum_mass*pendulum_length_center_of_mass*pendulum_length_center_of_mass;
		double Lmw2sinth = pendulum_length_center_of_mass*pendulum_mass*((*VEL)[1].x * (*VEL)[1].x)*sinth;
		
		double kpwpgLmsinth = pendulum_kang * (*VEL)[1].x - myEntSystem->gravity.y * pendulum_length_center_of_mass * pendulum_mass * sinth;
		
		double umkcvlmw2sinth = add_this_vel_to_cart_next_frame - (cart_kvel*(*VEL)[0].x) + Lmw2sinth;
		
		double Lmcosth = pendulum_length_center_of_mass * pendulum_mass * costh;
		
		double mPM = pendulum_mass + cart_mass;
		
		double denompart_true = IIpL2m*mPM - pendulum_mass*pendulum_mass*costh*costh*pendulum_length_center_of_mass*pendulum_length_center_of_mass;
		
#if 1
		(*VEL)[0].x = (Lmcosth * kpwpgLmsinth + IIpL2m * umkcvlmw2sinth)
				/ (denompart_true);
		
		(*VEL)[1].x = (-1.0*(mPM*kpwpgLmsinth + Lmcosth*umkcvlmw2sinth))
				/ (denompart_true);
		
#else
		(*VEL)[0].x = -3.0*pendulum_mass*(-1.0*myEntSystem->gravity.y)*sinth*costh
			+ 4.0*pendulum_mass*pendulum_length_center_of_mass*(*POS)[1].x*(*POS)[1].x*sinth
			- 4.0*cart_kvel*(*POS)[0].x
			//- (3.0*pendulum_kang/pendulum_length_center_of_mass)*(*POS)[1].x*costh
			+ 4.0*add_this_vel_to_cart_next_frame;
		(*VEL)[0].x /= (3.0*denompart);
		
		(*VEL)[1].x = -pendulum_mass*pendulum_length_center_of_mass*((*POS)[1].x*(*POS)[1].x)*sinth*costh
				+ (cart_mass+pendulum_mass)*(-1.0*myEntSystem->gravity.y)*sinth
				+ cart_kvel*(*POS)[0].x*costh
				//- (1.0 + (cart_mass/pendulum_mass))*(pendulum_kang/pendulum_length)*(*POS)[1].x
				- add_this_vel_to_cart_next_frame*costh;
		(*VEL)[1].x /= (denompart * pendulum_length_center_of_mass);
#endif


		(*POS)[0] *= deltaT;
		(*VEL)[0] *= deltaT;
		(*POS)[1] *= deltaT;
		(*VEL)[1] *= deltaT;
	}




}
