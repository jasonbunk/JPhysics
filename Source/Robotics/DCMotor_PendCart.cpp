/*
 * Author: Jason Bunk
 * Web page: http://sites.google.com/site/jasonbunk
 * License: The MIT License (MIT) (Expat)
 * Copyright (c) 2015 Jason Bunk
 */

#include "stdafx.h"
#include "Robotics/DCMotor_PendCart.h"
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

	dcmotor_pendcart::dcmotor_pendcart()
	{
		myEntSystem = NULL;
		given_pwm_delta = 0.0;
		positions.resize(1);
		velocities.resize(1);
		originalPrecrashColorSaved[0] = 0;
		originalPrecrashColorSaved[1] = 0;
		originalPrecrashColorSaved[2] = 0;
		hasCrashedIntoSides = false;
		
		Imot = 0.5*0.01*(0.01*0.01);
		stalltorque = 1.0; // ???
		motorBeltDriverRadius = 0.01;
		
		drawn_motor_position = -5.0;
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

	void dcmotor_pendcart::draw()
	{
		phys::point last_drawn_pendulum_bob_pos_cartesian;
		last_drawn_pendulum_bob_pos_cartesian.x = positions[0].x + (pendulum_length_center_of_mass * cos(positions[0].y + ONE_HALF_PI));
		last_drawn_pendulum_bob_pos_cartesian.y =                  (pendulum_length_center_of_mass * sin(positions[0].y + ONE_HALF_PI));
		
		glColor3ub(color[0],color[1],color[2]);
		
		glBegin(GL_LINES);
		//pendulum
		GLVERT2(point(positions[0].x, 0.0));
		GLVERT2(last_drawn_pendulum_bob_pos_cartesian);
		
		GLVERT2(point(positions[0].x + drawn_cart_width*0.5, drawn_cart_height*0.5));
		GLVERT2(point(positions[0].x + drawn_cart_width*(-0.5), drawn_cart_height*0.5));
		
		GLVERT2(point(positions[0].x + drawn_cart_width*(-0.5), drawn_cart_height*0.5));
		GLVERT2(point(positions[0].x + drawn_cart_width*(-0.5), drawn_cart_height*(-0.5)));
		
		GLVERT2(point(positions[0].x + drawn_cart_width*(-0.5), drawn_cart_height*(-0.5)));
		GLVERT2(point(positions[0].x + drawn_cart_width*0.5, drawn_cart_height*(-0.5)));
		
		GLVERT2(point(positions[0].x + drawn_cart_width*0.5, drawn_cart_height*(-0.5)));
		GLVERT2(point(positions[0].x + drawn_cart_width*0.5, drawn_cart_height*0.5));
		glEnd();
		
		glBegin(GL_LINE_STRIP);
		GLCIRCLE2D(last_drawn_pendulum_bob_pos_cartesian, drawn_bob_diameter*0.5, 12);
		glEnd();
		
		glColor3ub(color[0],color[1],color[2]);
		glBegin(GL_LINE_STRIP);
		GLCIRCLE2D(point(drawn_motor_position, 0.0), motorBeltDriverRadius, 5);
		glEnd();
		
		glBegin(GL_LINES);
		GLVERT2(point(drawn_motor_position, 0.0));
		GLVERT2(point(drawn_motor_position + drawn_bob_diameter*cos(positions[0].x / motorBeltDriverRadius), drawn_bob_diameter*sin(positions[0].x / motorBeltDriverRadius)));
		glEnd();
		
		glBegin(GL_LINES);
		GLVERT2(point(cart_bounds_plusminus_x + drawn_cart_width*0.5, -1.0));
		GLVERT2(point(cart_bounds_plusminus_x + drawn_cart_width*0.5,  1.0));
		GLVERT2(point(-cart_bounds_plusminus_x - drawn_cart_width*0.5,-1.0));
		GLVERT2(point(-cart_bounds_plusminus_x - drawn_cart_width*0.5, 1.0));
		glEnd();
	}

	void dcmotor_pendcart::drawWithOffset(const point& offset, const point& aspectStretch)
	{
		std::cout<<"drawWithOffset() --- dont use this!!!!!!!!!"<<std::endl;
	}
	
	bool dcmotor_pendcart::update(JPHYS_FLOAT_UNIT frametime)
	{
		if(myEntSystem==NULL)
			return true;
		
		integrate::RK4(this, frametime, &positions, &velocities);
		
		if(cart_bounds_plusminus_x > 0.0) {
			if(positions[0].x > cart_bounds_plusminus_x) {
				positions[0].x = cart_bounds_plusminus_x;
				velocities[0].x = 0.0;
				
				if(hasCrashedIntoSides == false) {
					hasCrashedIntoSides = true;
					originalPrecrashColorSaved[0] = color[0];
					originalPrecrashColorSaved[1] = color[1];
					originalPrecrashColorSaved[2] = color[2];
					color[0] = (unsigned char)(((int)255) - ((int)color[0]));
					color[1] = (unsigned char)(((int)255) - ((int)color[1]));
					color[2] = (unsigned char)(((int)255) - ((int)color[2]));
				}
			} else if(positions[0].x < (-1.0*cart_bounds_plusminus_x)) {
				positions[0].x = (-1.0*cart_bounds_plusminus_x);
				velocities[0].x = 0.0;
				
				if(hasCrashedIntoSides == false) {
					hasCrashedIntoSides = true;
					originalPrecrashColorSaved[0] = color[0];
					originalPrecrashColorSaved[1] = color[1];
					originalPrecrashColorSaved[2] = color[2];
					color[0] = (unsigned char)(((int)255) - ((int)color[0]));
					color[1] = (unsigned char)(((int)255) - ((int)color[1]));
					color[2] = (unsigned char)(((int)255) - ((int)color[2]));
				}
			}
		}
		
		//CenterPos = positions[0];
		return false;
	}
	
	void dcmotor_pendcart::applyImpulse(const point& Location, const point& Impulse)
	{
		std::cout<<"dcmotor_pendcart::applyImpulse() does nothing"<<std::endl;
	}
	
	/*
	positions[0].x == cart x in cartesian (cart y == 0)
	positions[0].y == pendulum.theta

	velocities[0].x == cart.vel.x
	velocities[0].y == pendulum.omega == theta-dot
	*/
	
#define _Ipc pendulum_I_about_CM
#define _tors stalltorque
#define _plcm pendulum_length_center_of_mass
#define _pm pendulum_mass
#define _mrad motorBeltDriverRadius
#define _kp pendulum_kang
#define _gravg (-1.0*myEntSystem->gravity.y)
	
	void dcmotor_pendcart::f( std::vector<point> *POS, std::vector<point> *VEL, JPHYS_FLOAT_UNIT deltaT )
	{
		double sinth = sin((*POS)[0].y);
		double costh = cos((*POS)[0].y);
		double cos2th = cos(2.0 * (*POS)[0].y);
		
		double original_posx = (*POS)[0].x;
		
		(*POS)[0] = (*VEL)[0];
		
		const double denompart = 2.0*Imot*(_Ipc + _plcm*_plcm*_pm) + _mrad*_mrad*(2.0*_Ipc*(_pm+cart_mass) + _plcm*_plcm*_pm*(_pm + 2.0*cart_mass - _pm*cos2th));
		
		const double kcrvmPWMtsplmrw2sinth = cart_kvel*_mrad*(*POS)[0].x - given_pwm_delta*_tors + _plcm*_pm*_mrad*(*POS)[0].y*(*POS)[0].y*sinth;
		
		(*VEL)[0].x = -2.0*_mrad*(-_mrad*costh*(_Ipc*_kp*(*POS)[0].y + _gravg*_plcm*_plcm*_pm*_pm*sinth) + (_Ipc+_plcm*_plcm*_pm)*kcrvmPWMtsplmrw2sinth) / denompart;
		
		(*VEL)[0].y = -2.0*_plcm*(-_kp*_pm*_mrad*_mrad*(*POS)[0].y*costh*costh + (Imot + (_pm+cart_mass)*_mrad*_mrad)*(_kp*(*POS)[0].y - _gravg*_pm*sinth) + _pm*_mrad*costh*kcrvmPWMtsplmrw2sinth) / denompart;
		
		
		(*POS)[0] *= deltaT;
		(*VEL)[0] *= deltaT;
		
		
		/*if(cart_bounds_plusminus_x > 0.0) {
			if(original_posx >= cart_bounds_plusminus_x) {
				(*POS)[0].x += -10000.0 * deltaT;
				(*POS)[0].y += -10000.0 * deltaT;
				(*VEL)[0].x += -10000.0 * deltaT;
				(*VEL)[0].y += -10000.0 * deltaT;
			} else if(original_posx < (-1.0*cart_bounds_plusminus_x)) {
				(*POS)[0].x +=  10000.0 * deltaT;
				(*POS)[0].y +=  10000.0 * deltaT;
				(*VEL)[0].x +=  10000.0 * deltaT;
				(*VEL)[0].y +=  10000.0 * deltaT;
			}
		}*/
	}




}
