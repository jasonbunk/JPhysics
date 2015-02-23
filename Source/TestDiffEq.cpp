/*
 * Author: Jason Bunk
 * Web page: http://sites.google.com/site/jasonbunk
 * License: The MIT License (MIT) (Expat)
 * Copyright (c) 2015 Jason Bunk
 */

#include "stdafx.h"
#include "TestDiffEq.h"
#include "Integrators.h"
#include "mathTools.h"
#include "Games/CollisionTools.h"
#include "GLVertexes.h"


namespace phys
{
using namespace mathTools; //from math_constants.h
using namespace drawing; //from GLVertexes.h


	ent_TestDiffEq::ent_TestDiffEq()
	{
		myEntSystem = NULL;
	}

	ent_TestDiffEq::ent_TestDiffEq(JPHYS_FLOAT_UNIT framETime, EntitySystem* entSys, SpawnParams params)
	{	
		CenterPos = params.pos_center;

		positions.clear();	 positions.push_back(params.pos_center);
		velocities.clear();	velocities.push_back(params.velocity);
		
		myEntSystem = entSys;

		mass = params.mass_total;
		kvel = params.air_resistance;
		color[0] = params.color[0];
		color[1] = params.color[1];
		color[2] = params.color[2];
		
		std::vector< std::vector<point> > tempPos;
		std::vector< std::vector<point> > tempVel;

		for(int aa=1; aa<=3; aa++) //do 3 times, for 3 previous positions
		{
			//draw();
			tempPos.clear(); tempVel.clear();
			tempPos.push_back(positions);
			tempVel.push_back(velocities);
			f(&tempPos[0], &tempVel[0], framETime);
			FoPPos.push_back(tempPos[0]);
			FoPVel.push_back(tempVel[0]);
			integrate::RK4(this, framETime, &positions, &velocities); //leave behind "Previous" positions while new positions move ahead one frame
		}
		lastY = positions[0].y;
		
		/*
		FoPPos[0][0].y = 0.0;			//0.0;
		FoPPos[1][0].y = 0.000099502;	//0.00099502;
		FoPPos[2][0].y = 0.00078421;	//0.0078421;
		positions[0].x = 0.3;
		positions[0].y = 0.0019656;
		*/
		
	}

	void ent_TestDiffEq::draw()
	{
		glBegin(GL_POINTS);
		glColor3ub(color[0],color[1],color[2]);
		GLVERT2(positions[0]);
		glEnd();
	}
	
	void ent_TestDiffEq::drawWithOffset(const point& offset, const point& aspectStretch)
	{
		glBegin(GL_POINTS);
		glColor3ub(color[0],color[1],color[2]);
		GLVERT2(positions[0], offset, aspectStretch);
		glEnd();
	}

	bool ent_TestDiffEq::update(JPHYS_FLOAT_UNIT frametime)
	{
		if(myEntSystem==NULL)
			return true;

		lastY = positions[0].y;

		//integrate::ABM44(this, frametime);

		CenterPos = positions[0];

		return false;
	}

	void ent_TestDiffEq::applyImpulse(const point& Location, const point& Impulse)
	{
		return;
	}

	void ent_TestDiffEq::f( std::vector<point> *POS, std::vector<point> *VEL, JPHYS_FLOAT_UNIT deltaT )
	{
		//  y' =   ( x^3 * cos(y)^2 ) - ( x * sin(2*y) )

		//  x' =  h

		(*POS)[0].y = (pow((*POS)[0].x, 3.0) * pow(cos((*POS)[0].y), 2.0)) - ((*POS)[0].x * sin((*POS)[0].y * 2.0));

		(*POS)[0].x = 1.0;

		(*POS)[0] *= deltaT;

	}
}
