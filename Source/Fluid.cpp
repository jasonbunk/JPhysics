/*
 * Author: Jason Bunk
 * Web page: http://sites.google.com/site/jasonbunk
 * License: The MIT License (MIT) (Expat)
 * Copyright (c) 2015 Jason Bunk
 */

#include "stdafx.h"
#include "Entities.h"
#include "Fluid.h"
#include "Integrators.h"
#include "mathTools.h"
#include "Games/CollisionTools.h"

#include "GLVertexes.h"
#include "clamp.h"


INLINE JPHYS_FLOAT_UNIT fluid_force(JPHYS_FLOAT_UNIT);

JPHYS_FLOAT_UNIT fluid_force(JPHYS_FLOAT_UNIT w)
{
	return -2.5 + 5.412214990709*w - 4.576064273404*w*w + 1.879216281049*w*w*w - 0.3810531398816*pow(w,4) + 0.03688696472164*pow(w,5) - 0.001366183878813*pow(w,6);
}


namespace phys
{
using namespace mathTools; //from math_constants.h
using namespace drawing; //from GLVertexes.h


	 ent_fluid::ent_fluid()
	 {
		myEntSystem = NULL;
		N=0;
		checkBounds=0;
		color[0] = 20;
		color[1] = 50;
		color[2] = 250;
		friction = 0.07;
		kcohesion = 15.0;
		krepulsion = 15.0;
		massEach = 1.0;
		distanceFactor = 0.4;
	 }
	 
	 ent_fluid::ent_fluid(SpawnParams_Fluid params)
	 {
		myEntSystem = NULL;
		N=0;
		checkBounds=0;
		color[0] = params.color[0];
		color[1] = params.color[1];
		color[2] = params.color[2];
		friction = params.friction;
		kcohesion = params.kCohesion;
		krepulsion = params.kRepulsion;
		massEach = params.mass_each;
		distanceFactor = params.distanceFactor;
	 }

	 void ent_fluid::add_element(SpawnParams params)
	 {
		 positions.push_back(params.pos_center);
		 velocities.push_back(params.velocity);
		 N++;
	 }

	 void ent_fluid::deleteAllElements()
	 {
		 positions.clear();
		 velocities.clear();
		 N = 0;
	 }

//-- IEntityOld required overrides:
	
	 void ent_fluid::draw()
	 {
		glBegin(GL_POINTS);
		glColor3ub(color[0],color[1],color[2]);

		std::vector<point>::iterator Iter = positions.begin();
		
		CenterPos.x = 0.0; CenterPos.y = 0.0;
		for(; Iter!=positions.end(); Iter++)
		{
			GLVERT2((*Iter));
			CenterPos += (*Iter);
		}
		CenterPos /= ((JPHYS_FLOAT_UNIT)N);

		glEnd();
	 }
	 
	 void ent_fluid::drawWithOffset(const point& offset, const point& aspectStretch)
	 {
		glBegin(GL_POINTS);
		glColor3ub(color[0],color[1],color[2]);

		std::vector<point>::iterator Iter = positions.begin();
		
		CenterPos.x = 0.0; CenterPos.y = 0.0;
		for(; Iter!=positions.end(); Iter++)
		{
			GLVERT2((*Iter), offset, aspectStretch);
			CenterPos += (*Iter);
		}
		CenterPos /= ((JPHYS_FLOAT_UNIT)N);

		glEnd();
	 }

	 void ent_fluid::update(JPHYS_FLOAT_UNIT frametime)
	 {
		if(myEntSystem==NULL)
			return;
		
		 checkBounds++;
		 if(checkBounds == 10) //check to delete it only once every 10 frames ("garbage collection" doesn't need to be immediate)
		 {
			checkBounds = 0;
			for(int a=0; a<N; a++)
			{
				if(fabs(positions[a].x) > myEntSystem->universeLimits.x || fabs(positions[a].y) > myEntSystem->universeLimits.y)
				{
					positions.erase(positions.begin()+a);
					velocities.erase(velocities.begin()+a);
					N--;
					a--; //so we don't overreach bounds of the arrays
				}
			}
		 }

		int n;
		point uv; point newpos; JPHYS_FLOAT_UNIT w;
		Hit pH;
		
		for(n=0; n<N; n++)
		{
			velocities[n] *= clamp((1.0 - (frametime*friction)), 0.0, 1.0); //"fake" friction imitation

			newpos = (positions[n] + (velocities[n]*frametime)); //quick semi-implicit Euler
			
			//////////////////////////////////////////////////////////////////////// COLLIDE WITH WORLD LINES
			std::list<WorldLine*>::iterator Iter = myEntSystem->allLines.begin();
			for(; Iter != myEntSystem->allLines.end(); Iter++)
			{
				pH = cTools::getHitOrDist(line(positions[n],newpos), (*(*Iter)));

				uv = pH.pt - positions[n]; //we want change in position from fluid's pos to the hit along line

				w = 1.7*pH.dist/distanceFactor;	//   divide by distancefactor
											 //   e.g. 2.0 means the particles should stay twice as far apart

				if(pH.b)
				{
					newpos = positions[n]; //we decide not to move at all because of collision

					cTools::collisionRotation(velocities[n], (*(*Iter)));
					
					velocities[n] *= (1.0 - clamp(friction, 0.0, 1.0)); //lose some velocity
				}
				else if(w < 1.499)
				{
					velocities[n] *= (1.0 - clamp(friction*0.04*(1.0-(w/1.499)), 0.0, 1.0)); //"fake" friction imitation for sliding along wall (in addition to normal air resistance), max friction when closest, friction falls off to zero when 1.499 away

					//uv.x = atan2(uv.y, uv.x);
					w = krepulsion*fluid_force(w);
					w *= frametime*pow(4,-0.5*pH.dist)*5.0; //*5 is arbitrary, it makes the wall repel 5x stronger than other particles

					velocities[n] += uv*(w/pH.dist);

					//velocities[n].x += w*cos(uv.x);
					//velocities[n].y += w*sin(uv.x);
				}
			}
			//////////////////////////////////////////////////////////////////////// COLLIDE WITH DANGER LINES (other entities)
			std::list<IEntity*>::iterator EntityIter = myEntSystem->allEntities.begin();
			for(; EntityIter != myEntSystem->allEntities.end(); EntityIter++)
			{
				std::vector<EntityEdgeLine>::iterator DangerIter = (*EntityIter)->DangerLines.begin();
				for(; DangerIter != (*EntityIter)->DangerLines.end(); DangerIter++)
				{
					pH = cTools::getHitOrDist(line(positions[n],newpos), (*DangerIter));

					uv = pH.pt - positions[n]; //we want change in position from fluid's pos to the hit along line

					w = 1.7*pH.dist/distanceFactor;	//   divide by distancefactor
												 //   e.g. 2.0 means the particles should stay twice as far apart

					if(pH.b)
					{
						//(*DangerIter).CollideWith(true,positions[n],newpos,velocities[n]);
						newpos = positions[n]; //we decide not to move at all because of collision

						cTools::collisionRotation(velocities[n], (*DangerIter).GetLine());
						
						velocities[n] *= (1.0 - clamp(friction, 0.0, 1.0)); //lose some velocity
					}
					else if(w < 1.499)
					{
						velocities[n] *= (1.0 - clamp(friction*0.04*(1.0-(w/1.499)), 0.0, 1.0)); //"fake" friction imitation for sliding along wall (in addition to normal air resistance), max friction when closest, friction falls off to zero when 1.499 away

						//uv.x = atan2(uv.y, uv.x);
						w = krepulsion*fluid_force(w);
						w *= frametime*pow(4,-0.5*pH.dist)*(*DangerIter).GetLength()*5.0; //x5 is arbitrary

						velocities[n] += uv*(w/pH.dist);

						(*DangerIter).Push(positions[n], uv*(-w/pH.dist));

						//velocities[n].x += w*cos(uv.x);
						//velocities[n].y += w*sin(uv.x);
					}
				}
			}
			////////////////////////////////////////////////////////////////////////
			
			positions[n] = newpos; //move it -after- we make sure that we didn't collide with anything

			GLVERT2(positions[n]);

			velocities[n].x += myEntSystem->gravity.x * frametime;
			velocities[n].y += myEntSystem->gravity.y * frametime;
		}
		

		for(n=0; n<N-1; n++)
		{
			for(int j=N-1; j>n; j--)
			{
				uv = positions[j] - positions[n];

				pH.dist = uv.getLength();
				
				w = pH.dist/distanceFactor;		//   divide by distancefactor
												 //   e.g. 2.0 means the particles should try to stay twice as far apart

				if(w < 7.499)
				{
					//uv.x = atan2(uv.y, uv.x);
					w = fluid_force(w);
					w *= frametime*pow(4,-0.5*pH.dist);

					w *= (w>0) ? kcohesion : krepulsion; //if force (w) is positive, it's cohesion; if force (w) is negative, it's repulsion

					velocities[n] += uv*(w/pH.dist);
					velocities[j] -= uv*(w/pH.dist);

					//velocities[n].x += w*cos(uv.x);
					//velocities[n].y += w*sin(uv.x);
					//velocities[j].x -= w*cos(uv.x);
					//velocities[j].y -= w*sin(uv.x);
				}
			}
		}
	 }
	 
	//Check collisions against another moving object nearby, by using a frame of reference in which
	 //this fluid particle is moving and the collidee is static
	void ent_fluid::checkCollisions(const line &hitSurface, const point& offset)
	{
		for(int n=0; n<N; n++)
		{
			Hit pH = cTools::getHitOrDist(line(positions[n],positions[n]-offset), hitSurface);

			if(pH.b)
			{
				positions[n] += offset;

				velocities[n] += (offset / std::max(0.10,myEntSystem->curFrameTime)); //collision affects velocity, but not too much
			}
		}
	}
	
	void ent_fluid::applyImpulse(const point& Location, const point& Impulse)
	{
		JPHYS_FLOAT_UNIT fDist = 0.25*(0.3*0.3); //allowed max distance from force location

		JPHYS_FLOAT_UNIT tempDist = 0.0;

		for(int j=0; j<N; j++)
		{
			tempDist = cTools::getDistSquared(positions[j], Location);

			if(tempDist <= fDist)
				velocities[j] += (Impulse) * (fDist - tempDist);
		}
	}

}
