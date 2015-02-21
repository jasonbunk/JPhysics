

#include "stdafx.h"
#include "Entities.h"
#include "Fluid_Old.h"
#include "Integrators.h"
#include "mathTools.h"
#include "Games/CollisionTools.h"

#include "GLVertexes.h"


INLINE JPHYS_FLOAT_UNIT fluid_force2222(JPHYS_FLOAT_UNIT);

JPHYS_FLOAT_UNIT fluid_force2222(JPHYS_FLOAT_UNIT w)
{
	return -2.5 + 5.412214990709*w - 4.576064273404*w*w + 1.879216281049*w*w*w - 0.3810531398816*pow(w,4) + 0.03688696472164*pow(w,5) - 0.001366183878813*pow(w,6);
}


namespace phys
{
using namespace mathTools; //from math_constants.h
using namespace drawing; //from GLVertexes.h


	 ent_fluid_old::ent_fluid_old()
	 {
		myEntSystem = NULL;
		N=0;
		color[0] = 20;
		color[1] = 50;
		color[2] = 250;
		kpos = 0.07;
		kcohesion = 15.0;
		krepulsion = 15.0;
		massEach = 1.0;
		distanceFactor = 0.4;
		gooThresh = 0.10;
		gooSize = 0.005;
	 }
	 
	 ent_fluid_old::ent_fluid_old(SpawnParams_Fluid params)
	 {
		myEntSystem = NULL;
		N=0;
		color[0] = params.color[0];
		color[1] = params.color[1];
		color[2] = params.color[2];
		kpos = params.friction;
		kcohesion = params.kCohesion;
		krepulsion = params.kRepulsion;
		massEach = params.mass_each;
		distanceFactor = params.distanceFactor;
		gooThresh = 0.10;
		gooSize = 0.005;
	 }

	 void ent_fluid_old::add_element(SpawnParams params)
	 {
		 positions.push_back(params.pos_center);
		 velocities.push_back(params.velocity);
		 N++;
		 
		 startTracePos.push_back(phys::point(0,0));
		 currentEdgePos.push_back(phys::point(0,0));
		 tracking.push_back(false);
	 }

	 void ent_fluid_old::deleteAllElements()
	 {
		 positions.clear();
		 velocities.clear();
		 N = 0;
		 
		 startTracePos.clear();
		 currentEdgePos.clear();
		 tracking.push_back(false);
	 }

//-- IEntityOld required overrides:
	
	 void ent_fluid_old::draw()
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
	 
	 void ent_fluid_old::drawWithOffset(const point& offset, const point& aspectStretch)
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

	 bool ent_fluid_old::update(JPHYS_FLOAT_UNIT frametime)
	 {
		if(myEntSystem==NULL)
			return true;
		
			for(int a=0; a<N; a++)
			{
				velocities[a] *= (1 - (frametime*kpos));
				if(positions[a].x < -myEntSystem->universeLimits.x || positions[a].x > myEntSystem->universeLimits.x
				|| positions[a].y < -myEntSystem->universeLimits.y || positions[a].y > myEntSystem->universeLimits.y)
				{
					positions.erase(positions.begin()+a);
					velocities.erase(velocities.begin()+a);
					N--;
					a--;
				}
			}
		
		//switch(myEntSystem->INTEGRATOR)
		//{
		//		case 1:
			integrate::SemiImplicitEuler(this, frametime, &positions, &velocities);
		/*break;	case 2:
			integrate::SemiImplicitEuler(this, frametime, &positions, &velocities);
		break;	case 3:
			integrate::SemiImplicitEuler2(this, frametime, &positions, &velocities);
		break;	case 4:
			integrate::SemiImplicitEuler(this, frametime, &positions, &velocities);
		break;	case 5:
			integrate::RK4(this, frametime, &positions, &velocities);
		break;	case 6:
			integrate::EulerBackwardsFromRK4(this, frametime, &positions, &velocities);											
		//break;	case 7:
		//	integrate::EulerBackwardsFromEuler(this, frametime, &positions, &velocities);
		//break;	case 8:
			//trapezoid
		//	integrate::BDF2_fromRK4(this, frametime, &positions, &velocities);
		break;	case 7:
			integrate::ABM22(this, frametime);
		break;	case 8:
			integrate::SemiImplicitEuler(this, (frametime*0.2), &positions, &velocities);
			integrate::SemiImplicitEuler(this, (frametime*0.2), &positions, &velocities);
			integrate::SemiImplicitEuler(this, (frametime*0.2), &positions, &velocities);
			integrate::SemiImplicitEuler(this, (frametime*0.2), &positions, &velocities);
			integrate::SemiImplicitEuler(this, (frametime*0.2), &positions, &velocities);
		break;	case 9:
			integrate::VVerlet1(this, (frametime*0.5), &positions, &velocities);
			integrate::VVerlet1(this, (frametime*0.5), &positions, &velocities);
		break;	case 0:
			integrate::VVerlet2(this, (frametime*0.5), &positions, &velocities);
			integrate::VVerlet2(this, (frametime*0.5), &positions, &velocities);
		break;
		}*/

		return false;
	 }

	void ent_fluid_old::f( std::vector<point> *POS, std::vector<point> *VEL, JPHYS_FLOAT_UNIT deltaT )
	{
		int n;
		point uv; JPHYS_FLOAT_UNIT w;
		std::vector<point> tempvel(N);
		Hit pH;
		
		for(n=0; n<N; n++)
		{
			tempvel[n] = (*VEL)[n];

			(*VEL)[n].x = myEntSystem->gravity.x;// - (tempvel[n]*0.02);
			(*VEL)[n].y = myEntSystem->gravity.y;// - (tempvel[n]*0.02);
			
			std::list<WorldLine*>::iterator Iter = myEntSystem->allLines.begin();
			for(; Iter != myEntSystem->allLines.end(); Iter++)
			{
				pH = cTools::nearestHitAlongLine((*POS)[n], (*(*Iter)));

				uv = pH.pt - (*POS)[n]; //we want change in position from fluid's point to the hit along line

				pH.dist = uv.getLength();

				w = pH.dist/distanceFactor;	//   divide by distancefactor
											 //   example: 2 means the particles should stay twice as far apart

				if(w < 1.499)
				{
					uv.x = atan2(uv.y, uv.x);
					w = krepulsion*fluid_force2222(w);
					w *= pow(4,-0.5*pH.dist)*20; //*5 is optional!! it makes the wall repel 5x stronger
					(*VEL)[n].x += w*cos(uv.x);
					(*VEL)[n].y += w*sin(uv.x);
				}
			}
		}
		

		for(n=0; n<N-1; n++)
		{
			for(int j=N-1; j>n; j--)
			{
				uv = (*POS)[j] - (*POS)[n];

				pH.dist = uv.getLength();
				
				w = pH.dist/distanceFactor;		//   divide by distancefactor
												 //   example: 2 means the particles should stay twice as far apart

				if(w < 7.499)
				{
					uv.x = atan2(uv.y, uv.x);
					w = fluid_force2222(w);
					w *= pow(4,-0.5*pH.dist);

					w *= (w>0) ? kcohesion : krepulsion; //if force (w) is positive, it's cohesion; if force (w) is negative, it's repulsion

					(*VEL)[n].x += w*cos(uv.x);
					(*VEL)[n].y += w*sin(uv.x);
					(*VEL)[j].x -= w*cos(uv.x);
					(*VEL)[j].y -= w*sin(uv.x);
				}
			}
		}

		for(n=0; n<N; n++)
		{
			(*POS)[n] = tempvel[n];

			//(*VEL)[n] -= (tempvel[n]*kpos);
			
			(*POS)[n] *= deltaT;
			(*VEL)[n] *= deltaT;
		}
	}
	
	void ent_fluid_old::applyImpulse(const point& Location, const point& Impulse)
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

	
	void ent_fluid_old::draw_metaballs(JPHYS_FLOAT_UNIT step, JPHYS_FLOAT_UNIT tolerance)
	{
	//	First, track the border for all balls and store
	//	it to pos0 and edgePos. The latter will move along the border,
	//	pos0 stays at the initial coordinates.
		for(int a=0; a<N; a++)
		{
			currentEdgePos[a] = startTracePos[a] = trackTheBorder(positions[a] + phys::point(0.0, 0.1));

			tracking[a] = true;
		}
		
		glBegin( GL_LINES );
		glColor3ub(1, 1, 255);

		int loopIndex = 0;
		while(loopIndex < 200)
		{
			loopIndex += 1;
			for(int ab=0; ab<N; ab++)
			{
				if(!tracking[ab])
					continue;

				//store the old coordinates
				phys::point old_pos = currentEdgePos[ab];

				//walk along the tangent, using chosen differential method
				currentEdgePos[ab] = rungeKutta2Tangent(currentEdgePos[ab], step);
            
				//correction step towards the border
				stepOnceTowardsBorder(currentEdgePos[ab]); //will modify parameter edgePos as "return value"

				glVertex2f(old_pos.x, old_pos.y);
				glVertex2f(currentEdgePos[ab].x, currentEdgePos[ab].y);
           
				//pygame.draw.line(screen, (255, 255, 255),
				//				  (old_pos.real, old_pos.imag),
				//				  (ball.edgePos.real, ball.edgePos.imag))

				//check if we've gone a full circle or hit some other
				//edge tracker
				for(int obb=0; obb<N; obb++)
				{
					if((obb!=ab || loopIndex>3) && (fabs((startTracePos[obb] - currentEdgePos[ab]).getLength()) < (step*tolerance)))
					{
						tracking[ab] = false;
					}
				}
			//	for ob in balls:
			//		if (ob is not ball or loopIndex > 3) and
			//			abs(ob.pos0 - ball.edgePos) < step:
			//			ball.tracking = False
			}

			int tracks = 0;
			for(int ab=0; ab<N; ab++)
			{
				if(tracking[ab])
					tracks++;
			}
			if(tracks == 0)
				break;
		}
		glEnd();
	}

	//Return the metaball field's force at point 'pos'.
	JPHYS_FLOAT_UNIT ent_fluid_old::calcForce(point pos)
	{
		  JPHYS_FLOAT_UNIT force = 0.0;
		  for(int i=0;i<N;i++)
		  {
			  //### Formula (1)
			  //should be this: div = abs(ball.pos - pos)^self.goo

			  JPHYS_FLOAT_UNIT div = (positions[i]-pos).getLengthSquared();

			  if(mathTools::cmpFloatToZero(div)) //# to prevent division by zero
				  force += 10000.0; //#"big number"
			  else
				  force += gooSize / div;
		  }
		  return force;
	}

	/**Return a normalized (magnitude = 1) normal at point 'pos'.*/
	  point ent_fluid_old::calcNormal(point pos)
	  {
		  point np;
		  point v;

		  for(int i=0;i<N;i++)
		  {
			  //### Formula (3)
			  //div = abs(ball.pos - pos)**(2 + self.goo)
			  //np += -self.goo * ball.size * (ball.pos - pos) / div

			  v=(positions[i]-pos);

			  //JPHYS_FLOAT_UNIT div = pow(v.getLength(), 2.0 + goo);

			  JPHYS_FLOAT_UNIT div(v.getLengthSquared());
			  div *= div;

			  //JPHYS_FLOAT_UNIT div = v.getLengthSquared() * (goo==2.0 ? v.getLength() : powf(v.getLength(), goo));

			  np += (v / div) * gooSize * -2.0;

		  }
		  return np.GetNormalized();

	  }

	/**Return a normalized (magnitude = 1) tangent at point 'pos'.*/
	  point ent_fluid_old::calcTangent(point pos)
	  {
		  phys::point np = calcNormal(pos);
		  //### Formula (7)
		  return phys::point(-np.y, np.x);
	  }

	  /**Step once towards the border of the metaball field, return
		 new coordinates and force at old coordinates.
	  */
	  JPHYS_FLOAT_UNIT ent_fluid_old::stepOnceTowardsBorder(point &pos)
	  {
		  JPHYS_FLOAT_UNIT force = calcForce(pos);
		  phys::point np = calcNormal(pos);
		  //### Formula (5)
		  JPHYS_FLOAT_UNIT stepsize = sqrt(gooSize / gooThresh) -
					 sqrt(gooSize / force) + 0.01;
		  pos += np * stepsize;
		  return force;
	  }

	  /**Track the border of the metaball field and return new
		 coordinates.
	  */
	  point ent_fluid_old::trackTheBorder(point pos)
	  {
		  JPHYS_FLOAT_UNIT force = 9999999;
		  //# loop until force is weaker than the desired threshold

		  int foo=0;
		  while (force > gooThresh)
		  {
			  foo++;
			  if(foo>100) break;
			  force = stepOnceTowardsBorder(pos);
		  }
		  return pos;
	  }



	/**Euler's method.
	   The most simple way to solve differential systems numerically.
	*/
	point ent_fluid_old::eulerTangent(point pos, JPHYS_FLOAT_UNIT h)
	{
		return pos + (calcTangent(pos) * h);
	}


	/**Runge-Kutta 2 (=mid-point).
	   This is only a little more complex than the Euler's method,
	   but significantly better.
	*/
	point ent_fluid_old::rungeKutta2Tangent(point pos, JPHYS_FLOAT_UNIT h)
	{
		return pos + (calcTangent(pos + (calcTangent(pos) * h / 2)) * h);
	}

}
