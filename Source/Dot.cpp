
#include "stdafx.h"
#include "Dot.h"
#include "Integrators.h"
#include "mathTools.h"
#include "Games/CollisionTools.h"
#include "GLVertexes.h"


namespace phys
{
using namespace mathTools; //from math_constants.h
using namespace drawing; //from GLVertexes.h

	ent_dot::ent_dot()
	{
		myEntSystem = NULL;
		mass = 1.0;
		kvel = 0.0001;
	}
	
	ent_dot::ent_dot(SpawnParams params)
	{
		myEntSystem = NULL;

		positions.clear();
		velocities.clear();

		positions.push_back(params.pos_center);
		velocities.push_back(params.velocity);
		
		mass = params.mass_total;
		kvel = params.air_resistance;
		color[0] = params.color[0];
		color[1] = params.color[1];
		color[2] = params.color[2];
	}
	
	void ent_dot::draw()
	{
		glBegin(GL_POINTS);
		glColor3ub(color[0],color[1],color[2]);
		GLVERT2(positions[0]);
		glEnd();
	}
	
	void ent_dot::drawWithOffset(const point& offset, const point& aspectStretch)
	{
		glBegin(GL_POINTS);
		glColor3ub(color[0],color[1],color[2]);
		GLVERT2(positions[0], offset, aspectStretch);
		glEnd();
	}

	bool ent_dot::update(JPHYS_FLOAT_UNIT frametime)
	{
		if(myEntSystem==NULL)
			return true;

		integrate::RK4(this, frametime, &positions, &velocities);

		//integrate::EulerBackwardsFromRK4(this, frametime, &positions, &velocities);
		
		CenterPos = positions[0];

		return false;
	}

	void ent_dot::applyImpulse(const point& Location, const point& Impulse)
	{
		JPHYS_FLOAT_UNIT fDist = 0.25; //allowed max distance from force location

		JPHYS_FLOAT_UNIT tempDist = 0.0;

		tempDist = cTools::getDistSquared(positions[0], Location);

		if(tempDist <= fDist)
			velocities[0] += (Impulse / mass) * (fDist - tempDist);
	}

	void ent_dot::f( std::vector<point> *POS, std::vector<point> *VEL, JPHYS_FLOAT_UNIT deltaT )
	{
		(*POS)[0] = (*VEL)[0];
		
		(*VEL)[0].x = myEntSystem->gravity.x;
		(*VEL)[0].y = myEntSystem->gravity.y;
		
		(*VEL)[0] -= (*POS)[0] * (kvel/mass);	//air resistance (calculated from velocity at start of frame, stored earlier in *POS[j])
		
		(*POS)[0] *= deltaT;
		(*VEL)[0] *= deltaT;
		
//------------------------------------------------------------------------------------
				if(!myEntSystem->allLines.empty())
				{
					Hit hitCheck;
					std::list<Hit> allHits;
					
					std::list<WorldLine*>::iterator Iter = myEntSystem->allLines.begin();
					for(; Iter != myEntSystem->allLines.end(); Iter++)
					{
						if(hitCheck = cTools::getHit( *(*Iter), line(positions[0], positions[0] + (*POS)[0])  ))
							allHits.push_back(hitCheck);
					}

					if(!allHits.empty())
					{
						allHits.sort();
						
						allHits.begin()->dist = (allHits.begin()->dist / (*POS)[0].getLength());

						//positions[j] = allHits.begin()->pt;

						(*POS)[0] = ((*VEL)[0]/deltaT) * -4.0;

						(*VEL)[0] = ((*VEL)[0]/deltaT) * -2.0;
					}
				}
//------------------------------------------------------------------------------------
	}

//=======================================================================================================================================
//=======================================================================================================================================
//=======================================================================================================================================

	
	ent3d_dot::ent3d_dot()
	{
		myEntSystem = NULL;
	}
	
	ent3d_dot::ent3d_dot(SpawnParams_3D params)
	{
		myEntSystem = NULL;

		positions.clear();
		velocities.clear();

		positions.push_back(params.pos_center);
		velocities.push_back(params.velocity);
		
		mass = params.mass_total;
		kvel = params.air_resistance;
		color[0] = params.color[0];
		color[1] = params.color[1];
		color[2] = params.color[2];
	}
	
	void ent3d_dot::draw()
	{
		glBegin(GL_POINTS);
		glColor3ub(color[0],color[1],color[2]);
		GLVERT3(positions[0]);
		glEnd();
	}
	
	void ent3d_dot::drawWithOffset(const point& offset, const point& aspectStretch)
	{
		glBegin(GL_POINTS);
		glColor3ub(color[0],color[1],color[2]);
		GLVERT3(positions[0], offset, aspectStretch);
		glEnd();
	}

	bool ent3d_dot::update(JPHYS_FLOAT_UNIT frametime)
	{
		if(myEntSystem==NULL)
			return true;

		integrate::RK4(this, frametime, &positions, &velocities);

		//integrate::EulerBackwardsFromRK4(this, frametime, &positions, &velocities);
		
		CenterPos = positions[0];

		return false;
	}

	void ent3d_dot::applyImpulse(const point& Location, const point& Impulse)
	{
		JPHYS_FLOAT_UNIT fDist = 0.25; //allowed max distance from force location

		JPHYS_FLOAT_UNIT tempDist = 0.0;

		tempDist = cTools::getDistSquared(positions[0], vec3(Location.x, Location.y, 0.0));

		if(tempDist <= fDist)
			velocities[0] += (vec3(Impulse.x, Impulse.y, 0.0) / mass) * (fDist - tempDist);
	}

	void ent3d_dot::f( std::vector<vec3> *POS, std::vector<vec3> *VEL, JPHYS_FLOAT_UNIT deltaT )
	{
		(*POS)[0] = (*VEL)[0];
		
		(*VEL)[0] = myEntSystem->gravity;
		
		(*VEL)[0] -= (*POS)[0] * (kvel/mass);	//air resistance (calculated from velocity at start of frame, stored earlier in *POS[j])
		
		(*POS)[0] *= deltaT;
		(*VEL)[0] *= deltaT;
		
//------------------------------------------------------------------------------------
				if(!myEntSystem->allLines.empty())
				{
					Hit hitCheck;
					std::list<Hit> allHits;
					
					std::list<WorldLine*>::iterator Iter = myEntSystem->allLines.begin();
					for(; Iter != myEntSystem->allLines.end(); Iter++)
					{
						if(hitCheck = cTools::getHit( *(*Iter), line(point(positions[0].x, positions[0].y), point((positions[0] + (*POS)[0]).x, (positions[0] + (*POS)[0]).y)  )) )
							allHits.push_back(hitCheck);
					}

					if(!allHits.empty())
					{
						allHits.sort();
						
						allHits.begin()->dist = (allHits.begin()->dist / (*POS)[0].getLength());

						//positions[j] = allHits.begin()->pt;

						(*POS)[0] = ((*VEL)[0]/deltaT) * -4.0;

						(*VEL)[0] = ((*VEL)[0]/deltaT) * -2.0;
					}
				}
//------------------------------------------------------------------------------------
	}

	
//=======================================================================================================================================
//=======================================================================================================================================
//=======================================================================================================================================

	
	ent3d_dotsystem::ent3d_dotsystem()
	{
		myEntSystem = nullptr;
	}
	
	ent3d_dotsystem::ent3d_dotsystem(SpawnParams_3D params)
	{
		myEntSystem = nullptr;

		positions.clear();
		velocities.clear();
		masses.clear();
		kvel.clear();

		positions.push_back(params.pos_center);
		velocities.push_back(params.velocity);
		
		masses.push_back(params.mass_total);
		kvel.push_back(params.air_resistance);
		color[0] = params.color[0];
		color[1] = params.color[1];
		color[2] = params.color[2];
	}


	void ent3d_dotsystem::add_point_particle(JPHYS_FLOAT_UNIT new_mass, vec3 new_position, vec3 new_velocity, JPHYS_FLOAT_UNIT new_air_resistance_constant/*=0.0*/, std::vector<unsigned char> new_colors/* = std::vector<unsigned char>()*/)
	{
		if(new_colors.empty() || new_colors.size() == 3)
			add_point_particle(new_position, new_velocity, new_mass, new_air_resistance_constant, new_colors);
	}

	void ent3d_dotsystem::add_point_particle(vec3 new_position, vec3 new_velocity, JPHYS_FLOAT_UNIT new_mass, JPHYS_FLOAT_UNIT new_air_resistance_constant/*=0.0*/, std::vector<unsigned char> new_colors/* = std::vector<unsigned char>()*/)
	{
		if(new_colors.empty() || new_colors.size() == 3)
		{
			positions.push_back(new_position);
			velocities.push_back(new_velocity);
			masses.push_back(new_mass);
			kvel.push_back(new_air_resistance_constant);
			
			if(new_colors.empty())
			{
				colors_vec.push_back(std::vector<unsigned char>(3,255));
				(*colors_vec.rbegin())[0] = color[0];
				(*colors_vec.rbegin())[1] = color[1];
				(*colors_vec.rbegin())[2] = color[2];
			}
			else
				colors_vec.push_back(new_colors);
		}
	}

	
	void ent3d_dotsystem::draw()
	{
		int SIZE = positions.size();
		
		if(colors_vec.empty()==false && color[0]==0 && color[1]==0 && color[2]==0)
		{
			glBegin(GL_POINTS);
			for(int nn=0; nn<SIZE; nn++)
			{
				glColor3ub(colors_vec[nn][0], colors_vec[nn][1], colors_vec[nn][2]);
				GLVERT3(positions[nn]);
			}
			glEnd();
		}
		else
		{
			glBegin(GL_POINTS);
			glColor3ub(color[0],color[1],color[2]);
			for(int nn=0; nn<SIZE; nn++)
			{
				GLVERT3(positions[nn]);
			}
			glEnd();
		}
	}
	
	void ent3d_dotsystem::drawWithOffset(const point& offset, const point& aspectStretch)
	{
		int SIZE = positions.size();
		
		if(colors_vec.empty()==false && color[0]==0 && color[1]==0 && color[2]==0)
		{
			glBegin(GL_POINTS);
			for(int nn=0; nn<SIZE; nn++)
			{
				glColor3ub(colors_vec[nn][0], colors_vec[nn][1], colors_vec[nn][2]);
				GLVERT3(positions[nn], offset, aspectStretch);
			}
			glEnd();
		}
		else
		{
			glBegin(GL_POINTS);
			glColor3ub(color[0],color[1],color[2]);
			for(int nn=0; nn<SIZE; nn++)
			{
				GLVERT3(positions[nn], offset, aspectStretch);
			}
			glEnd();
		}
	}

	bool ent3d_dotsystem::update(JPHYS_FLOAT_UNIT frametime)
	{
		if(myEntSystem==NULL)
			return true;

		integrate::RK4(this, frametime, &positions, &velocities);

		//integrate::EulerBackwardsFromRK4(this, frametime, &positions, &velocities);
		
		CenterPos = positions[0]; //.....whatever (this should be an average)

		return false;
	}

	void ent3d_dotsystem::applyImpulse(const point& Location, const point& Impulse)
	{
		JPHYS_FLOAT_UNIT fDist = 0.25; //allowed max distance from force location

		JPHYS_FLOAT_UNIT tempDist = 0.0;

		tempDist = cTools::getDistSquared(positions[0], vec3(Location.x, Location.y, 0.0));

		if(tempDist <= fDist)
		{
			///this should probably be fixed, for which particle was touched, but whatever

			if(masses.empty())
				velocities[0] += (vec3(Impulse.x, Impulse.y, 0.0)) * (fDist - tempDist);
			else
				velocities[0] += (vec3(Impulse.x, Impulse.y, 0.0) / masses[0]) * (fDist - tempDist);
		}
	}

	void ent3d_dotsystem::f( std::vector<vec3> *POS, std::vector<vec3> *VEL, JPHYS_FLOAT_UNIT deltaT )
	{
		int nn = 0;
		int SIZE = POS->size();

		for(; nn < SIZE; nn++)
		{
			(*POS)[nn] = (*VEL)[nn];
		
			(*VEL)[nn] = myEntSystem->gravity;
		
			(*VEL)[nn] -= (*POS)[nn] * (kvel[nn] / masses[nn]);	//air resistance (calculated from velocity at start of frame, stored earlier in *POS[j])
		
			(*POS)[nn] *= deltaT;
			(*VEL)[nn] *= deltaT;
		
//------------------------------------------------------------------------------------
				if(!myEntSystem->allLines.empty())
				{
					Hit hitCheck;
					std::list<Hit> allHits;
					
					std::list<WorldLine*>::iterator Iter = myEntSystem->allLines.begin();
					for(; Iter != myEntSystem->allLines.end(); Iter++)
					{
						if(hitCheck = cTools::getHit( *(*Iter), line(point(positions[nn].x, positions[nn].y), point((positions[nn] + (*POS)[nn]).x, (positions[nn] + (*POS)[nn]).y)  )) )
							allHits.push_back(hitCheck);
					}

					if(!allHits.empty())
					{
						allHits.sort();
						
						allHits.begin()->dist = (allHits.begin()->dist / (*POS)[nn].getLength());

						//positions[j] = allHits.begin()->pt;

						(*POS)[nn] = ((*VEL)[nn]/deltaT) * -4.0;

						(*VEL)[nn] = ((*VEL)[nn]/deltaT) * -2.0;
					}
				}
		}
//------------------------------------------------------------------------------------
	}



}
