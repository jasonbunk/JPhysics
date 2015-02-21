
#include "stdafx.h"
#include "Rectangle.h"
#include "Integrators.h"
#include "mathTools.h"
#include "Games/CollisionTools.h"

#include "GLVertexes.h"
#include "clamp.h"


namespace phys
{
using namespace mathTools; //from math_constants.h
using namespace drawing; //from GLVertexes.h


	ent_rectangle::ent_rectangle() :
		mass(0.0),
		kpos(0.0),
		eqdist(0.0),
		friction(0.0),
		drawHollow(false),
		draw_corner_dots(false),
		edges_are_fixed(false),
		calculate_diagonal_forces(true),
		left_edge_only_fixed(false),
		move_only_on_x_axis(false),
		affected_by_gravity(true)
	{
		myEntSystem = nullptr;
	}
		
	ent_rectangle::ent_rectangle(JPHYS_FLOAT_UNIT NN, JPHYS_FLOAT_UNIT MM, EntitySystem* entSys, SpawnParams_Soft params) :
		draw_corner_dots(false),
		edges_are_fixed(false),
		calculate_diagonal_forces(true),
		left_edge_only_fixed(false),
		move_only_on_x_axis(false),
		affected_by_gravity(true)
	{
		NN = fabs(floor(NN));
		MM = fabs(floor(MM)); //make sure user gave whole-number widths/heights
		
		if(cmpFloatToZero(NN) || cmpFloatToZero(MM)) {NN = 2.0; MM = 2.0;}
		
		CenterPos = params.entParams.pos_center;

		params.entParams.pos_center.x -= (NN-1)*params.vertex_equilibrium_distance*0.5;
		params.entParams.pos_center.y -= (MM-1)*params.vertex_equilibrium_distance*0.5;

		N = ((static_cast<unsigned int>(NN))*(static_cast<unsigned int>(MM)));

		 positions.resize(N);
		velocities.resize(N);
		TempVelocities.resize(N);
		
		myEntSystem = entSys;
		drawHollow = true;

		NN -= 0.1;
		MM -= 0.1; //so JPHYS_FLOAT_UNIT comparisons (<) never have to worry about being a little bit over

		N = 0;
		for(JPHYS_FLOAT_UNIT m=0; m<MM; m++)
		{
			for(JPHYS_FLOAT_UNIT n=0; n<NN; n++)
			{
				positions[N] = point(
					params.entParams.pos_center.x + n*params.vertex_equilibrium_distance,
					params.entParams.pos_center.y + m*params.vertex_equilibrium_distance	);
				
				velocities[N] = params.entParams.velocity;

				/*//give some of them an initial impulse
				if(N <= 3)
					velocities[N] = (params.entParams.velocity*-19.0);
				*/

				TempVelocities[N].x = TempVelocities[N].y = 0.0;

				N++;
			}
		}
		N = static_cast<int>(NN+0.2); M = static_cast<int>(MM+0.2);

		mass = params.entParams.mass_total / static_cast<double>(N*M);
		kpos = params.springconst_linear;
		eqdist = params.vertex_equilibrium_distance;
		friction = params.entParams.air_resistance;
		color[0] = params.entParams.color[0];
		color[1] = params.entParams.color[1];
		color[2] = params.entParams.color[2];
	}

	ent_rectangle::ent_rectangle(JPHYS_FLOAT_UNIT NN, JPHYS_FLOAT_UNIT MM, JPHYS_FLOAT_UNIT framETime, EntitySystem* entSys, SpawnParams_Soft params)
		: draw_corner_dots(false),
		edges_are_fixed(false),
		calculate_diagonal_forces(true),
		left_edge_only_fixed(false),
		move_only_on_x_axis(false),
		affected_by_gravity(true)
	{
		NN = fabs(floor(NN));
		MM = fabs(floor(MM)); //make sure user gave whole-number widths/heights
		
		if(cmpFloatToZero(NN) || cmpFloatToZero(MM)) {NN = 2.0; MM = 2.0;}
		
		CenterPos = params.entParams.pos_center;

		params.entParams.pos_center.x -= (NN-1)*params.vertex_equilibrium_distance*0.5;
		params.entParams.pos_center.y -= (MM-1)*params.vertex_equilibrium_distance*0.5;

		N = ((static_cast<unsigned int>(NN))*(static_cast<unsigned int>(MM)));

		 positions.resize(N);
		velocities.resize(N);
		TempVelocities.resize(N);
		
		myEntSystem = entSys;
		drawHollow = true;

		NN -= 0.1;
		MM -= 0.1; //so JPHYS_FLOAT_UNIT comparisons (<) never have to worry about being a little bit over

		N = 0;
		for(JPHYS_FLOAT_UNIT m=0; m<MM; m++)
		{
			for(JPHYS_FLOAT_UNIT n=0; n<NN; n++)
			{
				positions[N] = point(
					params.entParams.pos_center.x + n*params.vertex_equilibrium_distance,
					params.entParams.pos_center.y + m*params.vertex_equilibrium_distance	);
				
				velocities[N] = params.entParams.velocity;

				/*//give some of them an initial impulse
				if(N <= 3)
					velocities[N] = (params.entParams.velocity*-19.0);
				*/

				TempVelocities[N].x = TempVelocities[N].y = 0.0;

				N++;
			}
		}
		N = static_cast<int>(NN+0.2); M = static_cast<int>(MM+0.2);

		mass = params.entParams.mass_total / static_cast<double>(N*M);
		kpos = params.springconst_linear;
		eqdist = params.vertex_equilibrium_distance;
		friction = params.entParams.air_resistance;
		color[0] = params.entParams.color[0];
		color[1] = params.entParams.color[1];
		color[2] = params.entParams.color[2];
		
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
	}

	void ent_rectangle::draw()
	{
		int j,m,n;

		glBegin(GL_LINES);
		glColor4ub(color[0],color[1],color[2], 255);

		m = M*(N-1);
		for(j=0; j<m; j++)
		{
			n = j%(N-1) + N*(j-(j%(N-1)))/(N-1);

			GLVERT2(positions[n+1]);
			GLVERT2(positions[n]);
		}

		m = N*(M-1);
		for(j=0; j<m; j++)
		{
			GLVERT2(positions[N+j]);
			GLVERT2(positions[j  ]);
		}
				
		if(!drawHollow)
		{
			m = (N-1)*(M-1);
			for(j=0; j<m; j++)
			{
				n = j + (j-(j%(N-1)))/(N-1);
						
				GLVERT2(positions[N+n+1]);
				GLVERT2(positions[n    ]);
				GLVERT2(positions[N+n  ]);
				GLVERT2(positions[n+1  ]);
			}
			
			//uv = (*POS)[(N*M)-1] - (*POS)[0];
			//uv = (*POS)[N*(M-1)] - (*POS)[N-1];
			
			GLVERT2(positions[(N*M)-1]);
			GLVERT2(positions[0      ]);
			
			GLVERT2(positions[N*(M-1)]);
			GLVERT2(positions[N-1    ]);
		}

		glEnd();
		
		if(draw_corner_dots)
		{
			glBegin(GL_POINTS);

			m = (N*M);

			for(j=0; j<m; j++)
			{
				GLVERT2(positions[j]);
			}
			glEnd();
		}
	}
	void ent_rectangle::drawWithOffset(const point& offset, const point& aspectStretch)
	{
		std::cout << "HEY! DON'T CALL \"drawWithOffset\", use OpenGL matrices to push/pop draw matrices and deal with it that way!" << std::endl;
	}

	void ent_rectangle::UpdateCenterPos()
	{
		CenterPos.x = 0.0; CenterPos.y = 0.0;

		int Size = positions.size();

		for(int a=0; a<Size; a++)
		{
			CenterPos += positions[a];
			TempVelocities[a].x = TempVelocities[a].y = 0.0;
		}

		CenterPos /= ((JPHYS_FLOAT_UNIT)(N*M));
	}
	
	bool ent_rectangle::update(JPHYS_FLOAT_UNIT frametime)
	{
		if(myEntSystem == nullptr)
			return true;

		if(edges_are_fixed || left_edge_only_fixed)
		{
			int m=0;
			int j=0;
			for(int n=0; n<N; n++)
			{
				for(m=0; m<M; m++)
				{
					if(N > 1 && (n==0 || n==(N-1)))
					{
						if((left_edge_only_fixed && n==0) || left_edge_only_fixed==false) {
							velocities[j].Nullify();
						}
					}
					if(M > 1 && (m==0 || m==(M-1)) && left_edge_only_fixed==false)
					{
						velocities[j].Nullify();
					}
					j++;
				}
			}
		}

		JPHYS_FLOAT_UNIT temp_friction = friction;
		if(myEntSystem->INTEGRATOR == 1 || myEntSystem->INTEGRATOR == 2)
		{
			friction = (friction * 200.0 * frametime);
		
			if(myEntSystem->INTEGRATOR == 1)
			{
				int Size = positions.size();
				for(int a=0; a<Size; a++)
				{
					velocities[a] *= (1.0 - clamp(friction, 0.0, 1.0)); //"fake" friction imitation
				}
			}
			if(myEntSystem->INTEGRATOR == 2)
			{
				int Size = positions.size();
				for(int a=0; a<Size; a++)
				{
					velocities[a] *= (1 - (frametime*0.1));
					//JPHYS_FLOAT_UNIT LEN = ((double)velocities[a].getLength());
					//velocities[a] *= ((JPHYS_FLOAT_UNIT)pow(LEN,0.005));
				}
			}
		}
		
		FoPPos.clear(); FoPPos.push_back(positions);

		switch(myEntSystem->INTEGRATOR)
		{
				case 1:
			integrate::SemiImplicitEuler(this, frametime, &positions, &velocities);
		break;	case 2:
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
		}

		//integrate::RK4(this, frametime, &positions, &velocities);																								

		UpdateCenterPos();

		friction = temp_friction;

		return false;
	}
	
	void ent_rectangle::f( std::vector<point> *POS, std::vector<point> *VEL, JPHYS_FLOAT_UNIT deltaT )
	{
		std::vector<JPHYS_FLOAT_UNIT> HS((N-1)*M);
		std::vector<JPHYS_FLOAT_UNIT> HT((N-1)*M);
		std::vector<JPHYS_FLOAT_UNIT> VS((M-1)*N);
		std::vector<JPHYS_FLOAT_UNIT> VT((M-1)*N);
		std::vector<JPHYS_FLOAT_UNIT> AS((M-1)*(N-1)*2);
		std::vector<JPHYS_FLOAT_UNIT> AT((M-1)*(N-1)*2);

		int j,n,m;
		point uv;

		//------------------------
		point corners[2];
		
		if(calculate_diagonal_forces)
		{
			uv = (*POS)[(N*M)-1] - (*POS)[0];
			corners[0].y = atan2(uv.y, uv.x);
			uv.x = uv.getLength();
			corners[1].x = eqdist*sqrt(((JPHYS_FLOAT_UNIT)((N-1)*(N-1)))+((JPHYS_FLOAT_UNIT)((M-1)*(M-1))));

			corners[0].x = (uv.x - corners[1].x) * cos(corners[0].y);
			corners[0].y = (uv.x - corners[1].x) * sin(corners[0].y);
			
			uv = (*POS)[N*(M-1)] - (*POS)[N-1];
			corners[1].y = atan2(uv.y, uv.x);
			uv.x = uv.getLength();
			uv.y = corners[1].x;

			corners[1].x = (uv.x - uv.y) * cos(corners[1].y);
			corners[1].y = (uv.x - uv.y) * sin(corners[1].y);
		}

		//------------------------
		
		m = M*(N-1);
		for(j=0; j<m; j++)
		{
			n = j%(N-1) + N*(j-(j%(N-1)))/(N-1);
			uv = (*POS)[n+1] - (*POS)[n];
			HS[j] = uv.getLength() - eqdist;
			HT[j] = atan2(uv.y,uv.x);
		}

		m = N*(M-1);
		for(j=0; j<m; j++)
		{
			uv = (*POS)[N+j] - (*POS)[j];
			VS[j] = uv.getLength() - eqdist;
			VT[j] = atan2(uv.y,uv.x);
		}
		
		if(calculate_diagonal_forces)
		{
			m = (N-1)*(M-1);
			for(j=0; j<m; j++)
			{
				n = j + (j-(j%(N-1)))/(N-1);
				uv = (*POS)[N+n+1] - (*POS)[n];
				AS[j*2] = uv.getLength() - (eqdist*ROOT_TWO);
				AT[j*2] = atan2(uv.y,uv.x);

				uv = (*POS)[n+1] - (*POS)[N+n];
				AS[j*2+1] = uv.getLength() - (eqdist*ROOT_TWO);
				AT[j*2+1] = atan2(uv.y,uv.x);
			}
		}

		Hit hitCheck;

		int A,B;
		j = -1;
		for(m=0; m<M; m++)
		{
			for(n=0; n<N; n++)
			{
				j++;
				(*POS)[j] = (*VEL)[j];

				if(calculate_diagonal_forces == false)
				{
					(*VEL)[j].x = 0.0; (*VEL)[j].y = 0.0;
				}
				else
				{
					if(j!=0 && j!=(N-1) && j!=(N*(M-1)) && j!=((N*M)-1))
					{
						(*VEL)[j].x = 0.0; (*VEL)[j].y = 0.0;
					}
					else if(j==0)
					{
						(*VEL)[j] = corners[0];
					}
					else if(j==((N*M)-1))
					{
						(*VEL)[j] = (corners[0] * -1.0);
					}
					else if(j==(N-1))
					{
						(*VEL)[j] = corners[1];
					}
					else if(j==(N*(M-1)))
					{
						(*VEL)[j] = (corners[1] * -1.0);
					}
				}

				A = n-1 + m*(N-1);
				if(n>0)
				{
					(*VEL)[j].x -= HS[A]*cos(HT[A]);
					(*VEL)[j].y -= HS[A]*sin(HT[A]);
				}
				if(n<(N-1))
				{
					(*VEL)[j].x += HS[A+1]*cos(HT[A+1]);
					(*VEL)[j].y += HS[A+1]*sin(HT[A+1]);
				}
				if(m>0)
				{
					(*VEL)[j].x -= VS[j-N]*cos(VT[j-N]);
					(*VEL)[j].y -= VS[j-N]*sin(VT[j-N]);
				}
				if(m<(M-1))
				{
					(*VEL)[j].x += VS[j]*cos(VT[j]);
					(*VEL)[j].y += VS[j]*sin(VT[j]);
				}

				if(calculate_diagonal_forces)
				{
					B = (n-1+((m-1)*(N-1)))*2;

					if(n>0 && m>0)
					{
						(*VEL)[j].x -= AS[B]*cos(AT[B]);
						(*VEL)[j].y -= AS[B]*sin(AT[B]);
					}
					if(n>0 && m<(M-1))
					{
						(*VEL)[j].x -= AS[N*2+B-1]*cos(AT[N*2+B-1]);
						(*VEL)[j].y -= AS[N*2+B-1]*sin(AT[N*2+B-1]);
					}
					if(n<(N-1) && m>0)
					{
						(*VEL)[j].x += AS[B+3]*cos(AT[B+3]);
						(*VEL)[j].y += AS[B+3]*sin(AT[B+3]);
					}
					if(n<(N-1) && m<(M-1))
					{
						(*VEL)[j].x += AS[N*2+B]*cos(AT[N*2+B]);
						(*VEL)[j].y += AS[N*2+B]*sin(AT[N*2+B]);
					}
				}
				
				(*VEL)[j] *= (kpos/mass);
				
				//temporarily revoked air resistance, for testing integrators
				if(myEntSystem->INTEGRATOR != 1 && myEntSystem->INTEGRATOR != 2) {
					(*VEL)[j] -= (*POS)[j] * friction;	//air resistance (calculated from velocity at start of frame, stored earlier in *POS[j])
				}
				
				if(affected_by_gravity) {
					(*VEL)[j].x += myEntSystem->gravity.x;
					(*VEL)[j].y += myEntSystem->gravity.y;
				}
				
				(*VEL)[j] += TempVelocities[j];

	//			(*VEL)[j].x = CLAMP((*VEL)[j].x,-100.0,100.0);	//not a good idea
	//			(*VEL)[j].y = CLAMP((*VEL)[j].y,-100.0,100.0);	//a slightly better (still bad) idea would be to clamp the change in position...

				(*POS)[j] *= deltaT;	//needs to be the last thing, just before collision detection
				(*VEL)[j] *= deltaT;

				
				if(edges_are_fixed || left_edge_only_fixed)
				{
					if(N > 1 && (n==0 || n==(N-1)))
					{
						if((left_edge_only_fixed && n==0) || left_edge_only_fixed==false) {
							(*POS)[j].Nullify();
							(*VEL)[j].Nullify();
						}
					}
					if(left_edge_only_fixed==false && M > 1 && (m==0 || m==(M-1)))
					{
						(*POS)[j].Nullify();
						(*VEL)[j].Nullify();
					}
				}

//------------------------------------------------------------------------------------
				if(!myEntSystem->allLines.empty())
				{
					std::list<Hit> allHits;

					std::list<WorldLine*>::iterator WLineIter = myEntSystem->allLines.begin();
					for( ; WLineIter!=myEntSystem->allLines.end(); WLineIter++)
					{
						if(hitCheck = cTools::getHit( *(*WLineIter), line(positions[j], positions[j] + (*POS)[j])  ))
							allHits.push_back(hitCheck);
					}

					if(!allHits.empty())
					{
						allHits.sort();
						
						uv.x = (allHits.begin()->dist / (*POS)[j].getLength());

						//positions[j] = allHits.begin()->pt;

						(*POS)[j].x = (*POS)[j].y = 0.0;//uv.x;
						(*VEL)[j].x = (*VEL)[j].y = 0.0;//uv.x;
					}
				}
//------------------------------------------------------------------------------------
			}
		}
		
	}

	void ent_rectangle::applyImpulse(const point& Location, const point& Impulse)
	{
		JPHYS_FLOAT_UNIT fDist = 0.25*(eqdist*eqdist); //allowed max distance from force location

		JPHYS_FLOAT_UNIT tempDist = 0.0;

		for(int j=0; j<N*M; j++)
		{
			tempDist = cTools::getDistSquared(positions[j], Location);

			if(tempDist <= fDist)
				TempVelocities[j] += (Impulse / mass) * (fDist - tempDist);
		}
	}

}
