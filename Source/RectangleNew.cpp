
#include "stdafx.h"
#include "RectangleNew.h"
#include "Integrators.h"
#include "mathTools.h"
#include "Games/CollisionTools.h"
#include "Games/EntityEdgeLine.h"

#include "GLVertexes.h"
#include "clamp.h"


namespace phys
{
using namespace mathTools; //from math_constants.h
using namespace drawing; //from GLVertexes.h


	ent_rectangle_new::ent_rectangle_new()
	{
		myEntSystem = NULL;
		checkBounds = 0;
	}

	ent_rectangle_new::ent_rectangle_new(JPHYS_FLOAT_UNIT NN, JPHYS_FLOAT_UNIT MM, EntitySystem* entSys, SpawnParams_Soft params)
	{
		NN = floor(fabs(NN)+0.01);
		MM = floor(fabs(MM)+0.01); //make sure user gave whole-number widths/heights
		checkBounds = 0;
		
		if(cmpFloatToZero(NN) || cmpFloatToZero(MM)) {NN = 2.0; MM = 2.0;}
		
		CenterPos = params.entParams.pos_center;

		params.entParams.pos_center.x -= (NN-1)*params.vertex_equilibrium_distance*0.5;
		params.entParams.pos_center.y -= (MM-1)*params.vertex_equilibrium_distance*0.5;

		N = ((static_cast<unsigned int>(NN))*(static_cast<unsigned int>(MM)));

		 positions.resize(N);
		velocities.resize(N);
		
		myEntSystem = entSys;
		drawHollow = 1;

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

				N++;
			}
		}
		N = int(NN+0.2); M = int(MM+0.2);

		mass = params.entParams.mass_total / (NN*MM);
		kpos = params.springconst_linear;
		eqdist = params.vertex_equilibrium_distance;
		friction = params.entParams.air_resistance;
		color[0] = params.entParams.color[0];
		color[1] = params.entParams.color[1];
		color[2] = params.entParams.color[2];
		
		int j,m;	//initialize the edges of this entity, for collisions with other entities
		m = (N-1);
		for(j=0; j<m; j++)
		{
			DangerLines.push_back(EntityEdgeLine(&positions[j],  &velocities[j],
												 &positions[j+1],&velocities[j+1])); //bottom side
		}
		m = (M*N)-1;
		for(j=N*(M-1); j<m; j++)
		{
			DangerLines.push_back(EntityEdgeLine(&positions[j],  &velocities[j],
												 &positions[j+1],&velocities[j+1])); //top side
		}

		m = (M*N)-1-N;
		for(j=0; j<m; )
		{
			DangerLines.push_back(EntityEdgeLine(&positions[j],  &velocities[j],
												 &positions[j+N],&velocities[j+N])); //left side
				
			DangerLines.push_back(EntityEdgeLine(&positions[j+N-1],  &velocities[j+N-1],
												 &positions[j+N+N-1],&velocities[j+N+N-1])); //right side
			j += N;
		}
	}

	void ent_rectangle_new::draw()
	{
		int j,m,n;

		glBegin(GL_LINES);
		glColor4ub(color[0],color[1],color[2], 255);
		
		if(drawHollow <= 1)
		{
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
		}
		else if(drawHollow==2)
		{
			m = DangerLines.size();
			unsigned char COLORR[3];
			COLORR[0] = 255;
			COLORR[1] = 1;
			COLORR[2] = 2;
			for(j=0; j<m; j++)
			{
				DangerLines[j].draw(COLORR);
			}
			/*
			m = (N-1);
			for(j=0; j<m; j++)
			{
				GLVERT2(positions[j+1]);
				GLVERT2(positions[j]);
			}
			m = (M*N)-1;
			for(j=N*(M-1); j<m; j++)
			{
				GLVERT2(positions[j+1]);
				GLVERT2(positions[j]);
			}

			m = (M*N)-1-N;
			for(j=0; j<m; )
			{
				GLVERT2(positions[j  ]);
				GLVERT2(positions[j+N]); //left side
				
				GLVERT2(positions[j   +N-1]);
				GLVERT2(positions[j+N +N-1]); //right side

				j += N;
			}
			*/
		}
		else
		{
			glEnd();
			glBegin(GL_QUADS);
			glColor4ub(color[0],color[1],color[2], 255);
			m = (N-1)*(M-1);
			for(j=0; j<m; j++)
			{
				n = j + (j-(j%(N-1)))/(N-1);
						
				GLQUAD2(positions[n], positions[n+1], positions[n+N+1], positions[n+N]);
			}
			glEnd();
			glBegin(GL_LINES);
			glColor4ub(color[0],color[1],color[2], 255);
			
			if(drawHollow==4)
			{
				glColor4ub(1,1,1,255);
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
			}
		}
		
		if(drawHollow==0)
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
	}
	void ent_rectangle_new::drawWithOffset(const point& offset, const point& aspectStretch)
	{
		std::cout << "HEY! DON'T CALL \"drawWithOffset\", use OpenGL matrices to push/pop draw matrices and deal with it that way!" << std::endl;
	}

	void ent_rectangle_new::UpdateCenterPos()
	{
		CenterPos.x = 0.0; CenterPos.y = 0.0;

		int Size = positions.size();

		for(int a=0; a<Size; a++)
		{
			CenterPos += positions[a];
		}

		CenterPos /= ((JPHYS_FLOAT_UNIT)(N*M));
	}

	void ent_rectangle_new::integRK4(std::vector<point> & POSITIONS, std::vector<point> & VELOCITIES, JPHYS_FLOAT_UNIT hhh)
	{
		unsigned int j, size = static_cast<unsigned int>(positions.size());

		std::vector< std::vector<point> > kPos(4, std::vector<point>(size)); //make 4 big vectors, each with #size# parameters
		std::vector< std::vector<point> > kVel(4, std::vector<point>(size)); //make 4 big vectors, each with #size# parameters
		
		for(j=0; j<size; j++) { kPos[0][j] = positions[j];
								kVel[0][j] = velocities[j]; }
		motionFunc(kPos[0], kVel[0]);
		
		for(j=0; j<size; j++) { kPos[1][j] =  positions[j] + ((kPos[0][j]*=hhh)/2.0);
								kVel[1][j] = velocities[j] + ((kVel[0][j]*=hhh)/2.0); }
		motionFunc(kPos[1], kVel[1]);
		
		for(j=0; j<size; j++) { kPos[2][j] =  positions[j] + ((kPos[1][j]*=hhh)/2.0);
								kVel[2][j] = velocities[j] + ((kVel[1][j]*=hhh)/2.0); }
		motionFunc(kPos[2], kVel[2]);
		
		for(j=0; j<size; j++) { kPos[3][j] =  positions[j] + (kPos[2][j]*=hhh);
								kVel[3][j] = velocities[j] + (kVel[2][j]*=hhh); }
		motionFunc(kPos[3], kVel[3]);
		
		for(j=0; j<size; j++) { POSITIONS[j] += ((kPos[0][j] + kPos[1][j]*2.0 + kPos[2][j]*2.0 + (kPos[3][j]*hhh)) / 6.0);
								VELOCITIES[j] += ((kVel[0][j] + kVel[1][j]*2.0 + kVel[2][j]*2.0 + (kVel[3][j]*hhh)) / 6.0); }
	}

	void ent_rectangle_new::motionFunc(std::vector<point> & POS, std::vector<point> & VEL)
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
		
			uv = POS[(N*M)-1] - POS[0];
			corners[0].y = atan2(uv.y, uv.x);
			uv.x = uv.getLength();
			corners[1].x = eqdist*sqrt(((JPHYS_FLOAT_UNIT)((N-1)*(N-1)))+((JPHYS_FLOAT_UNIT)((M-1)*(M-1))));

			corners[0].x = (uv.x - corners[1].x) * cos(corners[0].y);
			corners[0].y = (uv.x - corners[1].x) * sin(corners[0].y);
			
			uv = POS[N*(M-1)] - POS[N-1];
			corners[1].y = atan2(uv.y, uv.x);
			uv.x = uv.getLength();
			uv.y = corners[1].x;

			corners[1].x = (uv.x - uv.y) * cos(corners[1].y);
			corners[1].y = (uv.x - uv.y) * sin(corners[1].y);

		//------------------------
		
		m = M*(N-1);
		for(j=0; j<m; j++)
		{
			n = j%(N-1) + N*(j-(j%(N-1)))/(N-1);
			uv = POS[n+1] - POS[n];
			HS[j] = uv.getLength() - eqdist;
			HT[j] = atan2(uv.y,uv.x);
		}

		m = N*(M-1);
		for(j=0; j<m; j++)
		{
			uv = POS[N+j] - POS[j];
			VS[j] = uv.getLength() - eqdist;
			VT[j] = atan2(uv.y,uv.x);
		}
		
		m = (N-1)*(M-1);
		for(j=0; j<m; j++)
		{
			n = j + (j-(j%(N-1)))/(N-1);
			uv = POS[N+n+1] - POS[n];
			AS[j*2] = uv.getLength() - (eqdist*ROOT_TWO);
			AT[j*2] = atan2(uv.y,uv.x);

			uv = POS[n+1] - POS[N+n];
			AS[j*2+1] = uv.getLength() - (eqdist*ROOT_TWO);
			AT[j*2+1] = atan2(uv.y,uv.x);
		}

		int A,B;
		j = -1;
		for(m=0; m<M; m++)
		{
			for(n=0; n<N; n++)
			{
				j++;
				POS[j] = VEL[j];

				if(j!=0 && j!=(N-1) && j!=(N*(M-1)) && j!=((N*M)-1))
				{
					VEL[j].x = 0.0; VEL[j].y = 0.0;
				}
				else if(j==0)
				{
					VEL[j] = corners[0];
				}
				else if(j==((N*M)-1))
				{
					VEL[j] = (corners[0] * -1.0);
				}
				else if(j==(N-1))
				{
					VEL[j] = corners[1];
				}
				else if(j==(N*(M-1)))
				{
					VEL[j] = (corners[1] * -1.0);
				}

				A = n-1 + m*(N-1);
				B = (n-1+((m-1)*(N-1)))*2;
				if(n>0)
				{
					VEL[j].x -= HS[A]*cos(HT[A]);
					VEL[j].y -= HS[A]*sin(HT[A]);
				}
				if(n<(N-1))
				{
					VEL[j].x += HS[A+1]*cos(HT[A+1]);
					VEL[j].y += HS[A+1]*sin(HT[A+1]);
				}
				if(m>0)
				{
					VEL[j].x -= VS[j-N]*cos(VT[j-N]);
					VEL[j].y -= VS[j-N]*sin(VT[j-N]);
				}
				if(m<(M-1))
				{
					VEL[j].x += VS[j]*cos(VT[j]);
					VEL[j].y += VS[j]*sin(VT[j]);
				}
				if(n>0 && m>0)
				{
					VEL[j].x -= AS[B]*cos(AT[B]);
					VEL[j].y -= AS[B]*sin(AT[B]);
				}
				if(n>0 && m<(M-1))
				{
					VEL[j].x -= AS[N*2+B-1]*cos(AT[N*2+B-1]);
					VEL[j].y -= AS[N*2+B-1]*sin(AT[N*2+B-1]);
				}
				if(n<(N-1) && m>0)
				{
					VEL[j].x += AS[B+3]*cos(AT[B+3]);
					VEL[j].y += AS[B+3]*sin(AT[B+3]);
				}
				if(n<(N-1) && m<(M-1))
				{
					VEL[j].x += AS[N*2+B]*cos(AT[N*2+B]);
					VEL[j].y += AS[N*2+B]*sin(AT[N*2+B]);
				}

				VEL[j] *= kpos;//(kpos/mass);

				//temporarily revoked air resistance, for testing integrators
				VEL[j] -= POS[j] * friction;	//air resistance (calculated from velocity at start of frame, stored earlier in *POS[j])

				VEL[j].x += myEntSystem->gravity.x;
				VEL[j].y += myEntSystem->gravity.y;
				
				//VEL[j] += TempVelocities[j];
			}
		}
	}

	void ent_rectangle_new::checkVertexCollisions(int vertex1, int vertex2)
	{
		point Offset; //double the average offset of the two vertices (approximates the motion of the line, and is 2x sensitive)
		line edgeSurface;

		std::list<WorldLine*>::iterator Iter = myEntSystem->allLines.begin();
		for(; Iter != myEntSystem->allLines.end(); Iter++)
		{
			if(cTools::checkIntersect(line(positions[vertex1], temp_positions[vertex1]), (*(*Iter))))
			{
				temp_positions[vertex1] = positions[vertex1]; //we decide not to move at all because of collision

				cTools::collisionRotation(temp_velocities[vertex1], (*(*Iter)));

				temp_velocities[vertex1] *= clamp((0.8*friction), 0.0, 1.0);

				//break; //no more collisions after this (even if it wasn't the first thing hit)
			}

			Offset = (temp_positions[vertex1] - positions[vertex1]) + (temp_positions[vertex2] - positions[vertex2]);
			edgeSurface = line(positions[vertex1], positions[vertex2]);

			if(cTools::checkIntersect(line((*Iter)->p1, (*Iter)->p1 - Offset), edgeSurface))
			{
				temp_positions[vertex1] = positions[vertex1];
				temp_positions[vertex2] = positions[vertex2]; //we decide not to move at all because of collision
				temp_velocities[vertex1] *= -1.0;
				temp_velocities[vertex2] *= -1.0; //move in "reverse" direction

			//	temp_velocities[vertex1] *= clamp((0.8*friction), 0.0, 1.0);
			//	temp_velocities[vertex2] *= clamp((0.8*friction), 0.0, 1.0);

				//break;
			}
			if(cTools::checkIntersect(line((*Iter)->p2, (*Iter)->p2 - Offset), edgeSurface))
			{
				temp_positions[vertex1] = positions[vertex1];
				temp_positions[vertex2] = positions[vertex2]; //we decide not to move at all because of collision
				temp_velocities[vertex1] *= -1.0;
				temp_velocities[vertex2] *= -1.0; //move 2x in "reverse" direction
				
			//	temp_velocities[vertex1] *= clamp((0.8*friction), 0.0, 1.0);
			//	temp_velocities[vertex2] *= clamp((0.8*friction), 0.0, 1.0);

				//break;
			}
		}
		
		/*
			if(cTools::checkIntersect(line(positions[n],positions[n]-offset), hitSurface))
			{
				positions[n] += offset;

				velocities[n] += (offset / std::max(0.10,myEntSystem->curFrameTime)); //collision affects velocity, but not too much
			}
		*/
	}
	
	void ent_rectangle_new::update(JPHYS_FLOAT_UNIT frametime)
	{
		if(myEntSystem==NULL)
			return;
		
		checkBounds++;
		if(checkBounds == 10) //check to delete it only once every 10 frames ("garbage collection" doesn't need to be immediate)
		{
			checkBounds = 0;
			UpdateCenterPos();
		}
		
		int j=positions.size();

		temp_positions.resize(j);
		temp_positions = positions;
		temp_velocities.resize(j);
		temp_velocities = velocities;

		integRK4(temp_positions, temp_velocities, frametime);
		

		point Offset;
		int m = (N-1);
		for(j=0; j<m; j++)
		{
			checkVertexCollisions(j, j+1); //bottom side
		}
		m = (M*N)-1;
		for(j=N*(M-1); j<m; j++)
		{
			checkVertexCollisions(j, j+1); //top side
		}

		m = (M*N)-1-N;
		for(j=0; j<m; ) //these are redundant for the corner vertices, which are checked twice...
		{
			checkVertexCollisions(j, j+N); //left side

			checkVertexCollisions(j+N-1, j+N+N-1); //right side
			j += N;
		}

		for(j=0; j<(M*N); j++)
		{
			positions[j] = temp_positions[j];	//move it -after- we checked that we didn't collide with anything
			velocities[j] = temp_velocities[j];
		}
	}
	
	//Check collisions against another moving object nearby, by using a frame of reference in which
	 //this particle is moving and the collidee is static
	void ent_rectangle_new::checkCollisions(const line &hitSurface, const point& offset)
	{
		int NNN = (N*M);
		for(int n=0; n<NNN; n++)
		{
			//Hit pH = cTools::getHitOrDist(line(positions[n],positions[n]-offset), hitSurface);
			//if(pH.b)
			if(cTools::checkIntersect(line(positions[n],positions[n]-offset), hitSurface))
			{
				positions[n] += offset;

				velocities[n] += (offset / std::max(0.10,myEntSystem->curFrameTime)); //collision affects velocity, but not too much
			}
		}
	}

	void ent_rectangle_new::applyImpulse(const point& Location, const point& Impulse)
	{
		JPHYS_FLOAT_UNIT fDist = 1.2*eqdist; //allowed max distance from force location

		JPHYS_FLOAT_UNIT tempDist = 0.0;
		int maxn = N*M;

		for(int j=0; j<maxn; j++)
		{
			tempDist = cTools::getDist(positions[j], Location);

			if(tempDist <= fDist)
				velocities[j] += (Impulse / mass) * (fDist - tempDist);
		}
	}

}
