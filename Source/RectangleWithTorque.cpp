/*
 * Author: Jason Bunk
 * Web page: http://sites.google.com/site/jasonbunk
 * License: The MIT License (MIT) (Expat)
 * Copyright (c) 2015 Jason Bunk
 */

#include "stdafx.h"
#include "RectangleWithTorque.h"
#include "mathTools.h"
#include "GLVertexes.h"
#include "Integrators.h"
#include <iostream>
#include "Games/CollisionTools.h"
#include "clamp.h"


#define ONLY_DO_2x2_SQUARE 1



namespace phys
{

inline JPHYS_FLOAT_UNIT constrainAngle_to_radians_zero_to_twopi(JPHYS_FLOAT_UNIT x)
{
    x = fmod(x, mathTools::TWO_PI);
    if (x < 0.0)
        x += mathTools::TWO_PI;
    return x;
}

inline JPHYS_FLOAT_UNIT constrainAngle_to_radians_negativepi_to_pi(JPHYS_FLOAT_UNIT x)
{
	x = fmod(x + mathTools::PI, mathTools::TWO_PI);
    
	if (x < 0.0)
        x += mathTools::TWO_PI;
    
	return x - mathTools::PI;
}



using namespace mathTools; //from math_constants.h
using namespace drawing; //from GLVertexes.h


ent_rectangle_torque::ent_rectangle_torque()
{
	myEntSystem = nullptr;
	drawHollow = true;
}

ent_rectangle_torque::ent_rectangle_torque(JPHYS_FLOAT_UNIT NN, JPHYS_FLOAT_UNIT MM, EntitySystem* entSys, SpawnParams_Soft params)
{
	NN = fabs(floor(NN));
	MM = fabs(floor(MM)); //make sure user gave whole-number widths/heights
		
	if(cmpFloatToZero(NN) || cmpFloatToZero(MM)) {NN = 2.0; MM = 2.0;}
		
	CenterPos = params.entParams.pos_center;

	params.entParams.pos_center.x -= (NN-1)*params.vertex_equilibrium_distance*0.5;
	params.entParams.pos_center.y -= (MM-1)*params.vertex_equilibrium_distance*0.5;

	N = ((static_cast<unsigned int>(NN))*(static_cast<unsigned int>(MM)));

#if ONLY_DO_2x2_SQUARE
	positions.resize(6);
	velocities.resize(6);
	TempVelocities.resize(6);
#else
	positions.resize(N);
	velocities.resize(N);
	TempVelocities.resize(N);
#endif
	
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
			

			/*
			//give some of them an initial impulse
			if(N > 3)
				velocities[N] = params.entParams.velocity;
			else
				velocities[N] = (params.entParams.velocity*-19.0);
			*/

			N++;
		}
	}

	N = int(NN+0.2); M = int(MM+0.2);

	mass = params.entParams.mass_total / (NN*MM);
	
	kpos = params.springconst_linear;
	eqdist = params.vertex_equilibrium_distance;
	
	kang = params.springconst_angular;
	eqang = params.vertex_equilibrium_angle;

	kvel = params.entParams.air_resistance;

	color[0] = params.entParams.color[0];
	color[1] = params.entParams.color[1];
	color[2] = params.entParams.color[2];
	
	/*
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
	*/
}

void ent_rectangle_torque::GiveStartingFlick(JPHYS_FLOAT_UNIT kick_vel)
{
	int maxj = RoundFloatToInt(0.24937238752 * static_cast<JPHYS_FLOAT_UNIT>(N)) + 1; //the JPHYS_FLOAT_UNIT is very arbitrary
	int jback = ((N*M) - 1);
	
	UpdateCenterPos();

#if 0
	positions[0].x += kick_vel;
	positions[0].y += kick_vel;

	positions[2].x -= kick_vel; //upper left kicked left
	positions[1].y -= kick_vel; //bottom right kicked down
#else
	velocities[0].x += kick_vel;
	velocities[0].y += kick_vel;

	velocities[2].x -= kick_vel; //upper left kicked left
	velocities[1].y -= kick_vel; //bottom right kicked down
#endif

	UpdateCenterPos();

	/*
	for(int jfront=0; jfront<maxj; )
	{
		velocities[jfront].x += kick_vel;
		velocities[jback].x -= kick_vel;
		
		velocities[jfront].y += kick_vel;
		velocities[jback].y -= kick_vel;

		jfront++;
		jback--;
	}
	*/
}

void ent_rectangle_torque::UpdateCenterPos()
{
	point totalvel;


	CenterPos.x = 0.0; CenterPos.y = 0.0;

#if ONLY_DO_2x2_SQUARE
	for(int a=0; a<3; a++)
	{
		CenterPos += positions[a];

		totalvel += velocities[a];

		TempVelocities[a].Nullify();
	}
	CenterPos /= 3.0;
#else
	int Size = positions.size();
	for(int a=0; a<Size; a++)
	{
		CenterPos += positions[a];

		totalvel += velocities[a];

		TempVelocities[a].Nullify();
	}
	CenterPos /= ((JPHYS_FLOAT_UNIT)(N*M));
#endif
}


bool ent_rectangle_torque::update(JPHYS_FLOAT_UNIT frametime)
{
	if(myEntSystem == nullptr)
		return true;

	integrate::RK4(this, frametime, &positions, &velocities);

	UpdateCenterPos();
	return false;
}
	
void ent_rectangle_torque::draw()
{
	int j,m,n;

	glBegin(GL_LINES);
	glColor4ub(color[0],color[1],color[2], 255);

#if ONLY_DO_2x2_SQUARE
	m = M*(N-1);
	for(j=0; j<m; j++)
	{
		n = j%(N-1) + N*(j-(j%(N-1)))/(N-1);

		if(n < 2)
		{
			GLVERT2(positions[n+1]);
			GLVERT2(positions[n]);
		}
	}

	m = N*(M-1);
	for(j=0; j<m; j++)
	{
		if(j != 1)
		{
			GLVERT2(positions[N+j]);
			GLVERT2(positions[j  ]);
		}
	}

	
	for(j=0; j<((N*M)-1); j++)
	{
		GLVERT2(positions[j]);
		GLVERT2(positions[j] + (velocities[j] * 0.1));
	}

	glColor4ub(255,1,1,255); //now the torques
	
	GLVERT2(positions[1]);
	GLVERT2(positions[1] + (velocities[5] * 1.0));
	
	GLVERT2(positions[2]);
	GLVERT2(positions[2] + (velocities[4] * 1.0));
	
	velocities[4].Nullify();
	velocities[5].Nullify();

	
	glEnd();

	glBegin(GL_POINTS);
	glColor4ub(color[0],color[1],color[2], 255);
	GLVERT2(CenterPos);

#else
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
#endif
	
	/*
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
			
		GLVERT2(positions[(N*M)-1]);
		GLVERT2(positions[0      ]);
			
		GLVERT2(positions[N*(M-1)]);
		GLVERT2(positions[N-1    ]);
	}
	*/

	glEnd();
}

void ent_rectangle_torque::drawWithOffset(const point& offset, const point& aspectStretch)
{
	draw();
	std::cout << "Why are you using \"drawWithOffset\"?? -Use OpenGL- to shift the whole drawing scene!" << std::endl;
}

void ent_rectangle_torque::f( std::vector<point> *POS, std::vector<point> *VEL, JPHYS_FLOAT_UNIT deltaT )
{
/*
p1.x = &p1.x;
p1.y = &p1.y;
vel1.x = &vel1.x;
vel1.y = &vel1.y;
p2.x = &p2.x;
p2.y = &p2.y;
vel2.x = &vel2.x;
vel2.y = &vel2.y;
originp.x = &originp.x;
originp.y = &originp.y;
originvel.x = &originvel.x;
originvel.y = &originvel.y;

	double S1 = sqrt((p1.x-originp.x)*(p1.x-originp.x) + (p1.y-originp.y)*(p1.y-originp.y));
	double T1 = atan2(p1.y-originp.y,p1.x-originp.x);
	double S2 = sqrt((p2.x-originp.x)*(p2.x-originp.x) + (p2.y-originp.y)*(p2.y-originp.y));
	double T2 = atan2(p2.y-originp.y,p2.x-originp.x);

	p1.x = vel1.x;
	p1.y = vel1.y;
	p2.x = vel2.x;
	p2.y = vel2.y;

	double FF = T1-T2;
	if(FF < 0.0)		//the measured angle between the two vectors should be 0 < deltatheta < 2*pi
		FF += TWO_PI;
	FF = kang*(FF-eqang);

	double t1x = (FF*cos(T1+fONE_HALF_PI))/S1;
	double t1y = (FF*sin(T1+fONE_HALF_PI))/S1;

	double t2x = (FF*cos(T2+fONE_HALF_PI))/S2;
	double t2y = (FF*sin(T2+fONE_HALF_PI))/S2;

	vel1.x = FF*S1*cos(T1-ONE_HALF_PI);
	vel1.y = FF*S1*sin(T1-ONE_HALF_PI);

	vel2.x = -FF*S2*cos(T2-ONE_HALF_PI);
	vel2.y = -FF*S2*sin(T2-ONE_HALF_PI);

	originp.x = originvel.x;
	originp.y = originvel.y;
	originvel.x = -(vel1.x) - (vel2.x) - t1x/m + t2x/m - kvel*originp.x/m;
	originvel.y = -(vel1.y) - (vel2.y) - t1y/m + t2y/m - kvel*originp.y/m;

	vel1.x +=  t1x/m - kvel*p1.x/m;
	vel1.y +=  t1y/m - kvel*p1.y/m;

	vel2.x += -t2x/m - kvel*p2.x/m;
	vel2.y += -t2y/m - kvel*p2.y/m;
*/

	std::vector<JPHYS_FLOAT_UNIT> HS((N-1)*M); // "horizontal" connections, distance
	std::vector<JPHYS_FLOAT_UNIT> HT((N-1)*M); // "horizontal" connections, angle
	std::vector<JPHYS_FLOAT_UNIT> VS((M-1)*N);
	std::vector<JPHYS_FLOAT_UNIT> VT((M-1)*N);

	int j,n,mm;
	point uv;


	mm = M*(N-1);
	for(j=0; j<mm; j++)	//"horizontal" connections
	{
		n = j%(N-1) + N*(j-(j%(N-1)))/(N-1);
		uv = (*POS)[n+1] - (*POS)[n];
		HS[j] = uv.getLength();
		HT[j] = atan2(uv.y,uv.x);
	}
	
	mm = N*(M-1);
	for(j=0; j<mm; j++) //"vertical" connections
	{
		uv = (*POS)[j+N] - (*POS)[j];
		VS[j] = uv.getLength();
		VT[j] = atan2(uv.y,uv.x);
	}
	
	point torque1,torque2;
	

#if ONLY_DO_2x2_SQUARE
	(*VEL)[4].Nullify();
	(*VEL)[5].Nullify();
#endif


	int A,B;
	j = -1;
	for(mm=0; mm<M; mm++)
	{
		for(n=0; n<N; n++)
		{
			j++;
			(*POS)[j] = (*VEL)[j];
			(*VEL)[j].Nullify();

			A = n-1 + mm*(N-1);
			B = (n-1+((mm-1)*(N-1)))*2;

#if ONLY_DO_2x2_SQUARE
			if(j == 1)
			{
				if(n>0)						//add linear spring connection to the left
				{
					(*VEL)[j].x -= (HS[A]-eqdist)*cos(HT[A]);
					(*VEL)[j].y -= (HS[A]-eqdist)*sin(HT[A]);
				}
			}
				
			if(j == 0)
			{
				if(n<(N-1))					//add linear spring connection to the right
				{
					(*VEL)[j].x += (HS[A+1]-eqdist)*cos(HT[A+1]);
					(*VEL)[j].y += (HS[A+1]-eqdist)*sin(HT[A+1]);
				}
			}

			if(j == 2)
			{
				if(mm>0)					//add linear spring connection below
				{
					(*VEL)[j].x -= (VS[j-N]-eqdist)*cos(VT[j-N]);
					(*VEL)[j].y -= (VS[j-N]-eqdist)*sin(VT[j-N]);
				}
			}

			if(j == 0)
			{
				if(mm<(M-1))				//add linear spring connection above
				{
					(*VEL)[j].x += (VS[j]-eqdist)*cos(VT[j]);
					(*VEL)[j].y += (VS[j]-eqdist)*sin(VT[j]);
				}
			}
#else
			if(n>0)						//add linear spring connection to the left
			{
				(*VEL)[j].x -= (HS[A]-eqdist)*cos(HT[A]);
				(*VEL)[j].y -= (HS[A]-eqdist)*sin(HT[A]);
			}
			if(n<(N-1))					//add linear spring connection to the right
			{
				(*VEL)[j].x += (HS[A+1]-eqdist)*cos(HT[A+1]);
				(*VEL)[j].y += (HS[A+1]-eqdist)*sin(HT[A+1]);
			}
			if(mm>0)					//add linear spring connection below
			{
				(*VEL)[j].x -= (VS[j-N]-eqdist)*cos(VT[j-N]);
				(*VEL)[j].y -= (VS[j-N]-eqdist)*sin(VT[j-N]);
			}
			if(mm<(M-1))				//add linear spring connection above
			{
				(*VEL)[j].x += (VS[j]-eqdist)*cos(VT[j]);
				(*VEL)[j].y += (VS[j]-eqdist)*sin(VT[j]);
			}
#endif
			
			(*VEL)[j] *= (kpos/mass);
			
			//temporarily revoked air resistance, for testing integrators
			//(*VEL)[j] -= (*POS)[j] * friction;	//air resistance (calculated from velocity at start of frame, stored earlier in *POS[j])

			(*VEL)[j].x += myEntSystem->gravity.x;
			(*VEL)[j].y += myEntSystem->gravity.y;
			
		//	(*VEL)[j] += TempVelocities[j];

//------------------------------------------------------------------------------------
			
			
			if(n<(N-1) && mm<(M-1))		//add angular torque, with right & above
			{							//THIS CORNER IS DONE DONE DONE YAYYYY (thursday april 19 2012 at 7:30pm)

				uv.x = minimum(constrainAngle_to_radians_zero_to_twopi(VT[j] - HT[A+1]), constrainAngle_to_radians_zero_to_twopi(HT[A+1] - VT[j]));	//above - right
				
				uv.x = constrainAngle_to_radians_negativepi_to_pi(uv.x);

				uv.x = kang*(uv.x - eqang); //u is magnitude of the torque to be applied


				torque1.x = (-sin(VT[j])) * uv.x * VS[j] / mass;	//torque1 is on the "vertical" bar
				torque1.y = ( cos(VT[j])) * uv.x * VS[j] / mass;
				
				torque2.x = ( sin(HT[A+1])) * uv.x * HS[A+1] / mass;  //torque2 is on the "horizontal" bar
				torque2.y = (-cos(HT[A+1])) * uv.x * HS[A+1] / mass;


				(*VEL)[j] += (torque1 + torque2); //center
				(*VEL)[j+N] -= torque1; //above
				(*VEL)[j+1] -= torque2; //right



				(*VEL)[4] = torque1;
				(*VEL)[5] = torque2;


				/*
				v = -u*sin(VT[j])/VS[j]; //t1x		//u = tao;  v = tao/r = F
				w = u*cos(VT[j])/VS[j]; //t1y

				y = u*sin(HT[A+1])/HS[A+1]; //t2x
				z = -u*cos(HT[A+1])/HS[A+1]; //t2y

				(*VEL)[j].x += y/mass + v/mass;	//center
				(*VEL)[j].y += z/mass + w/mass;

				(*VEL)[j+N].x -=  v/mass;	//above
				(*VEL)[j+N].y -=  w/mass;
				(*VEL)[j+1].x -=  y/mass;	//right
				(*VEL)[j+1].y -=  z/mass;
				*/
			}
			
#if 0
			if(n<(N-1) && mm<(M-1))		//add angular torque, with right & above
			{							//THIS CORNER IS DONE DONE DONE YAYYYY (thursday april 19 2012 at 7:30pm)

				u = VT[j]-HT[A+1];	//above - right
				if(u < 0.0)
					u += fTWO_PI;
				u = kang*(u-eqang); //u is magnitude of the torque to be applied

				v = -u*sin(VT[j])/VS[j]; //t1x		//u = tao;  v = tao/r = F
				w = u*cos(VT[j])/VS[j]; //t1y

				y = u*sin(HT[A+1])/HS[A+1]; //t2x
				z = -u*cos(HT[A+1])/HS[A+1]; //t2y

				(*x)[j*4+2] += y/m + v/m;	//center
				(*x)[j*4+3] += z/m + w/m;

				(*x)[(j+N)*4+2] -=  v/m;	//above
				(*x)[(j+N)*4+3] -=  w/m;
				(*x)[(j+1)*4+2] -=  y/m;	//right
				(*x)[(j+1)*4+3] -=  z/m;

			}
			if(n<(N-1) && mm>0)			//add angular torque, with right & below
			{							//THIS CORNER DONE?!!!! (wednesday may 30 2012 at 10:18pm)

				u = HT[A+1]-VT[j-N];	//right - below
				if(u < 0.0)
					u += fTWO_PI;	// the angle between needs to be from 0 to 2*pi
				u = kang*(u-eqang-fPI);

				v = -u*sin(HT[A+1])/HS[A+1]; //t1x, right
				w = u*cos(HT[A+1])/HS[A+1]; //t1y

				y = u*sin(VT[j-N])/VS[j-N]; //t2x			//u = tao;  v = tao/r = F
				z = -u*cos(VT[j-N])/VS[j-N]; //t2y, below

				(*x)[j*4+2] += y/m + v/m;	//center
				(*x)[j*4+3] += z/m + w/m;

				(*x)[(j+1)*4+2] -=  v/m;	//right
				(*x)[(j+1)*4+3] -=  w/m;
				(*x)[(j-N)*4+2] -=  y/m;	//below
				(*x)[(j-N)*4+3] -=  z/m;

			}
			if(n>0 && mm>0)				//add angular torque, with left & below
			{							//THIS CORNER DONE, (wednesday may 30 2012 at 10:33pm)

				u = VT[j-N]-HT[A];	//below - left
				if(u < 0.0)
					u += fTWO_PI;
				u = -kang*(u-eqang);

				v = -u*sin(VT[j-N])/VS[j-N]; //t1x			//u = tao;  v = tao/r = F
				w = u*cos(VT[j-N])/VS[j-N]; //t1y, below

				y = u*sin(HT[A])/HS[A]; //t2x, left
				z = -u*cos(HT[A])/HS[A]; //t2y

				(*x)[j*4+2] += y/m + v/m;	//center
				(*x)[j*4+3] += z/m + w/m;

				(*x)[(j-N)*4+2] -=  v/m;	//below
				(*x)[(j-N)*4+3] -=  w/m;
				(*x)[(j-1)*4+2] -=  y/m;	//left
				(*x)[(j-1)*4+3] -=  z/m;

			}
			if(n>0 && mm<(M-1))			//add angular torque, with left & above
			{
/*
				u = fPI+HT[A]-VT[j];	//left - above
				if(u < 0.0)
					u += fTWO_PI;
				u = (u-eqang);
				u *= kang;

				v = u*cos(HT[A])/HS[A]; //t1x			//u = tao;  v = tao/r = F
				w = -u*sin(HT[A])/HS[A]; //t1y, left

				y = u*sin(VT[j])/VS[j]; //t2x, above
				z = -u*cos(VT[j])/VS[j]; //t2y

				(*x)[j*4+2] += y/m + v/m;	//center
				(*x)[j*4+3] += z/m + w/m;

				(*x)[(j+N)*4+2] -=  v/m;	//above
				(*x)[(j+N)*4+3] -=  w/m;
				(*x)[(j-1)*4+2] -=  y/m;	//left
				(*x)[(j-1)*4+3] -=  z/m;
*/
			}

	//remember to add torques also along horizontal and vertical lines... so that you can make a 1x3 flexible line


//				if(py[j] < -15.0)
//				{
//					(*x)[j*4+3] -= 90*g;
//				}
//				else
//				{
				(*x)[j*4+3] += g;
//				}
#endif
				
			(*POS)[j] *= deltaT;
			(*VEL)[j] *= deltaT;
		}
	}

	
				(*POS)[0].Nullify();
				(*VEL)[0].Nullify();

				(*POS)[1].Nullify();
				(*VEL)[1].Nullify();
}



void ent_rectangle_torque::applyImpulse(const point& Location, const point& Impulse)
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



} //end of namespace
