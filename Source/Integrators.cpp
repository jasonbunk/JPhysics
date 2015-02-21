#include "stdafx.h"
#include "Integrators.h"
#include <assert.h>


namespace phys
{

	void integrate::BDF2_fromLF(IPhysObject *F, JPHYS_FLOAT_UNIT h, std::vector<point> *POS, std::vector<point> *VEL)
	{
		unsigned int j, size = static_cast<unsigned int>((*POS).size());
		
		std::vector<point> FofNPosplusTwo(size); //make 1 big vector, with #size# parameters
		std::vector<point> FofNVelplusTwo(size); //make 1 big vector, with #size# parameters

		std::vector<point> PosNplusTwo(size); //make 1 big vector, with #size# parameters
		std::vector<point> VelNplusTwo(size); //make 1 big vector, with #size# parameters
		
		for(j=0; j<size; j++) { PosNplusTwo[j] = (*POS)[j];
								VelNplusTwo[j] = (*VEL)[j]; }	//n
		SemiImplicitEuler(F, h, &PosNplusTwo, &VelNplusTwo);	//n+1
		SemiImplicitEuler(F, h, &PosNplusTwo, &VelNplusTwo);	//n+2
		
		for(j=0; j<size; j++) { FofNPosplusTwo[j] = PosNplusTwo[j];
								FofNVelplusTwo[j] = VelNplusTwo[j]; }
		F->f(&FofNPosplusTwo, &FofNVelplusTwo, (h*0.66666667));

		for(j=0; j<size; j++) { (*POS)[j] = (((*POS)[j]*-0.33333333) - PosNplusTwo[j] + FofNPosplusTwo[j])*-0.750;
								(*VEL)[j] = (((*VEL)[j]*-0.33333333) - VelNplusTwo[j] + FofNVelplusTwo[j])*-0.750; }
	}

	void integrate::BDF2_fromRK4(IPhysObject *F, JPHYS_FLOAT_UNIT h, std::vector<point> *POS, std::vector<point> *VEL)
	{
		unsigned int j, size = static_cast<unsigned int>((*POS).size());
		
		std::vector<point> FofNPosplusTwo(size); //make 1 big vector, with #size# parameters
		std::vector<point> FofNVelplusTwo(size); //make 1 big vector, with #size# parameters

		std::vector<point> PosNplusTwo(size); //make 1 big vector, with #size# parameters
		std::vector<point> VelNplusTwo(size); //make 1 big vector, with #size# parameters
		
		for(j=0; j<size; j++) { PosNplusTwo[j] = (*POS)[j];
								VelNplusTwo[j] = (*VEL)[j]; }	//n
		RK4(F, h, &PosNplusTwo, &VelNplusTwo);	//n+1
		RK4(F, h, &PosNplusTwo, &VelNplusTwo);	//n+2
		
		for(j=0; j<size; j++) { FofNPosplusTwo[j] = PosNplusTwo[j];
								FofNVelplusTwo[j] = VelNplusTwo[j]; }
		F->f(&FofNPosplusTwo, &FofNVelplusTwo, (h*0.66666667));

		for(j=0; j<size; j++) { (*POS)[j] = (((*POS)[j]*-0.33333333) - PosNplusTwo[j] + FofNPosplusTwo[j])*-0.750;
								(*VEL)[j] = (((*VEL)[j]*-0.33333333) - VelNplusTwo[j] + FofNVelplusTwo[j])*-0.750; }
	}
	
	void integrate::Trapezoid(IPhysObject *F, JPHYS_FLOAT_UNIT h)
	{
		unsigned int j, size = static_cast<unsigned int>(F->positions.size());
		
		std::vector<point> kPosNplusOne(size); //make 1 big vector, with #size# parameters
		std::vector<point> kVelNplusOne(size); //make 1 big vector, with #size# parameters
		
		std::vector<point> kPosN(size); //make 1 big vector, with #size# parameters
		std::vector<point> kVelN(size); //make 1 big vector, with #size# parameters
		
		for(j=0; j<size; j++) { kPosNplusOne[j] = kPosN[j] = F->positions[j];
								kVelNplusOne[j] = kVelN[j] = F->velocities[j];}

		RK4(F, h, &kPosNplusOne, &kVelNplusOne);	//n+1
		
		F->f(&kPosN, &kVelN, h);
		F->f(&kPosNplusOne,   &kVelNplusOne, h);

		for(j=0; j<size; j++) { F->positions[j]   += ((kPosN[j] + kPosNplusOne[j])/2.0);
								F->velocities[j]  += ((kVelN[j] + kVelNplusOne[j])/2.0); }
	}
	
	void integrate::ABM41(IPhysObject *F, JPHYS_FLOAT_UNIT h)
	{
		unsigned int j, size = static_cast<unsigned int>(F->positions.size());
		
		F->FoPPos.push_back(F->positions);
		F->FoPVel.push_back(F->velocities);	//we will soon "pop_front" as we move along, but for now, add a temporary portion here
		
		F->f(&F->FoPPos[3], &F->FoPVel[3], h); //f of current position, for predictor, and we will save this for future frames, too
		
		for(j=0; j<size; j++) { F->FoPPos[0][j] = F->positions[j] + (((F->FoPPos[0][j]*-9.0) + (F->FoPPos[1][j]*37.0) - (F->FoPPos[2][j]*59.0) + (F->FoPPos[3][j]*55.0)) / 24.0);
								F->FoPVel[0][j] = F->velocities[j]+ (((F->FoPVel[0][j]*-9.0) + (F->FoPVel[1][j]*37.0) - (F->FoPVel[2][j]*59.0) + (F->FoPVel[3][j]*55.0)) / 24.0);
								}
		
		F->f(&F->FoPPos[0], &F->FoPVel[0], h); //f of predicted position (only for corrector; and we no longer need the first part [0] of positions anyway)

		for(j=0; j<size; j++) { F->positions[j] += F->FoPPos[0][j];
								F->velocities[j]+= F->FoPVel[0][j];
								}

		F->FoPPos.pop_front(); F->FoPVel.pop_front(); //we only keep the 3 previous points, so delete the last (it was overwritten by the predictor anyway)
	}
	
	//[0] = n-3
	//[1] = n-2
	//[2] = n-1
	//[3] = n
	//...........[0] = n+1   (during corrector)

	void integrate::ABM22(IPhysObject *F, JPHYS_FLOAT_UNIT h)
	{
		unsigned int j, size = static_cast<unsigned int>(F->positions.size());
		
		F->FoPPos.push_back(F->positions);
		F->FoPVel.push_back(F->velocities);	//we will soon "pop_front" as we move along, but for now, add a temporary portion here
		
		F->f(&F->FoPPos[3], &F->FoPVel[3], h); //f of current position, for predictor, and we will save this for future frames, too
		
		for(j=0; j<size; j++) { F->FoPPos[0][j] = F->positions[j] + (((F->FoPPos[3][j]*3.0) - F->FoPPos[2][j]) * 0.5);
								F->FoPVel[0][j] = F->velocities[j]+ (((F->FoPVel[3][j]*3.0) - F->FoPVel[2][j]) * 0.5);
								}
		
		F->f(&F->FoPPos[0], &F->FoPVel[0], h); //f of predicted position (only for corrector; and we no longer need the first part [0] of positions anyway)
		
		for(j=0; j<size; j++) { F->positions[j] += ((F->FoPPos[3][j] + F->FoPPos[0][j]) * 0.5);
								F->velocities[j]+= ((F->FoPVel[3][j] + F->FoPVel[0][j]) * 0.5);
								}

		F->FoPPos.pop_front(); F->FoPVel.pop_front(); //we only keep the 3 previous points, so delete the last (it was overwritten by the predictor anyway)
	}
	
	void integrate::ABM33(IPhysObject *F, JPHYS_FLOAT_UNIT h)
	{
		unsigned int j, size = static_cast<unsigned int>(F->positions.size());
		
		F->FoPPos.push_back(F->positions);
		F->FoPVel.push_back(F->velocities);	//we will soon "pop_front" as we move along, but for now, add a temporary portion here
		
		F->f(&F->FoPPos[3], &F->FoPVel[3], h); //f of current position, for predictor, and we will save this for future frames, too
		
		for(j=0; j<size; j++) { F->FoPPos[0][j] = F->positions[j] + (((F->FoPPos[1][j]*5.0) - (F->FoPPos[2][j]*16.0) + (F->FoPPos[3][j]*23.0)) / 12.0);
								F->FoPVel[0][j] = F->velocities[j]+ (((F->FoPVel[1][j]*5.0) - (F->FoPVel[2][j]*16.0) + (F->FoPVel[3][j]*23.0)) / 12.0);
								}
		
		F->f(&F->FoPPos[0], &F->FoPVel[0], h); //f of predicted position (only for corrector; and we no longer need the first part [0] of positions anyway)

		for(j=0; j<size; j++) { F->positions[j] += (((F->FoPPos[3][j]*8.0) - F->FoPPos[2][j] + (F->FoPPos[0][j]*5.0)) / 12.0);
								F->velocities[j]+= (((F->FoPVel[3][j]*8.0) - F->FoPVel[2][j] + (F->FoPVel[0][j]*5.0)) / 12.0);
								}

		F->FoPPos.pop_front(); F->FoPVel.pop_front(); //we only keep the 3 previous points, so delete the last (it was overwritten by the predictor anyway)
	}
	
	//[0] = n-3
	//[1] = n-2
	//[2] = n-1
	//[3] = n
	//...........[0] = n+1   (during corrector)

	void integrate::ABM42(IPhysObject *F, JPHYS_FLOAT_UNIT h)
	{
		unsigned int j, size = static_cast<unsigned int>(F->positions.size());
		
		F->FoPPos.push_back(F->positions);
		F->FoPVel.push_back(F->velocities);	//we will soon "pop_front" as we move along, but for now, add a temporary portion here
		
		F->f(&F->FoPPos[3], &F->FoPVel[3], h); //f of current position, for predictor, and we will save this for future frames, too
		
		for(j=0; j<size; j++) { F->FoPPos[0][j] = F->positions[j] + (((F->FoPPos[0][j]*-9.0) + (F->FoPPos[1][j]*37.0) - (F->FoPPos[2][j]*59.0) + (F->FoPPos[3][j]*55.0)) / 24.0);
								F->FoPVel[0][j] = F->velocities[j]+ (((F->FoPVel[0][j]*-9.0) + (F->FoPVel[1][j]*37.0) - (F->FoPVel[2][j]*59.0) + (F->FoPVel[3][j]*55.0)) / 24.0);
								}
		
		F->f(&F->FoPPos[0], &F->FoPVel[0], h); //f of predicted position (only for corrector; and we no longer need the first part [0] of positions anyway)

		for(j=0; j<size; j++) { F->positions[j] += ((F->FoPPos[3][j] + F->FoPPos[0][j]) / 2.0);
								F->velocities[j]+= ((F->FoPVel[3][j] + F->FoPVel[0][j]) / 2.0);
								}

		F->FoPPos.pop_front(); F->FoPVel.pop_front(); //we only keep the 3 previous points, so delete the last (it was overwritten by the predictor anyway)
	}

	void integrate::ABM43(IPhysObject *F, JPHYS_FLOAT_UNIT h)
	{
		unsigned int j, size = static_cast<unsigned int>(F->positions.size());
		
		F->FoPPos.push_back(F->positions);
		F->FoPVel.push_back(F->velocities);	//we will soon "pop_front" as we move along, but for now, add a temporary portion here
		
		F->f(&F->FoPPos[3], &F->FoPVel[3], h); //f of current position, for predictor, and we will save this for future frames, too
		
		for(j=0; j<size; j++) { F->FoPPos[0][j] = F->positions[j] + (((F->FoPPos[0][j]*-9.0) + (F->FoPPos[1][j]*37.0) - (F->FoPPos[2][j]*59.0) + (F->FoPPos[3][j]*55.0)) / 24.0);
								F->FoPVel[0][j] = F->velocities[j]+ (((F->FoPVel[0][j]*-9.0) + (F->FoPVel[1][j]*37.0) - (F->FoPVel[2][j]*59.0) + (F->FoPVel[3][j]*55.0)) / 24.0);
								}
		
		F->f(&F->FoPPos[0], &F->FoPVel[0], h); //f of predicted position (only for corrector; and we no longer need the first part [0] of positions anyway)

		for(j=0; j<size; j++) { F->positions[j] += (((F->FoPPos[3][j]*8.0) - F->FoPPos[2][j] + (F->FoPPos[0][j]*5.0)) / 12.0);
								F->velocities[j]+= (((F->FoPVel[3][j]*8.0) - F->FoPVel[2][j] + (F->FoPVel[0][j]*5.0)) / 12.0);
								}

		F->FoPPos.pop_front(); F->FoPVel.pop_front(); //we only keep the 3 previous points, so delete the last (it was overwritten by the predictor anyway)
	}

	//[0] = n-3
	//[1] = n-2
	//[2] = n-1
	//[3] = n
	//...........[0] = n+1   (during corrector)

	void integrate::ABM44(IPhysObject *F, JPHYS_FLOAT_UNIT h)
	{
		unsigned int j, size = static_cast<unsigned int>(F->positions.size());
		
		F->FoPPos.push_back(F->positions);
		F->FoPVel.push_back(F->velocities);	//we will soon "pop_front" as we move along, but for now, add a temporary portion here
		
		F->f(&F->FoPPos[3], &F->FoPVel[3], h); //f of current position, for predictor, and we will save this for future frames, too
		
		for(j=0; j<size; j++) { F->FoPPos[0][j] = F->positions[j] + (((F->FoPPos[0][j]*-9.0) + (F->FoPPos[1][j]*37.0) - (F->FoPPos[2][j]*59.0) + (F->FoPPos[3][j]*55.0)) / 24.0);
								F->FoPVel[0][j] = F->velocities[j]+ (((F->FoPVel[0][j]*-9.0) + (F->FoPVel[1][j]*37.0) - (F->FoPVel[2][j]*59.0) + (F->FoPVel[3][j]*55.0)) / 24.0);
								}
		
		F->f(&F->FoPPos[0], &F->FoPVel[0], h); //f of predicted position (only for corrector; and we no longer need the first part [0] of positions anyway)

		for(j=0; j<size; j++) { F->positions[j] += ((F->FoPPos[1][j] - (F->FoPPos[2][j]*5.0) + (F->FoPPos[3][j]*19.0) + (F->FoPPos[0][j]*9.0)) / 24.0);
								F->velocities[j]+= ((F->FoPVel[1][j] - (F->FoPVel[2][j]*5.0) + (F->FoPVel[3][j]*19.0) + (F->FoPVel[0][j]*9.0)) / 24.0);
								}

		F->FoPPos.pop_front(); F->FoPVel.pop_front(); //we only keep the 3 previous points, so delete the last (it was overwritten by the predictor anyway)
	}

	void integrate::RK4(IPhysObject *F, JPHYS_FLOAT_UNIT h, std::vector<point> *POS, std::vector<point> *VEL)
	{
		unsigned int j, size = static_cast<unsigned int>((*POS).size());

		std::vector< std::vector<point> > kPos(4, std::vector<point>(size)); //make 4 big vectors, each with #size# parameters
		std::vector< std::vector<point> > kVel(4, std::vector<point>(size)); //make 4 big vectors, each with #size# parameters
		
		for(j=0; j<size; j++) { kPos[0][j] = (*POS)[j];
								kVel[0][j] = (*VEL)[j]; }
		F->f(&kPos[0], &kVel[0], h);
		
		for(j=0; j<size; j++) { kPos[1][j] = (*POS)[j] + (kPos[0][j]/2.0);
								kVel[1][j] = (*VEL)[j] + (kVel[0][j]/2.0); }
		F->f(&kPos[1], &kVel[1], h);
		
		for(j=0; j<size; j++) { kPos[2][j] = (*POS)[j] + (kPos[1][j]/2.0);
								kVel[2][j] = (*VEL)[j] + (kVel[1][j]/2.0); }
		F->f(&kPos[2], &kVel[2], h);
		
		for(j=0; j<size; j++) { kPos[3][j] = (*POS)[j] + kPos[2][j];
								kVel[3][j] = (*VEL)[j] + kVel[2][j]; }
		F->f(&kPos[3], &kVel[3], h);
		
		for(j=0; j<size; j++) { (*POS)[j] += ((kPos[0][j] + kPos[1][j]*2.0 + kPos[2][j]*2.0 + kPos[3][j]) / 6.0);
								(*VEL)[j] += ((kVel[0][j] + kVel[1][j]*2.0 + kVel[2][j]*2.0 + kVel[3][j]) / 6.0); }
	}

	void integrate::RK4(I3DPhysObject *F, JPHYS_FLOAT_UNIT h, std::vector<vec3> *POS, std::vector<vec3> *VEL) //3D
	{
		unsigned int j, size = static_cast<unsigned int>((*POS).size());

		std::vector< std::vector<vec3> > kPos(4, std::vector<vec3>(size)); //make 4 big vectors, each with #size# parameters
		std::vector< std::vector<vec3> > kVel(4, std::vector<vec3>(size)); //make 4 big vectors, each with #size# parameters
		
		for(j=0; j<size; j++) { kPos[0][j] = (*POS)[j];
								kVel[0][j] = (*VEL)[j]; }	//predict based on the starting point
		F->f(&kPos[0], &kVel[0], h);
		
		for(j=0; j<size; j++) { kPos[1][j] = (*POS)[j] + (kPos[0][j]/2.0);		//go halfway (if we went the full way, it'd be Euler's method)
								kVel[1][j] = (*VEL)[j] + (kVel[0][j]/2.0); }	//then predict from halfway
		F->f(&kPos[1], &kVel[1], h);
		
		for(j=0; j<size; j++) { kPos[2][j] = (*POS)[j] + (kPos[1][j]/2.0);		//predict from halfway again, based on previous prediction-from-halfway
								kVel[2][j] = (*VEL)[j] + (kVel[1][j]/2.0); }
		F->f(&kPos[2], &kVel[2], h);
		
		for(j=0; j<size; j++) { kPos[3][j] = (*POS)[j] + kPos[2][j];		//predict from the end, based on the latest prediction-from-halfway
								kVel[3][j] = (*VEL)[j] + kVel[2][j]; }
		F->f(&kPos[3], &kVel[3], h);
		
		for(j=0; j<size; j++) { (*POS)[j] += ((kPos[0][j] + kPos[1][j]*2.0 + kPos[2][j]*2.0 + kPos[3][j]) / 6.0);
								(*VEL)[j] += ((kVel[0][j] + kVel[1][j]*2.0 + kVel[2][j]*2.0 + kVel[3][j]) / 6.0); }
	}
	
	void integrate::RK4(I1DPhysStateObj *F, JPHYS_FLOAT_UNIT h, std::vector<double> *STATE) //"1D"
	{
		unsigned int j, size = static_cast<unsigned int>(STATE->size());

		std::vector< std::vector<double> > kPos(4, std::vector<double>(size)); //make 4 big vectors, each with #size# parameters
		
		for(j=0; j<size; j++) { kPos[0][j] = (*STATE)[j]; }	//predict based on the starting point
		F->f(&kPos[0], h);
		
		for(j=0; j<size; j++) { kPos[1][j] = (*STATE)[j] + (kPos[0][j]/2.0); }	//predict from halfway
		F->f(&kPos[1], h);
		
		for(j=0; j<size; j++) { kPos[2][j] = (*STATE)[j] + (kPos[1][j]/2.0); } //predict from halfway again, based on previous prediction-from-halfway
		F->f(&kPos[2], h);
		
		for(j=0; j<size; j++) { kPos[3][j] = (*STATE)[j] + kPos[2][j]; } //predict from the end, based on the latest prediction-from-halfway
		F->f(&kPos[3], h);
		
		for(j=0; j<size; j++) { (*STATE)[j] += ((kPos[0][j] + kPos[1][j]*2.0 + kPos[2][j]*2.0 + kPos[3][j]) / 6.0); }
	}

	JPHYS_FLOAT_UNIT integrate::RK45_adaptive(I3DPhysObject *F, JPHYS_FLOAT_UNIT h, JPHYS_FLOAT_UNIT epsilon, std::vector<vec3> *POS, std::vector<vec3> *VEL) //3D
	{
		unsigned int j, size = static_cast<unsigned int>((*POS).size());
		
		std::vector< std::vector<vec3> > kPos(6, std::vector<vec3>(size)); //make 6 big vectors, each with #size# parameters
		std::vector< std::vector<vec3> > kVel(6, std::vector<vec3>(size)); //make 6 big vectors, each with #size# parameters
		vec3 est4Pos, est5Pos;
		vec3 est4Vel, est5Vel;
		double idealstepPos, idealstepVel, idealstep;
		int wholeFractionalSteps = 0;
		
		for(j=0; j<size; j++) { kPos[0][j] = (*POS)[j];
								kVel[0][j] = (*VEL)[j]; }
		F->f(&kPos[0], &kVel[0], h);
		
		for(j=0; j<size; j++) { kPos[1][j] = (*POS)[j] + (kPos[0][j]*(1./4.));
								kVel[1][j] = (*VEL)[j] + (kVel[0][j]*(1./4.)); }
		F->f(&kPos[1], &kVel[1], h);
		
		for(j=0; j<size; j++) { kPos[2][j] = (*POS)[j] + (kPos[0][j]*(3./32.)) + (kPos[1][j]*(9./32.));
								kVel[2][j] = (*VEL)[j] + (kVel[0][j]*(3./32.)) + (kVel[1][j]*(9./32.)); }
		F->f(&kPos[2], &kVel[2], h);
		
		for(j=0; j<size; j++) { kPos[3][j] = (*POS)[j] + (kPos[0][j]*(1932./2197.)) - (kPos[1][j]*(7200./2197.)) + (kPos[2][j]*(7296./2197.));
								kVel[3][j] = (*VEL)[j] + (kVel[0][j]*(1932./2197.)) - (kVel[1][j]*(7200./2197.)) + (kVel[2][j]*(7296./2197.)); }
		F->f(&kPos[3], &kVel[3], h);
		
		for(j=0; j<size; j++) { kPos[4][j] = (*POS)[j] + (kPos[0][j]*(439./216.)) - (kPos[1][j]*8.) + (kPos[2][j]*(3680./513.)) - (kPos[3][j]*(845./4104.));
								kVel[4][j] = (*VEL)[j] + (kVel[0][j]*(439./216.)) - (kVel[1][j]*8.) + (kVel[2][j]*(3680./513.)) - (kVel[3][j]*(845./4104.)); }
		F->f(&kPos[4], &kVel[4], h);
		
		for(j=0; j<size; j++) { kPos[5][j] = (*POS)[j] - (kPos[0][j]*(8./27.)) + (kPos[1][j]*2.) - (kPos[2][j]*(3544./2565.)) + (kPos[3][j]*(1859./4104.)) - (kPos[4][j]*(11./40.));
								kVel[5][j] = (*VEL)[j] - (kVel[0][j]*(8./27.)) + (kVel[1][j]*2.) - (kVel[2][j]*(3544./2565.)) + (kVel[3][j]*(1859./4104.)) - (kVel[4][j]*(11./40.)); }
		F->f(&kPos[5], &kVel[5], h);
		
		for(j=0; j<size; j++) {
			est4Pos = (*POS)[j] + (kPos[0][j]*(25./216.)) + (kPos[2][j]*(1408./2565.)) + (kPos[3][j]*(2197./4104.)) - (kPos[4][j]*(1./5.));
			est4Vel = (*VEL)[j] + (kVel[0][j]*(25./216.)) + (kVel[2][j]*(1408./2565.)) + (kVel[3][j]*(2197./4104.)) - (kVel[4][j]*(1./5.));
			
			est5Pos = (*POS)[j] + (kPos[0][j]*(16./135.)) + (kPos[2][j]*(6656./12825.)) + (kPos[3][j]*(28561./56430.)) - (kPos[4][j]*(9./50.)) + (kPos[5][j]*(2./55.));
			est5Vel = (*VEL)[j] + (kVel[0][j]*(16./135.)) + (kVel[2][j]*(6656./12825.)) + (kVel[3][j]*(28561./56430.)) - (kVel[4][j]*(9./50.)) + (kVel[5][j]*(2./55.));
			
			idealstepPos = h*powf((epsilon*h)/(2.0*(est4Pos-est5Pos).getLength()), 0.25);
			idealstepVel = h*powf((epsilon*h)/(2.0*(est4Vel-est5Vel).getLength()), 0.25);
			idealstep = std::min(idealstepPos, idealstepVel);
			if(idealstep < h) {
				wholeFractionalSteps = (h / idealstep);
			}
		}
		return h;
	}

	void integrate::RK2(IPhysObject *F, JPHYS_FLOAT_UNIT h, std::vector<point> *POS, std::vector<point> *VEL)
	{
		unsigned int j, size = static_cast<unsigned int>((*POS).size());

		std::vector< std::vector<point> > kPos(2, std::vector<point>(size)); //make 2 big vectors, each with #size# parameters
		std::vector< std::vector<point> > kVel(2, std::vector<point>(size)); //make 2 big vectors, each with #size# parameters
		
		for(j=0; j<size; j++) { kPos[0][j] = (*POS)[j];
								kVel[0][j] = (*VEL)[j]; }
		F->f(&kPos[0], &kVel[0], h);
		
		for(j=0; j<size; j++) { kPos[1][j] = (*POS)[j] + kPos[0][j];
								kVel[1][j] = (*VEL)[j] + kVel[0][j]; }
		F->f(&kPos[1], &kVel[1], h);

		for(j=0; j<size; j++) { (*POS)[j] += (kPos[0][j] + kPos[1][j]) / 2.0;
								(*VEL)[j] += (kVel[0][j] + kVel[1][j]) / 2.0; }
	}
	
	void integrate::RK2(I3DPhysObject *F, JPHYS_FLOAT_UNIT h, std::vector<vec3> *POS, std::vector<vec3> *VEL)
	{
		unsigned int j, size = static_cast<unsigned int>((*POS).size());

		std::vector< std::vector<vec3> > kPos(2, std::vector<vec3>(size)); //make 2 big vectors, each with #size# parameters
		std::vector< std::vector<vec3> > kVel(2, std::vector<vec3>(size)); //make 2 big vectors, each with #size# parameters
		
		for(j=0; j<size; j++) { kPos[0][j] = (*POS)[j];
								kVel[0][j] = (*VEL)[j]; }
		F->f(&kPos[0], &kVel[0], h);
		
		for(j=0; j<size; j++) { kPos[1][j] = (*POS)[j] + kPos[0][j];
								kVel[1][j] = (*VEL)[j] + kVel[0][j]; }
		F->f(&kPos[1], &kVel[1], h);

		for(j=0; j<size; j++) { (*POS)[j] += (kPos[0][j] + kPos[1][j]) / 2.0;
								(*VEL)[j] += (kVel[0][j] + kVel[1][j]) / 2.0; }
	}

	/*void integrate::Leapfrog(IPhysObject *F, JPHYS_FLOAT_UNIT h, std::vector<point> *POS, std::vector<point> *VEL)
	{
		unsigned int j, size = static_cast<unsigned int>((*POS).size());

		F->bVerlet = !(F->bVerlet);

		if(F->bVerlet)
		{
			for(j=0; j<size; j++) { ((*POS)[j] += ((*VEL)[j] * h)); }
		}
		else
		{
			std::vector<point> kPos(size); //make 1 big vector, with #size# parameters
			std::vector<point> kVel(size); //make 1 big vector, with #size# parameters
			
			for(j=0; j<size; j++) { kPos[j] = (*POS)[j]; }
			F->f(&kPos, &kVel, h);

			for(j=0; j<size; j++) { (*VEL)[j] += kVel[j]; }
		}
	}
	
	void integrate::Leapfrog(I3DPhysObject *F, JPHYS_FLOAT_UNIT h, std::vector<vec3> *POS, std::vector<vec3> *VEL)
	{
		unsigned int j, size = static_cast<unsigned int>((*POS).size());

		F->bVerlet = !(F->bVerlet);

		if(F->bVerlet)
		{
			for(j=0; j<size; j++) { ((*POS)[j] += ((*VEL)[j] * h)); }
		}
		else
		{
			std::vector<vec3> kPos(size); //make 1 big vector, with #size# parameters
			std::vector<vec3> kVel(size); //make 1 big vector, with #size# parameters
			
			for(j=0; j<size; j++) { kPos[j] = (*POS)[j]; }
			F->f(&kPos, &kVel, h);

			for(j=0; j<size; j++) { (*VEL)[j] += kVel[j]; }
		}
	}*/

	void integrate::VVerlet1(IPhysObject *F, JPHYS_FLOAT_UNIT h, std::vector<point> *POS, std::vector<point> *VEL)
	{
		unsigned int j, size = static_cast<unsigned int>((*POS).size());
		
		std::vector<point> kPos(size);
		std::vector<point> kVel(size); //make 2 big vectors, each with #size# parameters
		
		for(j=0; j<size; j++) { kPos[j] = (*POS)[j]; }	// find a(x(t))
		F->f(&kPos, &kVel, h);

		for(j=0; j<size; j++) { (*VEL)[j] += (kVel[j]*0.5);						// v(t+h/2) = v(t) + (h/2)*a(x(t))
								kPos[j]    = ((*POS)[j] += ((*VEL)[j]*h));		}	// x(t+h) = x(t) + v(t+h/2)*h
		
		F->f(&kPos, &kVel, h);				//x is done updating, now finish velocity, so find a(x(t+h))
		
		for(j=0; j<size; j++) { (*VEL)[j] += (kVel[j]*0.5); }	// v(t+h) = v(t+h/2) + (h/2)*a(x(t+h))
	}
	
	void integrate::VVerlet2(IPhysObject *F, JPHYS_FLOAT_UNIT h, std::vector<point> *POS, std::vector<point> *VEL)
	{
		unsigned int j, size = static_cast<unsigned int>((*POS).size());
		
		std::vector<point> kPos(size);
		std::vector< std::vector<point> > kVel(2, std::vector<point>(size)); //make 2 big vectors, each with #size# parameters
		
		for(j=0; j<size; j++) { kPos[j] = (*POS)[j]; }
		F->f(&kPos, &kVel[0], h);

		for(j=0; j<size; j++) { kPos[j] = ((*POS)[j] += ((*VEL)[j]*h) + (kVel[0][j]*0.5*h)); } //x(t+h) = x(t) + v(t)*h + a(t)*0.5*h*h
		
		F->f(&kPos, &kVel[1], h);
		
		for(j=0; j<size; j++) { (*VEL)[j] += ((kVel[0][j] + kVel[1][j])*0.5); }	// v(t+h) = v(t) + (a(t) + a(t+h))*0.5*h
	}
	
	void integrate::SemiImplicitEuler2(IPhysObject *F, JPHYS_FLOAT_UNIT h, std::vector<point> *POS, std::vector<point> *VEL)
	{
		unsigned int j, size = static_cast<unsigned int>((*POS).size());

		std::vector<point> kPos(size); //make 1 big vector, with #size# parameters
		std::vector<point> kVel(size); //make 1 big vector, with #size# parameters
		
		for(j=0; j<size; j++) { kPos[j] = ((*POS)[j] += ((*VEL)[j] * (h*(1-h)))); }
		F->f(&kPos, &kVel, h);

		for(j=0; j<size; j++) { (*VEL)[j] += kVel[j]; }
	}

	void integrate::SemiImplicitEuler(IPhysObject *F, JPHYS_FLOAT_UNIT h, std::vector<point> *POS, std::vector<point> *VEL)
	{
		unsigned int j, size = static_cast<unsigned int>((*POS).size());

		std::vector<point> kPos(size); //make 1 big vector, with #size# parameters
		std::vector<point> kVel(size); //make 1 big vector, with #size# parameters
		
		for(j=0; j<size; j++) { kPos[j] = ((*POS)[j] += ((*VEL)[j] * h)); }
		F->f(&kPos, &kVel, h);

		for(j=0; j<size; j++) { (*VEL)[j] += kVel[j]; }
		
		/*
			for(j=0; j<size; j++) { kPos[j] = (*POS)[j]; }
			F->f(&kPos, &kVel, h);

			for(j=0; j<size; j++) { (*VEL)[j] += kVel[j];
									(*POS)[j] += ((*VEL)[j] * h);
																}
		*/
	}

	void integrate::Euler(IPhysObject *F, JPHYS_FLOAT_UNIT h, std::vector<point> *POS, std::vector<point> *VEL)
	{
		unsigned int j, size = static_cast<unsigned int>((*POS).size());

		std::vector<point> kPos(size); //make 1 big vector, with #size# parameters
		std::vector<point> kVel(size); //make 1 big vector, with #size# parameters
		
		for(j=0; j<size; j++) { kPos[j] = (*POS)[j];
								kVel[j] = (*VEL)[j]; }
		F->f(&kPos, &kVel, h);

		for(j=0; j<size; j++) { (*POS)[j] += kPos[j];
								(*VEL)[j] += kVel[j]; }
	}
	
	void integrate::EulerBackwardsFromSemiImplicitEuler(IPhysObject *F, JPHYS_FLOAT_UNIT h, std::vector<point> *POS, std::vector<point> *VEL)
	{
		unsigned int j, size = static_cast<unsigned int>((*POS).size());

		std::vector<point> TPos(size); //make 1 big vector, with #size# parameters
		std::vector<point> TVel(size); //make 1 big vector, with #size# parameters
		for(j=0; j<size; j++) { TPos[j] = (*POS)[j];
								TVel[j] = (*VEL)[j]; } //n
		SemiImplicitEuler(F, h, &TPos, &TVel);				   //n+1
		
		F->f(&TPos, &TVel, h);

		for(j=0; j<size; j++) { (*POS)[j] += TPos[j];
								(*VEL)[j] += TVel[j]; }

		/*std::vector<point> kPos(size); //make 1 big vector, with #size# parameters
		std::vector<point> kVel(size); //make 1 big vector, with #size# parameters
		
		for(j=0; j<size; j++) { kPos[j] = (TPos[j] += (TVel[j] * h)); }
		F->f(&kPos, &kVel, h);

		for(j=0; j<size; j++) { TVel[j] += kVel[j]; }
		
		//for(j=0; j<size; j++) { (*POS)[j] = TPos[j];
		//						  (*VEL)[j] = TVel[j]; } //if it stopped here, it would be regular leapfrog
		
		F->f(&TPos, &TVel, h);

		for(j=0; j<size; j++) { (*POS)[j] += TPos[j];
								(*VEL)[j] += TVel[j]; }
		*/
	}
	
	void integrate::EulerBackwardsFromEuler(IPhysObject *F, JPHYS_FLOAT_UNIT h, std::vector<point> *POS, std::vector<point> *VEL)
	{
		unsigned int j, size = static_cast<unsigned int>((*POS).size());

		std::vector<point> TPos(size); //make 1 big vector, with #size# parameters
		std::vector<point> TVel(size); //make 1 big vector, with #size# parameters
		for(j=0; j<size; j++) { TPos[j] = (*POS)[j];
								TVel[j] = (*VEL)[j]; } //n
		SemiImplicitEuler(F, h, &TPos, &TVel);				   //n+1
		
		F->f(&TPos, &TVel, h);

		for(j=0; j<size; j++) { TPos[j] = (*POS)[j] + TPos[j];
								TVel[j] = (*VEL)[j] + TVel[j]; }
		
		F->f(&TPos, &TVel, h);
		
		for(j=0; j<size; j++) { (*POS)[j] += TPos[j];
								(*VEL)[j] += TVel[j]; } //n
	}

	void integrate::EulerBackwardsFromRK2(IPhysObject *F, JPHYS_FLOAT_UNIT h, std::vector<point> *POS, std::vector<point> *VEL)
	{
		unsigned int j, size = static_cast<unsigned int>((*POS).size());

		std::vector< std::vector<point> > kPos(2, std::vector<point>(size)); //make 2 big vectors, each with #size# parameters
		std::vector< std::vector<point> > kVel(2, std::vector<point>(size)); //make 2 big vectors, each with #size# parameters
		
		for(j=0; j<size; j++) { kPos[0][j] = (*POS)[j];
								kVel[0][j] = (*VEL)[j]; }
		F->f(&kPos[0], &kVel[0], h);
		
		for(j=0; j<size; j++) { kPos[1][j] = (*POS)[j] + kPos[0][j];
								kVel[1][j] = (*VEL)[j] + kVel[0][j]; }
		F->f(&kPos[1], &kVel[1], h);

		for(j=0; j<size; j++) { kPos[1][j] = (*POS)[j] + ((kPos[0][j] + kPos[1][j]) / 2.0);
								kVel[1][j] = (*VEL)[j] + ((kVel[0][j] + kVel[1][j]) / 2.0); }
		F->f(&kPos[1], &kVel[1], h);

		 //finish RK2, then recalculate using this endpoint for backwards euler
		
		for(j=0; j<size; j++) { (*POS)[j] += kPos[1][j];
								(*VEL)[j] += kVel[1][j]; }
	}
	
	void integrate::EulerBackwardsFromRK4(IPhysObject *F, JPHYS_FLOAT_UNIT h, std::vector<point> *POS, std::vector<point> *VEL)
	{
		
		unsigned int j, size = static_cast<unsigned int>((*POS).size());

		/*std::vector<std::vector<point>> kPos(4, std::vector<point>(size)); //make 4 big vectors, each with #size# parameters
		std::vector<std::vector<point>> kVel(4, std::vector<point>(size)); //make 4 big vectors, each with #size# parameters
		
		for(j=0; j<size; j++) { kPos[0][j] = (*POS)[j];
								kVel[0][j] = (*VEL)[j]; }
		F->f(&kPos[0], &kVel[0], h);
		
		for(j=0; j<size; j++) { kPos[1][j] = (*POS)[j] + (kPos[0][j]/2.0);
								kVel[1][j] = (*VEL)[j] + (kVel[0][j]/2.0); }
		F->f(&kPos[1], &kVel[1], h);
		
		for(j=0; j<size; j++) { kPos[2][j] = (*POS)[j] + (kPos[1][j]/2.0);
								kVel[2][j] = (*VEL)[j] + (kVel[1][j]/2.0); }
		F->f(&kPos[2], &kVel[2], h);
		
		for(j=0; j<size; j++) { kPos[3][j] = (*POS)[j] + kPos[2][j];
								kVel[3][j] = (*VEL)[j] + kVel[2][j]; }
		F->f(&kPos[3], &kVel[3], h);
		
		for(j=0; j<size; j++) { kPos[1][j] = (*POS)[j] + ((kPos[0][j] + kPos[1][j]*2.0 + kPos[2][j]*2.0 + kPos[3][j]) / 6.0);
								kVel[1][j] = (*VEL)[j] + ((kVel[0][j] + kVel[1][j]*2.0 + kVel[2][j]*2.0 + kVel[3][j]) / 6.0); }
		F->f(&kPos[1], &kVel[1], h);

		 //finish RK4, then recalculate using this endpoint for backwards euler
		
		for(j=0; j<size; j++) { (*POS)[j] += kPos[1][j];
								(*VEL)[j] += kVel[1][j]; }
		*/
		
		std::vector<point> PosNplusOne(size); //make 1 big vector, with #size# parameters
		std::vector<point> VelNplusOne(size); //make 1 big vector, with #size# parameters
		
		for(j=0; j<size; j++) { PosNplusOne[j] = (*POS)[j];
								VelNplusOne[j] = (*VEL)[j]; }
		RK4(F, h, &PosNplusOne, &VelNplusOne);
		
		F->f(&PosNplusOne, &VelNplusOne, h); //now that we have the "N plus one" position, use it to calculate euler backwards once
		
		for(j=0; j<size; j++) { (*POS)[j] += PosNplusOne[j];
								(*VEL)[j] += VelNplusOne[j]; }
	}
	

	/*
	// kutta's "3/8ths" RK4 method
	void integrate::simp_RK4_2(IPhysObject *F, const JPHYS_FLOAT_UNIT h, std::vector<JPHYS_FLOAT_UNIT*> *x)
	{
		unsigned int j, size = static_cast<unsigned int>((*x).size());

		std::vector<std::vector<JPHYS_FLOAT_UNIT>> k (4,std::vector<JPHYS_FLOAT_UNIT>(size)); //make 4 big vectors, each with #size# parameters

		for(j=0; j<size; j++) { k[0][j] = *(*x)[j]; }
		F->f(&k[0]);

		for(j=0; j<size; j++) { k[1][j] = (*(*x)[j] + (k[0][j]*=h)/3) ; }
		F->f(&k[1]);

		for(j=0; j<size; j++) { k[2][j] = (*(*x)[j] + (k[1][j]*=h) - k[0][j]/3) ; }
		F->f(&k[2]);

		for(j=0; j<size; j++) { k[3][j] = (*(*x)[j] + (k[2][j]*=h) + k[0][j] - k[1][j]) ; }
		F->f(&k[3]);

		for(j=0; j<size; j++) { *(*x)[j] += (k[0][j] + 3*k[1][j] + 3*k[2][j] + h*k[3][j]) / 8; }
	}
	*/
	
	JPHYS_FLOAT_UNIT integrate::numeric_trapezoid(const std::vector<std::pair<JPHYS_FLOAT_UNIT,JPHYS_FLOAT_UNIT>> & givenPointsFirstIsTimes, int startIdx, int endIdx)
	{
		assert(endIdx < givenPointsFirstIsTimes.size() && startIdx >= 0 && startIdx < endIdx);
		double accumulated = 0.0;
		for(int ii=startIdx; ii<endIdx; ii++) {
			accumulated += 0.5*(givenPointsFirstIsTimes[ii].second + givenPointsFirstIsTimes[ii+1].second)*(givenPointsFirstIsTimes[ii+1].first - givenPointsFirstIsTimes[ii].first);
		}
		return accumulated;
	}
	
	JPHYS_FLOAT_UNIT integrate::numeric_trapezoid(const std::vector<std::vector<double>> & givenPointsFirstIsTimes, int startIdx, int endIdx)
	{
		assert(endIdx < givenPointsFirstIsTimes.size() && startIdx >= 0 && startIdx < endIdx);
		double accumulated = 0.0;
		for(int ii=startIdx; ii<endIdx; ii++) {
			assert(givenPointsFirstIsTimes[ii].size() == 2);
			accumulated += 0.5*(givenPointsFirstIsTimes[ii][1] + givenPointsFirstIsTimes[ii+1][1])*(givenPointsFirstIsTimes[ii+1][0] - givenPointsFirstIsTimes[ii][0]);
		}
		return accumulated;
	}
	
	JPHYS_FLOAT_UNIT integrate::numeric_trapezoid_const_time_step(const std::deque<double> & givenPoints, int startIdx, int endIdx, double dt_per_step)
	{
		assert(endIdx < givenPoints.size() && startIdx >= 0 && startIdx < endIdx);
		double accumulated = 0.0;
		for(int ii=startIdx; ii<endIdx; ii++) {
			accumulated += 0.5*(givenPoints[ii]+givenPoints[ii+1])*dt_per_step;
		}
		return accumulated;
	}
	
	JPHYS_FLOAT_UNIT integrate::numeric_trapezoid(JPHYS_FLOAT_UNIT (*function)(JPHYS_FLOAT_UNIT), JPHYS_FLOAT_UNIT start, JPHYS_FLOAT_UNIT end, unsigned int steps)
	{
		JPHYS_FLOAT_UNIT h = (end-start)/((JPHYS_FLOAT_UNIT)steps);
		JPHYS_FLOAT_UNIT currentvalue = 0.0;
		JPHYS_FLOAT_UNIT currentstep = start;
		
		JPHYS_FLOAT_UNIT ktemps[2];

		ktemps[0] = function(currentstep);
		for(unsigned int jjj=0; jjj<steps; jjj++)
		{
			ktemps[1] = function(currentstep + h);

			currentvalue += (ktemps[0] + ktemps[1]);
			currentstep += h;

			ktemps[0] = ktemps[1];
		}
		
		return (currentvalue * (h/2.0));
	}

	JPHYS_FLOAT_UNIT integrate::numeric_simpson(JPHYS_FLOAT_UNIT (*function)(JPHYS_FLOAT_UNIT), JPHYS_FLOAT_UNIT start, JPHYS_FLOAT_UNIT end, unsigned int steps)
	{
		JPHYS_FLOAT_UNIT h = (end-start)/((JPHYS_FLOAT_UNIT)steps);
		JPHYS_FLOAT_UNIT currentvalue = 0.0;
		JPHYS_FLOAT_UNIT currentstep = start;
		
		JPHYS_FLOAT_UNIT ktemps[3];

		ktemps[0] = function(currentstep);
		for(unsigned int jjj=0; jjj<steps; jjj++)
		{
			ktemps[1] = function(currentstep + (h/2.0));
			ktemps[2] = function(currentstep + h);

			currentvalue += (ktemps[0] + 4.0*ktemps[1] + ktemps[2]);
			currentstep += h;
			
			ktemps[0] = ktemps[2];
		}

		return (currentvalue * (h/6.0));
	}

	JPHYS_FLOAT_UNIT integrate::numeric_simpson5( JPHYS_FLOAT_UNIT (*function)(JPHYS_FLOAT_UNIT), JPHYS_FLOAT_UNIT start, JPHYS_FLOAT_UNIT end, unsigned int steps)
	{
		JPHYS_FLOAT_UNIT h = (end-start)/((JPHYS_FLOAT_UNIT)steps);
		JPHYS_FLOAT_UNIT currentvalue = 0.0;
		JPHYS_FLOAT_UNIT currentstep = start;

		for(unsigned int jjj=0; jjj<steps; jjj++)
		{
			std::vector<JPHYS_FLOAT_UNIT> ktemps(5, 0.0); //make 5 temporary JPHYS_FLOAT_UNITs

			ktemps[0] = function(currentstep);
			ktemps[1] = function(currentstep + (h*0.25));
			ktemps[2] = function(currentstep + (h*0.50));
			ktemps[3] = function(currentstep + (h*0.75));
			ktemps[4] = function(currentstep + h);

			currentvalue += (ktemps[0] + ktemps[1] + (2.0*ktemps[2]) + ktemps[3] + ktemps[4]);
			currentstep += h;
		}

		return (currentvalue * (h/6.0));
	}
}
