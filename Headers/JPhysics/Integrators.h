//-----------------------------------------------------------------
// July 27, 2012
// Jason Bunk
// Integrators.h
//
// Numerical integrators to be used by physics objects' update() functions.
//-----------------------------------------------------------------

#ifndef __INTEGRATORS_H__
#define __INTEGRATORS_H__

#include "exports.h"
#include "Entities.h"
#include "Templates.h"


namespace phys
{
	class integrate
	{
	public:
	
	JPHYSICS_API static void BDF2_fromLF(IPhysObject *F, JPHYS_FLOAT_UNIT h, std::vector<point> *POS, std::vector<point> *VEL);
	
	JPHYSICS_API static void BDF2_fromRK4(IPhysObject *F, JPHYS_FLOAT_UNIT h, std::vector<point> *POS, std::vector<point> *VEL);
	
	JPHYSICS_API static void RK4(IPhysObject *F, JPHYS_FLOAT_UNIT h, std::vector<point> *POS, std::vector<point> *VEL);
	JPHYSICS_API static void RK4(I3DPhysObject *F, JPHYS_FLOAT_UNIT h, std::vector<vec3> *POS, std::vector<vec3> *VEL); //3D
	JPHYSICS_API static void RK4(I1DPhysStateObj *F, JPHYS_FLOAT_UNIT h, std::vector<double> *STATE); //"1D"


	JPHYSICS_API static JPHYS_FLOAT_UNIT RK45_adaptive(I3DPhysObject *F, JPHYS_FLOAT_UNIT h, JPHYS_FLOAT_UNIT epsilon, std::vector<vec3> *POS, std::vector<vec3> *VEL); //3D
	
	JPHYSICS_API static void RK2(IPhysObject *F, JPHYS_FLOAT_UNIT h, std::vector<point> *POS, std::vector<point> *VEL);
	JPHYSICS_API static void RK2(I3DPhysObject *F, JPHYS_FLOAT_UNIT h, std::vector<vec3> *POS, std::vector<vec3> *VEL);
	
	JPHYSICS_API static void Trapezoid(IPhysObject *F, JPHYS_FLOAT_UNIT h);
	
	JPHYSICS_API static void ABM22(IPhysObject *F, JPHYS_FLOAT_UNIT h);
	
	JPHYSICS_API static void ABM33(IPhysObject *F, JPHYS_FLOAT_UNIT h);
	
	JPHYSICS_API static void ABM41(IPhysObject *F, JPHYS_FLOAT_UNIT h);
	
	JPHYSICS_API static void ABM42(IPhysObject *F, JPHYS_FLOAT_UNIT h);
	
	JPHYSICS_API static void ABM43(IPhysObject *F, JPHYS_FLOAT_UNIT h);
	
	JPHYSICS_API static void ABM44(IPhysObject *F, JPHYS_FLOAT_UNIT h);
	
	JPHYSICS_API static void Leapfrog(IPhysObject *F, JPHYS_FLOAT_UNIT h, std::vector<point> *POS, std::vector<point> *VEL);
	JPHYSICS_API static void Leapfrog(I3DPhysObject *F, JPHYS_FLOAT_UNIT h, std::vector<vec3> *POS, std::vector<vec3> *VEL);
	
	JPHYSICS_API static void VVerlet1(IPhysObject *F, JPHYS_FLOAT_UNIT h, std::vector<point> *POS, std::vector<point> *VEL);
	JPHYSICS_API static void VVerlet2(IPhysObject *F, JPHYS_FLOAT_UNIT h, std::vector<point> *POS, std::vector<point> *VEL);
	
	JPHYSICS_API static void SemiImplicitEuler2(IPhysObject *F, JPHYS_FLOAT_UNIT h, std::vector<point> *POS, std::vector<point> *VEL);
	JPHYSICS_API static void SemiImplicitEuler(IPhysObject *F, JPHYS_FLOAT_UNIT h, std::vector<point> *POS, std::vector<point> *VEL);
	
	JPHYSICS_API static void Euler(IPhysObject *F, JPHYS_FLOAT_UNIT h, std::vector<point> *POS, std::vector<point> *VEL);
	
	JPHYSICS_API static void EulerBackwardsFromSemiImplicitEuler(IPhysObject *F, JPHYS_FLOAT_UNIT h, std::vector<point> *POS, std::vector<point> *VEL);
	
	JPHYSICS_API static void EulerBackwardsFromEuler(IPhysObject *F, JPHYS_FLOAT_UNIT h, std::vector<point> *POS, std::vector<point> *VEL);
	
	JPHYSICS_API static void EulerBackwardsFromRK2(IPhysObject *F, JPHYS_FLOAT_UNIT h, std::vector<point> *POS, std::vector<point> *VEL);
	
	JPHYSICS_API static void EulerBackwardsFromRK4(IPhysObject *F, JPHYS_FLOAT_UNIT h, std::vector<point> *POS, std::vector<point> *VEL);
	
	JPHYSICS_API static JPHYS_FLOAT_UNIT numeric_trapezoid(const std::vector<std::pair<JPHYS_FLOAT_UNIT,JPHYS_FLOAT_UNIT> > & givenPointsFirstIsTimes, int startIdx, int endIdx);
	JPHYSICS_API static JPHYS_FLOAT_UNIT numeric_trapezoid(const std::vector<std::vector<double> > & givenPointsFirstIsTimes, int startIdx, int endIdx);
	
	JPHYSICS_API static JPHYS_FLOAT_UNIT numeric_trapezoid_const_time_step(const std::deque<double> & givenPoints, int startIdx, int endIdx, double dt_per_step);
	
	JPHYSICS_API static JPHYS_FLOAT_UNIT numeric_trapezoid(JPHYS_FLOAT_UNIT (*function)(JPHYS_FLOAT_UNIT), JPHYS_FLOAT_UNIT start, JPHYS_FLOAT_UNIT end, unsigned int steps);
	JPHYSICS_API static JPHYS_FLOAT_UNIT numeric_simpson(  JPHYS_FLOAT_UNIT (*function)(JPHYS_FLOAT_UNIT), JPHYS_FLOAT_UNIT start, JPHYS_FLOAT_UNIT end, unsigned int steps);
	JPHYSICS_API static JPHYS_FLOAT_UNIT numeric_simpson5( JPHYS_FLOAT_UNIT (*function)(JPHYS_FLOAT_UNIT), JPHYS_FLOAT_UNIT start, JPHYS_FLOAT_UNIT end, unsigned int steps);
	
	//-----------------------------------------------------------
	// fourth order Runge-Kutta

	// NOTE: time steps of 1/30 second are the best, by far; 60 fps is okay, but about 2-3 times as inaccurate
	//                                                15 fps is not good, like 8-10 times as inaccurate (still way better than euler though; euler is like 10,000 times worse)

/*
	// regular Runge-Kutta
	JPHYSICS_API static void RK4(IPhysObject *F, const float h, std::vector<float*> *POS, std::vector<float*> *VEL);

	JPHYSICS_API static void BackEuler(IPhysObject *F, const float h, std::vector<float*> *POS, std::vector<float*> *VEL);

	JPHYSICS_API static void simp_RK1(IPhysObject *F, const float h, std::vector<float*> *POS, std::vector<float*> *VEL);

	JPHYSICS_API static void simp_RK2(IPhysObject *F, const float h, std::vector<float*> *POS, std::vector<float*> *VEL);

	JPHYSICS_API static void simp_RK3(IPhysObject *F, const float h, std::vector<float*> *POS, std::vector<float*> *VEL);

	// kutta's "3/8ths" RK4 method
	JPHYSICS_API static void simp_RK4_2(IPhysObject *F, const float h, std::vector<float*> *POS, std::vector<float*> *VEL);
*/

	};
}

#endif
